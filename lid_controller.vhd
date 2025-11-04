-- ============================================================================
--  Lid Controller for UVC MagSafe Sanitizer (servo-driven lid)
--  - Generates 50 Hz servo PWM (1.0–2.0 ms) to open/close the lid
--  - Debounced open/close buttons
--  - Emergency stop (hard disable)
--  - Lid-closed interlock to allow UV only when closed & safe
--  - Open/close timeouts & simple fault handling
--
--  Map to your board pins:
--    clk           : system clock (e.g., 50 MHz)
--    rst_n         : active-low reset
--    open_btn      : request to open lid (debounced internally)
--    close_btn     : request to close lid (debounced internally)
--    lid_closed_sw : 1 when lid is physically confirmed closed
--    e_stop        : 1 triggers emergency stop
--    uv_req        : request to enable UV (software/higher-level)
--    servo_pwm     : PWM output to servo signal line
--    uv_enable     : goes high only when lid closed & safe
--    fault_led     : lights on fault
--    state_debug   : 3-bit state export for debug LEDs/ILA
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity lid_controller is
  generic (
    G_CLK_HZ       : integer := 50_000_000; -- system clock
    G_SERVO_HZ     : integer := 50;         -- 50 Hz servo period
    -- Servo pulse limits (in microseconds)
    G_PULSE_MIN_US : integer := 1000;       -- fully closed (~1.0 ms)
    G_PULSE_MAX_US : integer := 2000;       -- fully open   (~2.0 ms)
    -- Target positions (in microseconds)
    G_CLOSED_US    : integer := 1000;
    G_OPEN_US      : integer := 1800;       -- a bit shy of max to avoid bind
    -- Debounce time for buttons (ms)
    G_DEBOUNCE_MS  : integer := 20;
    -- Timeouts (ms)
    G_OPEN_TIMEOUT_MS  : integer := 1500;   -- how long we allow opening
    G_CLOSE_TIMEOUT_MS : integer := 1500
  );
  port (
    clk           : in  std_logic;
    rst_n         : in  std_logic;

    open_btn      : in  std_logic;  -- async, active-high
    close_btn     : in  std_logic;  -- async, active-high
    lid_closed_sw : in  std_logic;  -- 1 when physically closed
    e_stop        : in  std_logic;  -- 1 = emergency stop
    uv_req        : in  std_logic;  -- request from host to enable UV

    servo_pwm     : out std_logic;
    uv_enable     : out std_logic;
    fault_led     : out std_logic;
    state_debug   : out std_logic_vector(2 downto 0)
  );
end entity;

architecture rtl of lid_controller is

  -- =======================
  -- Clock-derived constants
  -- =======================
  constant C_PERIOD_TICKS : integer := G_CLK_HZ / G_SERVO_HZ;
  -- 1 us worth of ticks:
  constant C_TICKS_PER_US : integer := G_CLK_HZ / 1_000_000;

  -- Debounce
  constant C_DB_TICKS : integer := (G_DEBOUNCE_MS * G_CLK_HZ) / 1000;

  -- Timeouts in ticks
  constant C_OPEN_TO_TICKS  : integer := (G_OPEN_TIMEOUT_MS  * G_CLK_HZ) / 1000;
  constant C_CLOSE_TO_TICKS : integer := (G_CLOSE_TIMEOUT_MS * G_CLK_HZ) / 1000;

  -- =======================
  -- State machine
  -- =======================
  type t_state is (
    S_CLOSED,      -- lid down
    S_OPENING,     -- driving to open
    S_OPEN,        -- lid up
    S_CLOSING,     -- driving to close
    S_FAULT,       -- timed out or invalid feedback
    S_ESTOP        -- emergency stop asserted
  );
  signal state, nstate : t_state;

  -- =======================
  -- Button debouncers
  -- =======================
  -- two-flop synchronizers + counters
  signal open_sync1, open_sync2  : std_logic := '0';
  signal close_sync1, close_sync2: std_logic := '0';
  signal open_db, close_db       : std_logic := '0';
  signal open_cnt, close_cnt     : integer range 0 to C_DB_TICKS := 0;

  -- =======================
  -- Servo PWM
  -- =======================
  signal pwm_cnt     : integer range 0 to C_PERIOD_TICKS-1 := 0;
  signal target_us   : integer := G_CLOSED_US;
  signal high_ticks  : integer := G_CLOSED_US * C_TICKS_PER_US;

  -- =======================
  -- Timers / fault
  -- =======================
  signal action_timer : integer range 0 to integer'high := 0;
  signal fault        : std_logic := '0';

begin

  -- ============================================================
  -- Synchronize and debounce open_btn
  -- ============================================================
  process(clk, rst_n)
  begin
    if rst_n = '0' then
      open_sync1 <= '0'; open_sync2 <= '0';
      open_cnt   <= 0;   open_db    <= '0';
    elsif rising_edge(clk) then
      open_sync1 <= open_btn;
      open_sync2 <= open_sync1;
      if open_sync2 = open_db then
        open_cnt <= 0;
      else
        if open_cnt < C_DB_TICKS then
          open_cnt <= open_cnt + 1;
        else
          open_db  <= open_sync2;
          open_cnt <= 0;
        end if;
      end if;
    end if;
  end process;

  -- ============================================================
  -- Synchronize and debounce close_btn
  -- ============================================================
  process(clk, rst_n)
  begin
    if rst_n = '0' then
      close_sync1 <= '0'; close_sync2 <= '0';
      close_cnt   <= 0;   close_db    <= '0';
    elsif rising_edge(clk) then
      close_sync1 <= close_btn;
      close_sync2 <= close_sync1;
      if close_sync2 = close_db then
        close_cnt <= 0;
      else
        if close_cnt < C_DB_TICKS then
          close_cnt <= close_cnt + 1;
        else
          close_db  <= close_sync2;
          close_cnt <= 0;
        end if;
      end if;
    end if;
  end process;

  -- ============================================================
  -- Servo PWM period counter and pulse generation
  -- ============================================================
  process(clk, rst_n)
  begin
    if rst_n = '0' then
      pwm_cnt    <= 0;
      servo_pwm  <= '0';
      high_ticks <= G_CLOSED_US * C_TICKS_PER_US;
    elsif rising_edge(clk) then
      -- Limit/clip the requested pulse width to safe bounds
      if target_us < G_PULSE_MIN_US then
        high_ticks <= G_PULSE_MIN_US * C_TICKS_PER_US;
      elsif target_us > G_PULSE_MAX_US then
        high_ticks <= G_PULSE_MAX_US * C_TICKS_PER_US;
      else
        high_ticks <= target_us * C_TICKS_PER_US;
      end if;

      if pwm_cnt = C_PERIOD_TICKS - 1 then
        pwm_cnt <= 0;
      else
        pwm_cnt <= pwm_cnt + 1;
      end if;

      if pwm_cnt < high_ticks then
        servo_pwm <= '1';
      else
        servo_pwm <= '0';
      end if;
    end if;
  end process;

  -- ============================================================
  -- State machine: next-state logic
  -- ============================================================
  process(state, open_db, close_db, lid_closed_sw, e_stop, fault, action_timer)
  begin
    nstate <= state;
    case state is
      when S_ESTOP =>
        if e_stop = '0' then
          -- Recover to safe known position: if closed switch says closed, go CLOSED; else go CLOSING.
          if lid_closed_sw = '1' then
            nstate <= S_CLOSED;
          else
            nstate <= S_CLOSING;
          end if;
        end if;

      when S_FAULT =>
        -- Wait for user to press close, then attempt to close again
        if e_stop = '1' then
          nstate <= S_ESTOP;
        elsif close_db = '1' then
          nstate <= S_CLOSING;
        end if;

      when S_CLOSED =>
        if e_stop = '1' then
          nstate <= S_ESTOP;
        elsif open_db = '1' then
          nstate <= S_OPENING;
        end if;

      when S_OPENING =>
        if e_stop = '1' then
          nstate <= S_ESTOP;
        elsif action_timer >= C_OPEN_TO_TICKS then
          nstate <= S_FAULT; -- took too long
        elsif open_db = '0' and close_db = '1' then
          nstate <= S_CLOSING; -- user changed mind
        elsif lid_closed_sw = '1' then
          -- still closed; keep opening until timer hits; rely on mechanical travel
          nstate <= S_OPENING;
        else
          -- switch went low (no longer closed). Assume lid is up enough -> consider OPEN.
          nstate <= S_OPEN;
        end if;

      when S_OPEN =>
        if e_stop = '1' then
          nstate <= S_ESTOP;
        elsif close_db = '1' then
          nstate <= S_CLOSING;
        end if;

      when S_CLOSING =>
        if e_stop = '1' then
          nstate <= S_ESTOP;
        elsif action_timer >= C_CLOSE_TO_TICKS then
          nstate <= S_FAULT; -- took too long
        elsif lid_closed_sw = '1' then
          nstate <= S_CLOSED;
        end if;
    end case;
  end process;

  -- ============================================================
  -- State machine: sequential + timers + target position
  -- ============================================================
  process(clk, rst_n)
  begin
    if rst_n = '0' then
      state        <= S_CLOSED;
      action_timer <= 0;
      target_us    <= G_CLOSED_US;
      fault        <= '0';
    elsif rising_edge(clk) then
      if state /= nstate then
        action_timer <= 0;
      else
        if (state = S_OPENING) or (state = S_CLOSING) then
          if action_timer < integer'high then
            action_timer <= action_timer + 1;
          end if;
        else
          action_timer <= 0;
        end if;
      end if;

      -- Drive target positions by state
      case nstate is
        when S_ESTOP  => target_us <= G_CLOSED_US; fault <= '0';
        when S_FAULT  => target_us <= G_CLOSED_US; fault <= '1';
        when S_CLOSED => target_us <= G_CLOSED_US; fault <= '0';
        when S_OPEN   => target_us <= G_OPEN_US;   fault <= '0';
        when S_OPENING=> target_us <= G_OPEN_US;   fault <= '0';
        when S_CLOSING=> target_us <= G_CLOSED_US; fault <= '0';
      end case;

      state <= nstate;
    end if;
  end process;

  -- ============================================================
  -- UV enable: only when (requested AND closed AND safe)
  -- ============================================================
  uv_enable <= '1' when (uv_req = '1' and lid_closed_sw = '1'
                         and state = S_CLOSED and e_stop = '0' and fault = '0')
              else '0';

  fault_led  <= fault;
  -- Optional: export state for LED/ILA
  with state select
    state_debug <= "000" when S_CLOSED,
                   "001" when S_OPENING,
                   "010" when S_OPEN,
                   "011" when S_CLOSING,
                   "100" when S_FAULT,
                   "101" when S_ESTOP,
                   "111" when others;

end architecture;
