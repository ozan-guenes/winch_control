# ~/winch_ws/src/winch_control/src/winch_control/servo_utils_pm54.py
#!/usr/bin/env python3
"""
servo_utils for DYNAMIXEL-P (PM54-060-S250-R)
- Uses PRO/PRO+ (P-series) control table addresses.
- Goal/Present Current are in mA (signed 16-bit).
- Torque Enable at 512; Operating Mode at 11; Goal Current at 550; Present Current at 574.

API (compatible with your existing calls):
  - ServoController(port, baud, dxl_id, mode=MODE_CURRENT, pulses_per_rev=None)
  - set_operating_mode(mode)
  - enable_torque(on=True) / disable_torque()
  - set_goal_current(mA: int)
  - get_present_current() -> int  (mA, signed)
  - get_present_position() -> float  (degrees; requires pulses_per_rev)
  - get_current_limit_mA() / set_current_limit_mA(mA)
  - set_goal_pwm(pwm_signed) / get_present_pwm()
  - shutdown()
"""

from dynamixel_sdk import PortHandler, PacketHandler

# ---- Operating modes (Protocol 2.0) ----
MODE_CURRENT        = 0
MODE_VELOCITY       = 1
MODE_POSITION       = 3
MODE_EXT_POSITION   = 4
MODE_PWM            = 16

# ---- PM54 / P-series control table (Protocol 2.0) ----
ADDR_MODEL_NUMBER         = 0      # (2B)
ADDR_OPERATING_MODE       = 11     # (1B)
ADDR_PWM_LIMIT            = 36     # (2B)
ADDR_CURRENT_LIMIT        = 38     # (2B) units = mA
ADDR_TORQUE_ENABLE        = 512    # (1B)
ADDR_LED                  = 513    # (1B)
ADDR_GOAL_PWM             = 548    # (2B, signed)
ADDR_GOAL_CURRENT         = 550    # (2B, signed) units = mA
ADDR_GOAL_VELOCITY        = 552    # (4B, signed)
ADDR_GOAL_POSITION        = 564    # (4B, signed)
ADDR_BUS_WATCHDOG         = 546    # (1B) write 0 to clear if tripped
ADDR_PRESENT_PWM          = 572    # (2B, signed)
ADDR_PRESENT_CURRENT      = 574    # (2B, signed) units = mA
ADDR_PRESENT_VELOCITY     = 576    # (4B, signed) 0.01 rev/min
ADDR_PRESENT_POSITION     = 580    # (4B, signed) unit: pulses (encoder ticks)

def _ok(comm, err, where):
    if comm != 0 or err != 0:
        raise RuntimeError(f"{where} failed (c={comm}, e={err})")

def _to_u16_signed(val):
    """pack signed 16-bit into unsigned 0..65535"""
    return val & 0xFFFF

def _from_u16_signed(u):
    """unpack unsigned 0..65535 to signed -32768..32767"""
    return u - 0x10000 if u & 0x8000 else u

def _from_u32_signed(u):
    """unpack unsigned 0..2^32-1 to signed -2^31..2^31-1"""
    return u - 0x100000000 if u & 0x80000000 else u


class ServoController(object):
    def __init__(self, port, baud, dxl_id, mode=MODE_CURRENT, pulses_per_rev=None):
        """
        pulses_per_rev: encoder pulses per mechanical revolution for Present Position → degrees.
                        If None, defaults to 502833.0 (tune for your unit if needed).
        """
        self.port = port
        self.baud = baud
        self.dxl_id = dxl_id
        self.port_handler = PortHandler(self.port)
        if not self.port_handler.openPort():
            raise RuntimeError(f"openPort({self.port}) failed")
        if not self.port_handler.setBaudRate(self.baud):
            raise RuntimeError(f"setBaudRate({self.baud}) failed")
        self.packet_handler = PacketHandler(2.0)  # Protocol 2.0

        # Optional: clear Bus Watchdog (some units block goal writes if tripped)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_BUS_WATCHDOG, 0)

        # Set mode & torque enable
        self.enable_torque(False)
        self.set_operating_mode(mode)
        self.enable_torque(True)

        # cache current limit (mA)
        try:
            self._current_limit_mA = self.get_current_limit_mA()
        except Exception:
            self._current_limit_mA = None

        # position conversion
        self.pulses_per_rev = float(pulses_per_rev) if pulses_per_rev is not None else 502833.0

    # ----------------- Basic control -----------------
    def enable_torque(self, on=True):
        val = 1 if on else 0
        comm, err = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, val)
        _ok(comm, err, "Torque Enable(512)")

    def disable_torque(self):
        self.enable_torque(False)

    def set_operating_mode(self, mode):
        # NOTE: must torque-off for EEPROM writes on some models
        comm, err = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_OPERATING_MODE, int(mode))
        _ok(comm, err, "Operating Mode(11)")

    # ----------------- Current (mA) ------------------
    def set_goal_current(self, milliamp):
        """Set signed current in mA. Clamped to Current Limit if known."""
        val = int(milliamp)
        if self._current_limit_mA is not None:
            lim = int(self._current_limit_mA)
            if val >  lim: val =  lim
            if val < -lim: val = -lim
        u16 = _to_u16_signed(val)
        comm, err = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_CURRENT, u16)
        _ok(comm, err, "Goal Current(550)")

    def get_present_current(self):
        """Return present current in mA (signed)."""
        raw, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_CURRENT)
        _ok(comm, err, "Present Current(574)")
        return _from_u16_signed(raw)

    def get_current_limit_mA(self):
        raw, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, ADDR_CURRENT_LIMIT)
        _ok(comm, err, "Current Limit(38)")
        return _from_u16_signed(raw)  # limit is non-negative, but return int for consistency

    def set_current_limit_mA(self, milliamp):
        """Set current limit (mA). Torque must be disabled."""
        self.disable_torque()
        mA = max(0, int(milliamp))
        comm, err = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, ADDR_CURRENT_LIMIT, mA & 0xFFFF)
        _ok(comm, err, "Set Current Limit(38)")
        self.enable_torque(True)
        self._current_limit_mA = mA

    # ----------------- PWM (optional) ----------------
    def set_goal_pwm(self, pwm_signed):
        """Signed PWM command (range limited by PWM Limit)."""
        comm, err = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_PWM, _to_u16_signed(int(pwm_signed)))
        _ok(comm, err, "Goal PWM(548)")

    def get_present_pwm(self):
        raw, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_PWM)
        _ok(comm, err, "Present PWM(572)")
        return _from_u16_signed(raw)

    # ----------------- Position / Velocity ----------
    def get_present_position(self):
        """
        Return present position in degrees.
        Uses pulses_per_rev for conversion; tune pulses_per_rev if needed.
        """
        raw, comm, err = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        _ok(comm, err, "Present Position(580)")
        signed = _from_u32_signed(raw)
        deg = (signed / self.pulses_per_rev) * 360.0
        return deg

    def get_present_velocity_rpm(self):
        """Return velocity in RPM (approx; Present Velocity unit is 0.01 rev/min)."""
        raw, comm, err = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_VELOCITY)
        _ok(comm, err, "Present Velocity(576)")
        signed = _from_u32_signed(raw)
        rpm = (signed * 0.01)
        return rpm

    # ----------------- Housekeeping -----------------
    def shutdown(self):
        try:
            self.set_goal_current(0)
        except Exception:
            pass
        try:
            self.enable_torque(False)
        except Exception:
            pass
        try:
            self.port_handler.closePort()
        except Exception:
            pass
