#!/usr/bin/env python3
# winch_controller.py
"""
Three-mode winch controller using Dynamixel current control:
 - ASSIST: unwind on pull to maintain min tension; reel-in after slack timeout
 - REEL_IN: force reel-in regardless of pull
 - IDLE: hold light (or zero) tension; no auto unwind or tidy
"""
import atexit
import enum
import time
from collections import deque

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
# from winch_control.servo_utils import ServoController, MODE_CURRENT
from winch_control.servo_utils_pm54 import ServoController, MODE_CURRENT

class Mode(enum.Enum):
    ASSIST  = "ASSIST"
    REEL_IN = "REEL_IN"
    IDLE    = "IDLE"

class WinchControllerNode:
    def __init__(self):
        rospy.init_node("winch_controller", anonymous=False)

        port                     = rospy.get_param    ("~port", "/dev/ttyUSB0")
        baud                     = rospy.get_param    ("~baud", 57600)
        dxl_id                   = rospy.get_param    ("~id", 4)
        self.dir_wind            = int(rospy.get_param("~dir_wind",   +1))
        self.dir_unwind          = int(rospy.get_param("~dir_unwind", -1))

        #--- assist/tension control ----
        # "line tension" proxy is motor current magnitude (mA)
        self.min_tension_mA      = float(rospy.get_param("~min_tension_mA",           300.0))
        self.tension_hyst_mA     = float(rospy.get_param("~tension_hysteresis_mA",     50.0))
        self.assist_unwind_mA    = float(rospy.get_param("~assist_unwind_current_mA", 250.0))
        self.assist_reel_mA      = float(rospy.get_param("~assist_reel_current_mA",   200.0))
        self.idle_hold_mA        = float(rospy.get_param("~idle_hold_current_mA",     100.0))

        # ---- slack detection & tidy ----
        self.slack_timeout_s     = float(rospy.get_param("~slack_timeout_s",  0.6))
        self.slack_tidy_max_s    = float(rospy.get_param("~slack_tidy_max_s", 2.5))

        # ---- Stall (end-stop) guard ----
        self.use_stall_guard     = bool(rospy.get_param("~use_stall_guard", True))
        self.stall_current_mA    = float(rospy.get_param("~stall_current_mA",  1200.0))  # tune to motor/gearbox
        self.stall_speed_deg_s   = float(rospy.get_param("~stall_speed_deg_s",    5.0))    # "near zero"
        self.stall_confirm_s     = float(rospy.get_param("~stall_confirm_s",      0.3))      # sustain time to confirm
        self.stall_backoff_mA    = float(rospy.get_param("~stall_backoff_mA",   150.0))   # gentle peel-off
        self.stall_backoff_time_s= float(rospy.get_param("~stall_backoff_time_s", 0.25))

        # ---- filters & loop ----
        self.rate_hz              = float(rospy.get_param("~rate_hz",             50.0))
        self.ema_alpha_current    = float(rospy.get_param("~ema_alpha_current",   0.25))
        self.ema_alpha_speed      = float(rospy.get_param("~ema_alpha_speed",     0.30))
        self.max_current_slew_mA_s= float(rospy.get_param("~max_current_slew_mA_s", 3000.0))
        
        # ---- Setup motor (CURRENT mode) ----
        self.servo = ServoController(port, baud, dxl_id, MODE_CURRENT)

        # ---- State ----
        self.mode = Mode.ASSIST
        self._last_pull_time  = rospy.Time.now()
        self._tidy_started_at = None
        
        # Filter state
        self._ema_curr      = None
        self._ema_speed     = None
        self._last_pos_deg  = None
        self._last_time     = None
        
        # Stall state
        self._stall_start   = None
        self._stall_end     = None
        self._stall_lock_dir = 0  # +1 = wind, -1 = unwind, 0 = none

        # Command smoothing
        self.cmd_current_mA = 0.0  # smoothed command actually sent

        # ---- ROS ----
        self.mode_sub = rospy.Subscriber("/winch/mode", String, self._mode_cb, queue_size=1)
        self.stop_srv = rospy.Service("/winch/stop", Trigger, self._stop_cb)

        rospy.loginfo("WinchController started, mode = %s", self.mode.value)
        atexit.register(self._safe_shutdown)

    def _mode_cb(self, msg):
        text = (msg.data or "").strip().upper()
        if text in (m.value for m in Mode):
            self.mode = Mode(text)
            rospy.loginfo("Mode -> %s", self.mode.value)
            # reset tidy window when switching
            self._tidy_started_at = None
        else:
            rospy.logwarn("Unknown mode string: '%s' (ignored)", msg.data)

    def _stop_cb(self, _req):
        try:
            self._command_current(0.0)
            return TriggerResponse(success=True, message="Winch stop: current=0")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    # ---- helpers ----
    
    def _ema(self, prev, x, alpha):
        return x if prev is None else (alpha * x + (1.0 - alpha) * prev)
    
    def _read_filtered(self):
        """Return (ema_current_mA, pos_deg, ema_speed_deg_s)."""
        raw_mA = float(self.servo.get_present_current())     
        pos_deg = float(self.servo.get_present_position())  

        now = rospy.Time.now()
        if self._last_time is None:
            dt = 1.0 / max(self.rate_hz, 1.0)
        else:
            dt = (now - self._last_time).to_sec()
            if dt <= 0.0:
                dt = 1.0 / max(self.rate_hz, 1.0)
        self._last_time = now
        
        if self._last_pos_deg is None:
            speed = 0.0
        else:
            speed = (pos_deg - self._last_pos_deg) / dt
        self._last_pos_deg = pos_deg
        
        self._ema_curr  = self._ema(self._ema_curr,  raw_mA, self.ema_alpha_current)
        self._ema_speed = self._ema(self._ema_speed, speed,  self.ema_alpha_speed)

        return self._ema_curr, pos_deg, self._ema_speed

    def _apply_slew(self, desired_mA):
        """Limit current command rate-of-change for smoothness."""
        max_step = self.max_current_slew_mA_s / max(self.rate_hz, 1.0)
        delta = desired_mA - self.cmd_current_mA
        if   delta >  max_step: self.cmd_current_mA +=  max_step
        elif delta < -max_step: self.cmd_current_mA -=  max_step
        else:                   self.cmd_current_mA  = desired_mA
        return self.cmd_current_mA
    
    def _command_current(self, mA):
        smoothed = self._apply_slew(mA)
        self.servo.set_goal_current(int(smoothed))
        
    def _stall_guard(self, desired_current_mA, curr_mA, speed_deg_s, direction_sign):
        """
        Stall/end-stop detection using current spike + near-zero speed.
        direction_sign: +1 for wind-in, -1 for unwind (the *intended* motion).
        """
        if not self.use_stall_guard or direction_sign == 0:
            self._stall_start = None
            self._stall_lock_dir = 0
            return desired_current_mA

        # Keep backoff during its window regardless of new desired command
        now = rospy.Time.now()
        if self._stall_lock_dir != 0 and self._stall_end is not None and now < self._stall_end:
            return -self._stall_lock_dir * abs(self.stall_backoff_mA)
        else:
            self._stall_lock_dir = 0
            self._stall_end = None

        pushing_in_dir = (desired_current_mA * direction_sign) > 0
        if pushing_in_dir:
            hi_current = abs(curr_mA) >= self.stall_current_mA
            low_speed  = abs(speed_deg_s) <= self.stall_speed_deg_s

            if hi_current and low_speed:
                if self._stall_start is None:
                    self._stall_start = now
                elapsed = (now - self._stall_start).to_sec()
                if elapsed >= self.stall_confirm_s:
                    # Confirmed stall: initiate backoff
                    self._stall_lock_dir = direction_sign
                    self._stall_end = now + rospy.Duration(self.stall_backoff_time_s)
                    return -direction_sign * abs(self.stall_backoff_mA)
            else:
                self._stall_start = None

        return desired_current_mA


    # ---- Control Policies ----

    def _policy_idle(self, curr_mA, pos_deg, speed_deg_s):
        # Light inward preload, taut and smooth transitions.
        desired = self.idle_hold_mA * self.dir_wind if self.idle_hold_mA > 0 else 0.0
        desired = self._stall_guard(desired, curr_mA, speed_deg_s, direction_sign=(+1 if desired > 0 else 0))
        self._command_current(desired)

    def _policy_reel_in(self, curr_mA, pos_deg, speed_deg_s):
        desired = abs(self.assist_reel_mA) * self.dir_wind
        desired = self._stall_guard(desired, curr_mA, speed_deg_s, direction_sign=+1)
        self._command_current(desired)

    def _policy_assist(self, curr_mA, pos_deg, speed_deg_s):
        """
        Maintain minimum tension. Intuition:
          - If |curr| > (min + hysteresis): user is pulling => help UNWIND.
          - If |curr| within band: hold small inward bias to keep taut.
          - If |curr| < (min - hysteresis): after slack_timeout, TIDY (REEL IN) for up to slack_tidy_max_s.
        """
        abs_curr = abs(curr_mA)
        band_hi = self.min_tension_mA + self.tension_hyst_mA
        band_lo = self.min_tension_mA - self.tension_hyst_mA
        
        # Detect "pull": above min tension + hysteresis
        if abs_curr > band_hi:
            # Strong pull -> assist by unwinding (reduce tension).
            self._last_pull_time = rospy.Time.now()
            self._tidy_started_at = None
            desired = abs(self.assist_unwind_mA) * self.dir_unwind
            desired = self._stall_guard(desired, curr_mA, speed_deg_s, direction_sign=-1)

        elif band_lo <= abs_curr <= band_hi:
            # In the tension band -> small inward bias to keep taut.
            self._tidy_started_at = None
            desired = 0.5 * abs(self.assist_reel_mA) * self.dir_wind
            desired = self._stall_guard(desired, curr_mA, speed_deg_s, direction_sign=+1)
            
    
        else:
            # Below band (likely slack) -> tidy after timeout
            now = rospy.Time.now()
            since_pull = (now - self._last_pull_time).to_sec()
            if since_pull >= self.slack_timeout_s:
                if self._tidy_started_at is None:
                    self._tidy_started_at = now
                tidy_elapsed = (now - self._tidy_started_at).to_sec()
                if tidy_elapsed <= self.slack_tidy_max_s:
                    desired = abs(self.assist_reel_mA) * self.dir_wind
                else:
                    # end tidy, maintain small hold
                    desired = 0.3 * abs(self.assist_reel_mA) * self.dir_wind
                desired = self._stall_guard(desired, curr_mA, speed_deg_s, direction_sign=+1)
            else:
                # very recent pull — do not fight the user
                desired = 0.0

        self._command_current(desired)


    # ----- Main -----

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                curr_mA, pos_deg, speed_deg_s = self._read_filtered()

                if self.mode is Mode.IDLE:
                    self._policy_idle(curr_mA, pos_deg, speed_deg_s)
                elif self.mode is Mode.REEL_IN:
                    self._policy_reel_in(curr_mA, pos_deg, speed_deg_s)
                else:
                    self._policy_assist(curr_mA, pos_deg, speed_deg_s)

            except Exception as e:
                rospy.logerr_throttle(1.0, "Winch loop error: %s", str(e))
                try:
                    self._command_current(0.0)
                except Exception:
                    pass

            rate.sleep()

    def _safe_shutdown(self):
        try:
            self.servo.set_goal_current(0)
        except Exception:
            pass   

if __name__ == "__main__":
    node = WinchControllerNode()
    node.spin()
