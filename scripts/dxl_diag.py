#!/usr/bin/env python3
import sys, time
import rospy
from winch_control.servo_utils import ServoController, MODE_CURRENT

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
    # dxl_id = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    dxl_id = 4

    print(f"[diag] trying port={port} baud={baud} id={dxl_id}")
    sc = ServoController(port, baud, dxl_id, MODE_CURRENT)
    # If your ServoController raises on failure, you’ll see a traceback above.

    # Try a few reads:
    try:
        pos = sc.get_present_position()
        cur = sc.get_present_current()
        print(f"[diag] position(deg)={pos:.2f}  present_current(raw)={cur}")
    except Exception as e:
        print("[diag] read error:", e); sys.exit(2)

    # Try a tiny current pulse (wind in), then zero:
    try:
        print("[diag] enable small current...")
        sc.set_goal_current(100)   # NOTE: this is RAW LSB in your utils
        time.sleep(1.0)
        sc.set_goal_current(0)
        print("[diag] done")
    except Exception as e:
        print("[diag] write error:", e); sys.exit(3)
