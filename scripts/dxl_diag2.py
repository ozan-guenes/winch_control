#!/usr/bin/env python3
import sys, time
from winch_control.servo_utils_pm54 import ServoController, MODE_CURRENT
# We'll use the SDK objects already exposed by your ServoController
# Addresses for extra diagnostics (Protocol 2.0)
ADDR_MODEL_NUMBER     = 0      # 2 bytes (optional read)
ADDR_TORQUE_ENABLE    = 512     # 1 byte
ADDR_OPERATING_MODE   = 11     # 1 byte
ADDR_CURRENT_LIMIT    = 38     # 2 bytes (EEPROM)
ADDR_PWM_LIMIT        = 36     # 2 bytes (EEPROM)
ADDR_GOAL_CURRENT     = 550    # 2 bytes
ADDR_PRESENT_CURRENT  = 574    # 2 bytes (signed)
ADDR_PRESENT_VOLTAGE  = 592    # 2 bytes (0.1V units)

def rd1(sc, addr):
    val, comm, err = sc.packet_handler.read1ByteTxRx(sc.port_handler, sc.dxl_id, addr)
    if comm != 0 or err != 0: raise RuntimeError(f"read1 addr{addr} c={comm} e={err}")
    return val

def rd2(sc, addr):
    val, comm, err = sc.packet_handler.read2ByteTxRx(sc.port_handler, sc.dxl_id, addr)
    if comm != 0 or err != 0: raise RuntimeError(f"read2 addr{addr} c={comm} e={err}")
    # signed convert for PRESENT_CURRENT
    if addr == ADDR_PRESENT_CURRENT and val >= 0x8000:
        val -= 0x10000
    return val

def wr1(sc, addr, val):
    comm, err = sc.packet_handler.write1ByteTxRx(sc.port_handler, sc.dxl_id, addr, val)
    if comm != 0 or err != 0: raise RuntimeError(f"write1 addr{addr} val{val} c={comm} e={err}")

def wr2(sc, addr, val):
    comm, err = sc.packet_handler.write2ByteTxRx(sc.port_handler, sc.dxl_id, addr, int(val))
    if comm != 0 or err != 0: raise RuntimeError(f"write2 addr{addr} val{val} c={comm} e={err}")

if __name__ == "__main__":
    port  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
    dxl_id= int(sys.argv[3]) if len(sys.argv) > 3 else 4

    print(f"[diag2] port={port} baud={baud} id={dxl_id}")
    sc = ServoController(port, baud, dxl_id, MODE_CURRENT)

    try:
        try:
            model = rd2(sc, ADDR_MODEL_NUMBER)
            print(f"[diag2] model_number={model}")
        except Exception:
            pass

        om = rd1(sc, ADDR_OPERATING_MODE)
        tq = rd1(sc, ADDR_TORQUE_ENABLE)
        cl = rd2(sc, ADDR_CURRENT_LIMIT)
        pwm= rd2(sc, ADDR_PWM_LIMIT)
        pc = rd2(sc, ADDR_PRESENT_CURRENT)
        vin= rd2(sc, ADDR_PRESENT_VOLTAGE) / 10.0

        print(f"[diag2] op_mode={om}  torque_en={tq}  current_limit(raw)={cl}  pwm_limit={pwm}  present_current={pc}  vin={vin:.1f}V")

        # If current limit looks too small, raise it conservatively.
        # NOTE: units are the same raw units your utils use for GOAL_CURRENT.
        if cl < 600:   # ~600 raw ≈ 1.6 A on many X-series (2.69 mA/LSB)
            print(f"[diag2] raising current limit from {cl} to 800 (raw)")
            wr1(sc, ADDR_TORQUE_ENABLE, 0)     # disable torque to change EEPROM
            wr2(sc, ADDR_CURRENT_LIMIT, 800)
            wr1(sc, ADDR_TORQUE_ENABLE, 1)     # enable torque again
            cl = rd2(sc, ADDR_CURRENT_LIMIT)
            print(f"[diag2] new current_limit={cl}")

        # Apply a clear, feelable torque for 2 seconds (wind-in direction assumed +)
        goal = 600  # raw units; ~1.6 A. Adjust if your motor allows more/less.
        print(f"[diag2] set GOAL_CURRENT={goal} (raw)")
        wr2(sc, ADDR_GOAL_CURRENT, goal)

        for i in range(10):
            pc = rd2(sc, ADDR_PRESENT_CURRENT)
            print(f"[diag2] t={0.2*i:.1f}s  present_current={pc}")
            time.sleep(0.2)

        print("[diag2] zero current")
        wr2(sc, ADDR_GOAL_CURRENT, 0)

    finally:
        sc.shutdown()
