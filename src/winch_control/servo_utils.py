#!/usr/bin/env python

import atexit
import rospy
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Control-table addresses (Protocol 2.0)
ADDR_TORQUE_ENABLE    = 512    # RW, 1 byte
ADDR_OPERATING_MODE   = 11    # RW, 1 byte (EEPROM)
ADDR_GOAL_CURRENT     = 550   # RW, 2 bytes, mA
ADDR_GOAL_POSITION    = 564   # RW, 4 bytes, ticks
ADDR_PROFILE_VELOCITY = 560   # RW, 4 bytes, ticks/sec
ADDR_PRESENT_POSITION = 580   # R,  4 bytes, ticks
ADDR_PRESENT_CURRENT  = 574   # R,  2 bytes, mA

# Operating modes
MODE_CURRENT            = 0
MODE_VELOCITY           = 1
MODE_POSITION           = 3
MODE_EXTENDED_POSITION  = 4
MODE_CURRENT_POSITION   = 5

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

class ServoController:
    def __init__(self, port, baudrate, dxl_id, mode):       # port: str; *: int
        self.dxl_id = dxl_id                                
        self.port_handler = PortHandler(port)               
        if not self.port_handler.openPort():
            raise IOError("Cannot open port {}".format(port))
        if not self.port_handler.setBaudRate(baudrate):
            raise IOError("Cannot set baudrate {}".format(baudrate))
        self.packet_handler = PacketHandler(2.0)

        self.disable_torque()
        self.set_operating_mode(mode)
        self.enable_torque()

        # on shutdown, disable torque & close port
        atexit.register(self.shutdown)
        
    def disable_torque(self):
        comm, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Disable torque failed (c={},e={})".format(comm, err))
        
    def set_operating_mode(self, mode):
        comm, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_OPERATING_MODE, mode)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Set mode {} failed (c={},e={})".format(mode, comm, err))
        
    def enable_torque(self):
        comm, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Enable torque failed (c={},e={})".format(comm, err))

    def set_goal_current(self, mA, print = False):
        comm, err = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_GOAL_CURRENT, mA)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Set goal current {} mA failed (c={},e={})".format(mA, comm, err))
        else:
            if print: rospy.loginfo("Goal Current: {} mA".format(mA))

    def set_goal_position(self, ticks):
        comm, err = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, ticks)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Set goal position {} ticks failed (c={},e={})".format(ticks, comm, err))
        # else:
        #     rospy.loginfo("Goal Position: {} deg".format(self.ticks_to_deg(ticks)))

    def set_profile_velocity(self, ticks_per_sec):
        comm, err = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, ticks_per_sec)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError("Set profile vel {} failed (c={},e={})".format(ticks_per_sec, comm, err))
        else:
            rospy.loginfo("Profile Vel: {} deg/s".format(self.ticks_to_deg(ticks_per_sec)))
            
    def ticks_to_deg(self, ticks):
        """0-4095 ticks --> 0-360°"""
        return int(ticks * 360.0 / 4096.0)

    def get_present_current(self):
        raw, comm, err = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PRESENT_CURRENT)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Read present current failed (c={comm},e={err})")
        # else: 
        #     print("Current: {} mA".format(raw if raw < 0x8000 else raw - 0x10000))
        # convert unsigned to signed 16-bit
        return raw if raw < 0x8000 else raw - 0x10000
     
    def get_present_position(self):
        ticks, comm, err = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        if comm!=COMM_SUCCESS or err!=0:
            raise RuntimeError(f"Read present position failed (c={comm},e={err})")
        else:
            pos_deg = self.ticks_to_deg(ticks)
        return pos_deg
    
    def shutdown(self):
        try:
            self.disable_torque()
        except Exception:
            pass
        self.port_handler.closePort()