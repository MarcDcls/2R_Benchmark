from dynamixel_sdk import *
import numpy as np

BAUDRATE         = 3000000
PROTOCOL_VERSION = 2.0
DEVICENAME       = '/dev/ttyUSB0'

DXL_IDS = [1, 2]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

ADDR_RETURN_DELAY     = 9
ADDR_CONTROL_MODE     = 11
ADDR_MOVING_TRESHOLD  = 24
ADDR_TORQUE_ENABLE    = 64
ADDR_RETURN_STATUS    = 68
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

CURRENT_CONTROL_MODE                = 0
POSITION_CONTROL_MODE               = 3
CURRENT_BASED_POSITION_CONTROL_MODE = 5

RETURN_STATUS_PING = 0
RETURN_STATUS_PING_READ = 1
RETURN_STATUS_ALL = 2

COMM_SUCCESS = 0
COMM_TX_FAIL = -1001

DXL_MINIMUM_POSITION_VALUE = 0
DXL_ZERO_POSITION_VALUE    = 2048
DXL_MAXIMUM_POSITION_VALUE = 4095


def init_connection():
    """Initialize the connection with the motors"""
    print("Initializing connection...")

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
    
    # Disabling torque (in case of a previous interrupted connection)
    disable_torque()
    

def close_connection():
    """Close the connection with the motors"""
    portHandler.closePort()
    print("Connection closed")

def enable_torque(ids=DXL_IDS):
    """Enable the torque of the motors"""
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Motor with ID {id} has been successfully connected")

def disable_torque(ids=DXL_IDS):
    """Disable the torque of the motors"""
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Motor with ID {id} has been successfully disconnected")

def set_control_mode(mode, ids=DXL_IDS):
    """Set the control mode of the motors"""
    print("Setting control mode...")
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, id, ADDR_CONTROL_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Control mode set!")

def set_return_delay_time(delay, ids=DXL_IDS):
    """Set the return delay time of the motors"""
    print("Setting return delay time...")
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, id, ADDR_RETURN_DELAY, delay)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Return delay time set!")

def set_moving_threshold(threshold, ids=DXL_IDS):
    """Set the moving threshold of the motors"""
    print("Setting moving threshold...")
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, id, ADDR_MOVING_TRESHOLD, threshold)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Moving threshold set!")

def set_return_status(status, ids=DXL_IDS):
    """Set the return status of the motors"""
    print("Setting return status level...")
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, id, ADDR_RETURN_STATUS, status)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Return status level set!")

def set_position(id, position, write_only=True):
    """Set the position (rad) of the motor"""
    # Convert degree to index
    index = round(position * (DXL_MAXIMUM_POSITION_VALUE+1) / (2*np.pi)) + DXL_ZERO_POSITION_VALUE
    if index > DXL_MAXIMUM_POSITION_VALUE:
        index = DXL_MAXIMUM_POSITION_VALUE
    elif index < DXL_MINIMUM_POSITION_VALUE:
        index = DXL_MINIMUM_POSITION_VALUE

    # Write goal position
    if write_only:
        packetHandler.write4ByteTxOnly(
            portHandler, id, ADDR_GOAL_POSITION, index)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, id, ADDR_GOAL_POSITION, index)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False
        return True

def get_position(id):
    """Get the position (rad) of the motor"""
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert index to degree
        position = dxl_present_position * (2*np.pi) / \
            (DXL_MAXIMUM_POSITION_VALUE+1) - np.pi
        return position
