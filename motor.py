from dynamixel_sdk import *
import numpy as np

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

ADDR_CONTROL_MODE                   = 11
CURRENT_CONTROL_MODE                = 0
POSITION_CONTROL_MODE               = 3
CURRENT_BASED_POSITION_CONTROL_MODE = 5

DXL_MINIMUM_POSITION_VALUE  = 0
DXL_ZERO_POSITION_VALUE     = 2048
DXL_MAXIMUM_POSITION_VALUE  = 4095

DXL_IDS                     = []

DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = None

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = None

def init_connection(port, baudrate=57600, protocol_version=2.0):
    """Initialize the connection with the motors"""
    global portHandler
    global packetHandler
    global BAUDRATE

    portHandler = PortHandler(port)
    packetHandler = PacketHandler(protocol_version)
    BAUDRATE = baudrate
    
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
        
def set_motors_ids(ids):
    """Set the motors ids"""
    global DXL_IDS
    DXL_IDS = ids

def enable_torque(ids=DXL_IDS):
    """Enable the torque of the motors"""
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Motor with ID {id} has been successfully connected")

def disable_torque(ids=DXL_IDS):
    """Disable the torque of the motors"""
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Motor with ID {id} has been successfully disconnected")

def set_control_mode(ids, mode):
    """Set the control mode of the motors"""
    for id in ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


def set_position(id, position, write_only=True):
    """Set the position (degree) of the motor"""
    # Convert degree to index
    index = round(position * (DXL_MAXIMUM_POSITION_VALUE+1) / 360) + DXL_ZERO_POSITION_VALUE
    if index > DXL_MAXIMUM_POSITION_VALUE:
        index = DXL_MAXIMUM_POSITION_VALUE
    elif index < DXL_MINIMUM_POSITION_VALUE:
        index = DXL_MINIMUM_POSITION_VALUE

    # Write goal position
    if write_only:
        packetHandler.write4ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, index)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, index)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def get_position(id):
    """Get the position (degree) of the motor"""
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert index to degree
        position = dxl_present_position * 360 / (DXL_MAXIMUM_POSITION_VALUE+1) - 180
        return position

def close_connection():
    """Close the connection with the motors"""
    portHandler.closePort()


