from dynamixel_sdk import *
import numpy as np

BAUDRATE         = 3000000
PROTOCOL_VERSION = 2.0
DEVICENAME       = '/dev/ttyUSB0'

DXL_IDS = [1, 2]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

ADDR_RETURN_DELAY          = 9
ADDR_CONTROL_MODE          = 11
ADDR_MOVING_TRESHOLD       = 24
ADDR_TORQUE_ENABLE         = 64
ADDR_RETURN_STATUS         = 68
ADDR_GOAL_PWM              = 100
ADDR_GOAL_CURRENT          = 102
ADDR_GOAL_VELOCITY         = 104
ADDR_GOAL_POSITION         = 116
ADDR_PRESENT_PWM           = 124
ADDR_PRESENT_CURRENT       = 126
ADDR_PRESENT_VELOCITY      = 128
ADDR_PRESENT_POSITION      = 132

CURRENT_CONTROL_MODE                = 0
VELOCITY_CONTROL_MODE               = 1
POSITION_CONTROL_MODE               = 3
CURRENT_BASED_POSITION_CONTROL_MODE = 5
PWM_CONTROL_MODE                    = 16

RETURN_STATUS_PING = 0
RETURN_STATUS_PING_READ = 1
RETURN_STATUS_ALL = 2

COMM_SUCCESS = 0
COMM_TX_FAIL = -1001

DXL_MINIMUM_POSITION_VALUE = 0
DXL_ZERO_POSITION_VALUE    = 2048
DXL_MAXIMUM_POSITION_VALUE = 4095

    
def init_connection():
    """ Initialize the connection with the motors """
    print("Initializing connection...")

    if len(DXL_IDS) == 0:
        print("No motors to connect ! Please add entries to the DXL_IDS list in motor.py")
        exit()

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
    """ Close the connection with the motors """
    portHandler.closePort()
    print("Connection closed")
    
def enable_torque(ids=DXL_IDS, write_only=True):
    """ Enable the torque of the motors """
    for id in ids:
        if write_only:
            packetHandler.write1ByteTxOnly(portHandler, id, ADDR_TORQUE_ENABLE, 1)
        else:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Motor with ID {id} has been successfully connected")

def disable_torque(ids=DXL_IDS, write_only=True):
    """ Disable the torque of the motors """
    for id in ids:
        if write_only:
            packetHandler.write1ByteTxOnly(portHandler, id, ADDR_TORQUE_ENABLE, 0)
        else:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Motor with ID {id} has been successfully disconnected")

def set_control_mode(mode, ids=DXL_IDS, write_only=True):
    """ Set the control mode of the motors """
    print("Setting control mode...")
    for id in ids:
        if write_only:
            packetHandler.write1ByteTxOnly(portHandler, id, ADDR_CONTROL_MODE, mode)
        else:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_CONTROL_MODE, mode)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Control mode set!")

def set_return_delay_time(delay, ids=DXL_IDS, write_only=True):
    """ Set the return delay time of the motors """
    print("Setting return delay time...")
    for id in ids:
        if write_only:
            packetHandler.write1ByteTxOnly(portHandler, id, ADDR_RETURN_DELAY, delay)
        else:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_RETURN_DELAY, delay)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Return delay time set!")

def set_moving_threshold(threshold, ids=DXL_IDS, write_only=True):
    """ Set the moving threshold of the motors """
    print("Setting moving threshold...")
    for id in ids:
        if write_only:
            packetHandler.write4ByteTxOnly(portHandler, id, ADDR_MOVING_TRESHOLD, threshold)
        else:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_MOVING_TRESHOLD, threshold)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Moving threshold set!")

def set_return_status(status, ids=DXL_IDS, write_only=True):
    """ Set the return status of the motors """
    print("Setting return status level...")
    for id in ids:
        if write_only:
            packetHandler.write1ByteTxOnly(portHandler, id, ADDR_RETURN_STATUS, status)
        else:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_RETURN_STATUS, status)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("Return status level set!")

def set_position(id, position, write_only=True):
    """ Set the position (rad) of the motor """
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
    """ Get the position (rad) of the motor """
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert index to radian
        position = dxl_present_position * (2*np.pi) / (DXL_MAXIMUM_POSITION_VALUE+1) - np.pi
        return position
    
def set_velocity(id, velocity, use_rpm=False, write_only=True):
    """ Set the velocity (rad/s) of the motor """
    # Convert rad/s or rpm to index
    index = unsign(round(velocity / 0.229) if use_rpm else round(velocity * 60 / (2*np.pi) / 0.229), 4)

    # Write goal position
    if write_only:
        packetHandler.write4ByteTxOnly(portHandler, id, ADDR_GOAL_VELOCITY, index)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, index)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False
        return True
    
def get_velocity(id, use_rpm=False):
    """ Get the velocity (rad/s) of the motor """
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, id, ADDR_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        # Convert index to rad/s or rpm
        velocity = sign(dxl_present_velocity, 4) * 0.229 if use_rpm else sign(dxl_present_velocity, 4) * 0.229 * 2*np.pi / 60
        return velocity

def set_current(id, current, write_only=True):
    """ Set the current (mA) of the motor """
    # Convert mA to index
    index = unsign(round(current / 3.36), 2)

    # Write goal position
    if write_only:
        packetHandler.write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, index)
    else:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, index)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False
        return True

def get_current(id):
    """ Get the current (mA) of the motor """
    index, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id, ADDR_PRESENT_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        current = sign(index, 2) * 3.36
        return current
    
def set_pwm(id, pwm, write_only=True):
    """ Set the pwm (% of Vmax) of the motor """
    # Convert pwm to index
    index = unsign(round(pwm * 8.85), 2)

    # Write goal position
    if write_only:
        packetHandler.write2ByteTxOnly(portHandler, id, ADDR_GOAL_PWM, index)
    else:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_GOAL_PWM, index)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False
        return True

def get_pwm(id):
    """ Get the pwm (% of Vmax) of the motor """
    index, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id, ADDR_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pwm = sign(index, 2) / 8.85
        return pwm
    
def unsign(value, nb_bytes):
    """ Convert a signed value to an unsigned value """
    return value + 2**(8*nb_bytes) if value < 0 else value

def sign(value, nb_bytes):
    """ Convert an unsigned value to a signed value """
    return value - 2**(8*nb_bytes) if value >= 2**(8*nb_bytes-1) else value

def measure_getting_time(id=DXL_IDS[0], n=10000):
    """ Measure the time of getting the position of the motor """
    t0 = time.time()
    for i in range(n):
        get_position(id)
    return (time.time() - t0) / n

def measure_setting_write_only_time(id=DXL_IDS[0], n=10000):
    """ Measure the time of setting the position of the motor (write only) """
    pos = get_position(id)
    t0 = time.time()
    for i in range(n):
        set_position(id, pos, write_only=True)
    return (time.time() - t0) / n
    
def measure_setting_read_write_time(id=DXL_IDS[0], n=10000):
    """ Measure the time of setting the position of the motor (read/write) """
    pos = get_position(id)
    t0 = time.time()
    error_count = 0
    for i in range(n):
        if not set_position(id, pos, write_only=False):
            error_count += 1
    return (time.time() - t0) / n, error_count

def measure_timings():
    """ Measure the timings of set and get operations """
    print(f"Getting time: {measure_getting_time()} s")
    print(f"Setting time (write only): {measure_setting_write_only_time()} s")
    measured_time, error_count = measure_setting_read_write_time()
    print(f"Setting time (read/write): {measured_time} s")
    print(f"Error count: {error_count}")

def check_latency():
    """ Check the latency of the connection """
    print("Checking latency...")
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()
    latency = measure_getting_time(n=1)
    if latency > 0.001:
        print(f"Latency too high ({np.round(latency, 6)}s),  please check the connection and launch set_latency.sh before running the program")
        disable_torque()
        exit()
    print("Latency OK!")