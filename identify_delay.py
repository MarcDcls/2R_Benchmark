import json
import numpy as np
import matplotlib.pyplot as plt


with open("logs_1R/R1_sinus_0w.json", 'r') as f:
        data = json.load(f)

def moving_avr(signal, window_size):
    kernel = np.ones(window_size) / window_size
    return np.convolve(signal, kernel, mode='valid')

# plt.plot(data["timestamps"], data["read_position"], label="read position")
# plt.plot(data["timestamps"], data["goal_position"], label="goal position")
# plt.legend()
# plt.show()

# plt.plot(data["timestamps"], data["read_velocity"], label="read velocity")
# plt.plot(data["timestamps"][10:-10], moving_avr(data["read_velocity"], 21), label="avr velocity")  
# plt.plot(data["timestamps"], data["goal_velocity"], label="goal velocity")
# plt.legend()
# plt.show()

def find_zeros(values, timestamps):
    """
    Return the timestamps of the zeros in the values
    """
    values = moving_avr(values, 21)
    zeros = []
    last = values[0]

    for i, value in enumerate(values):
        if value == 0 or value * last < 0:
            zeros.append(timestamps[i + 10])
        last = value
    return zeros

read_pos_zeros = find_zeros(data["read_position"], data["timestamps"])
goal_pos_zeros = find_zeros(data["goal_position"], data["timestamps"])

if len(read_pos_zeros) != len(goal_pos_zeros):
    print("Different number of zeros in read and goal position")
    print("read position zeros size:", len(read_pos_zeros))
    print("goal position zeros size:", len(goal_pos_zeros))
    exit()

# Delay between read and goal position
pos_delay = np.mean(np.array(read_pos_zeros) - np.array(goal_pos_zeros))
print("Position read-goal-delay:", pos_delay)

read_vel_zeros = find_zeros(data["read_velocity"], data["timestamps"])
goal_vel_zeros = find_zeros(data["goal_velocity"], data["timestamps"])

if len(read_vel_zeros) != len(goal_vel_zeros):
    print("Different number of zeros in read and goal velocity")
    print("read velocity zeros size:", len(read_vel_zeros))
    print("goal velocity zeros size:", len(goal_vel_zeros))
    exit()

# We suppose the delay between goal position and read position egal 
# to the delay between goal velocity and real velocity.
# The computation of the read velocity by the motor cause a delay between 
# the read velocity and the real velocity.
# Thus (read_vel - real_vel) = (read_vel - goal_vel) - (read_pos - goal_pos)
vel_delay = np.mean(np.array(read_vel_zeros) - np.array(goal_vel_zeros)) - pos_delay
print("Velocity read-real-delay:", vel_delay)
