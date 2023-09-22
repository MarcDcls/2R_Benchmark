import numpy as np
import matplotlib.pyplot as plt

def random_sinusoidal_trajectory(segment_duration=1, duration=10.0, nb_points=10000):
    """ Generate a random trajectory with sinusoidal segments """
    t = np.linspace(0, duration, 100000)
    trajectory = np.zeros_like(t)
    num_segments = int(duration / segment_duration)
    up = True
    for i in range(num_segments):
        while True:
            print(i)
            # Randomly choose parameters for the sine wave
            amplitude = np.random.uniform(0.1, 1.0)
            print("amplitude: ", amplitude)
            frequency = np.random.uniform(0.5, 2.0)
            phase = np.random.uniform(-np.pi/2, np.pi/2)

            # Generate the sine wave for this segment
            segment = amplitude * np.sin(2 * np.pi * frequency * t + phase)
            
            # Check if the segment is valid
            if up and (segment[-1] - segment[0]) < 0:
                continue
            elif not up and (segment[-1] - segment[0]) > 0:
                continue

            print("OK")
            print(segment[-1] - segment[0])
            # Choose the direction of the next segment
            if segment[-1] > 0:
                up = False
            else:
                up = True
                
            # Add the segment to the trajectory
            trajectory += segment
            break
    return trajectory

# nb_points = 100000
# duration = 10

# t = np.linspace(0, duration, nb_points)
# traj = random_sinusoidal_trajectory(segment_duration=1, duration=duration, nb_points=nb_points)
# plt.figure(figsize=(8, 6))
# plt.plot(t, traj)
# plt.xlabel('Time')
# plt.ylabel('Position')
# plt.title('Random Trajectory with Sinusoidal Segments (Oscillating Around Zero)')
# plt.grid(True)
# plt.show()

# Define the parameters for the random sinusoids
num_segments = 3  # Number of sinusoidal segments
duration = 100.0  # Total duration of the trajectory
t = np.linspace(0, duration, 10000)  # Time vector

# Initialize the trajectory as an array of zeros
trajectory = np.zeros_like(t)

# Generate random sinusoidal segments and add them to the trajectory
for _ in range(num_segments):
    # Randomly choose parameters for the sine wave
    amplitude = np.random.uniform(0.1, 1.0)  # Amplitude
    frequency = np.random.uniform(0.5, 2.0)  # Frequency in Hz
    phase = np.random.uniform(0, 2 * np.pi)  # Phase

    # Generate the sine wave for this segment
    segment = amplitude * np.sin(2 * np.pi * frequency * t + phase)

    # Add the segment to the trajectory
    trajectory += segment

# Plot the random trajectory
plt.figure(figsize=(8, 6))
plt.plot(t, trajectory)
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Random Trajectory with Sinusoidal Segments')
plt.grid(True)
plt.show()