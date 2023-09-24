from learn_logs import Logs
from matplotlib import pyplot as plt
from mlp import MLP
from learn import *

plot_logs = open_logs(["logs/R1_random_pwm_22-14-10.json"])
net = make_mlp()
net.load("model.pt")

start_t = plot_logs.collections[0].smallestTimestamp() + 3.5

# Reading real data
timestamps = np.arange(start_t, start_t + 5, dt)
real_states = [plot_logs.history_state(plot_logs.collections[0], t) for t in timestamps]
real_actions = [plot_logs.history_action(plot_logs.collections[0], t) for t in timestamps]
real_states = np.array(real_states)
real_actions = np.array(real_actions)

# Creating the initial state actions sequence
state, _ = plot_logs.get_xy_sequence(plot_logs.collections[0], start_t - length * dt, length, dt)
state_length = len(state_entries)
action_length = len(action_entries)
state_action_length = state_length + action_length
predicted_values = [state[-state_action_length:-action_length]]

# Running the inference
for t in timestamps[1:]:
    new_state = net(torch.tensor(state, device=device).float())
    new_action = plot_logs.history_action(plot_logs.collections[0], t)
    predicted_values.append(new_state.tolist())
    state += new_state.tolist()
    state += new_action
    state = state[state_action_length:]

predicted_values = np.array(predicted_values)

# Plotting
fig, axs = plt.subplots(nrows=state_action_length, sharex=True)

for i in range(state_length):
    ax = axs[i]
    name = state_entries[i]
    ax.set_title(name)
    ax.plot(timestamps, real_states[:, i], label=f"real")
    ax.plot(timestamps, predicted_values[:, i], label=f"predicted")
    ax.grid()
    ax.legend()

for i in range(action_length):
    ax = axs[i + state_length]
    name = action_entries[i]
    ax.set_title(name)
    ax.plot(timestamps, real_actions[:, i], label=f"real")
    ax.grid()
    ax.legend()

plt.legend()
plt.show()
