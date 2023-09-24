import placo
import time
import numpy as np
import json
from mlp import MLP
from torch.nn import functional as F
import torch
from torchinfo import summary
from live_plot import LivePlot
from learn_logs import Logs

# Model parameter
dt = 0.01  # [s]
length = 3  # [steps]
state_entries = ["read_position", "read_velocity"]
action_entries = ["goal_pwm"]

# Learning parameter
lr = 1e-2  # this is scheduled
epochs = 256
epoch_length = 32
val_length = 32
batch_size = 256
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def open_logs(filenames: list[str]) -> placo.HistoryCollection:
    return Logs(filenames, state_entries, action_entries)


def make_mlp() -> MLP:
    input_size = (len(state_entries) + len(action_entries)) * length
    output_size = len(state_entries)
    return MLP(input_size, output_size, device)


def compute_loss(x, y):
    return F.smooth_l1_loss(x, y, beta=1e-3, reduction="mean")


if __name__ == "__main__":
    train_logs = open_logs(
        [
            "logs/R1_random_pwm_22-14-36.json",
            "logs/R1_random_pwm_22-14-30.json",
        ]
    )
    validate_logs = open_logs(
        [
            "logs/R1_random_pwm_22-14-10.json",
            "logs/R1_random_pwm_22-14-28.json",
            "logs/R1_random_pwm_22-14-29.json",
        ]
    )

    # Creating the network
    net = make_mlp()
    summary(net, input_size=(batch_size, (len(state_entries) + len(action_entries)) * length))
    state_size = len(state_entries)
    action_size = len(action_entries)

    plt = LivePlot("Loss", "Epoch", "Loss")
    optimizer = torch.optim.Adam(net.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=0.5, patience=256, verbose=True)
    ylim = None

    for epoch in range(epochs):
        print(f"* Epoch {epoch}")
        train_loss = 0
        for step in range(epoch_length):
            samples_x, samples_y = train_logs.sample_batch(dt, length, batch_size, device)

            loss = compute_loss(net(samples_x), samples_y)
            optimizer.zero_grad()
            loss.backward()
            train_loss += loss.item()
            optimizer.step()
            scheduler.step(loss)

        train_loss /= epoch_length
        plt.add_value("train_loss", epoch, train_loss)

        with torch.no_grad():
            validate_loss = 0
            naive_loss = 0

            for step in range(val_length):
                samples_x, samples_y = validate_logs.sample_batch(dt, length, batch_size, device)
                loss = compute_loss(net(samples_x), samples_y)
                validate_loss += loss.item()

                # Naive estimation just takes the last state
                naive_estimation = samples_x[:, -(state_size + action_size) : -action_size]
                loss = compute_loss(naive_estimation, samples_y)
                naive_loss += loss.item()

            validate_loss /= val_length
            plt.add_value("validate_loss", epoch, validate_loss)

            naive_loss /= val_length
            ylim = naive_loss if ylim is None else max(ylim, naive_loss)
            plt.add_value("naive_loss", epoch, naive_loss)
        plt.show(ylim=ylim*1.25)

        print(f"Train loss: {train_loss}, validate loss: {validate_loss}, naive loss: {naive_loss}")

    net.save("model.pt")
    plt.show(True, ylim=ylim*1.25)
