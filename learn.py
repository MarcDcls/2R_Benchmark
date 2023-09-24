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


dt = 0.01  # [s]
length = 3  # [steps]
lr = 1e-2  # learning rate
epochs = 256
epoch_length = 32
val_length = 32
batch_size = 256
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

if __name__ == "__main__":    
    train_logs = Logs(
        [
            "logs/R1_random_pwm_22-14-36.json",
            "logs/R1_random_pwm_22-14-30.json",
        ]
    )
    validate_logs = Logs(
        [
            "logs/R1_random_pwm_22-14-10.json",
            "logs/R1_random_pwm_22-14-28.json",
            "logs/R1_random_pwm_22-14-29.json",
        ]
    )

    def compute_loss(x, y):
        return F.smooth_l1_loss(x, y, beta=1e-3, reduction="mean")

    # Creating the network
    x, y = train_logs.sample(dt, length)
    net = MLP(len(x), len(y), device)
    summary(net, input_size=(1, len(x)))

    plt = LivePlot("Loss", "Epoch", "Loss")
    optimizer = torch.optim.Adam(net.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=.5, patience=256, verbose=True)

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

                naive_estimation = samples_x[:, -2:-1]

                # positions = samples_x[:, -3:-2]
                # velocitie = samples_x[:, -2:-1]
                # new_positions = positions + velocitie * dt
                # naive_estimation = torch.cat((new_positions, velocitie), dim=1)

                loss = compute_loss(naive_estimation, samples_y)
                naive_loss += loss.item()

            validate_loss /= val_length
            plt.add_value("validate_loss", epoch, validate_loss)

            naive_loss /= val_length
            plt.add_value("naive_loss", epoch, naive_loss)
        plt.show()

        print(f"Train loss: {train_loss}, validate loss: {validate_loss}, naive loss: {naive_loss}")

    net.save("model.pt")
    plt.show(True)