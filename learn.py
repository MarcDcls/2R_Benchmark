import placo
import numpy as np
import json
from mlp import MLP
from torch.nn import functional as F
import torch
from torchinfo import summary
from live_plot import LivePlot
from learn_logs import Logs


dt = 0.01  # [s]
length = 1  # [steps]
lr = 1e-3  # learning rate
epochs = 128
epoch_length = 16
batch_size = 256
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

train_logs = Logs(
    [
        "logs/R1_random_pwm_22-14-30.json",
        "logs/R1_random_pwm_22-14-36.json",
    ]
)
validate_logs = Logs(
    [
        "logs/R1_random_pwm_22-14-28.json",
        "logs/R1_random_pwm_22-14-10.json",
        "logs/R1_random_pwm_22-14-29.json",
    ]
)

# Creating the network
x, y = train_logs.sample(dt, length)
net = MLP(len(x), len(y), device)
summary(net, input_size=(1, len(x)))

plt = LivePlot("Loss", "Epoch", "Loss")
optimizer = torch.optim.Adam(net.parameters(), lr=lr, weight_decay=1e-5)

for epoch in range(epochs):
    print(f"* Epoch {epoch}")
    train_loss = 0
    for step in range(epoch_length):
        samples_x, samples_y = train_logs.sample_batch(dt, length, batch_size, device)

        loss = F.smooth_l1_loss(net(samples_x), samples_y, reduction="mean")
        loss.backward()
        train_loss += loss.item()
        optimizer.step()
        optimizer.zero_grad()

    train_loss /= epoch_length
    plt.add_value("train_loss", epoch, train_loss)

    validate_loss = 0
    for step in range(epoch_length):
        samples_x, samples_y = validate_logs.sample_batch(dt, length, batch_size, device)

        loss = F.smooth_l1_loss(net(samples_x), samples_y, reduction="mean")
        validate_loss += loss.item()

    validate_loss /= epoch_length
    plt.add_value("validate_loss", epoch, validate_loss)
    plt.show()

    print(f"Train loss: {train_loss}, validate loss: {validate_loss}")

net.save("model.pt")
