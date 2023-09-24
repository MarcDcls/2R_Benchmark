import torch as th


class MLP(th.nn.Module):
    def __init__(self, input_dimension: int, output_dimension: int):
        super().__init__()

        layers = [
            th.nn.Linear(input_dimension, 256),
            th.nn.ReLU(),
            th.nn.Linear(256, 256),
            th.nn.ReLU(),
            th.nn.Linear(256, 256),
            th.nn.ReLU(),
            th.nn.Linear(256, output_dimension),
        ]

        self.net = th.nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)
