import torch as th


class MLP(th.nn.Module):
    def __init__(self, input_dimension: int, output_dimension: int, device: th.device):
        self.device: th.device = device
        super().__init__()

        layers = [
            th.nn.Linear(input_dimension, 256),
            th.nn.ReLU(),
            th.nn.Linear(256, 256),
            th.nn.ReLU(),
            th.nn.Linear(256, output_dimension),
        ]

        self.net = th.nn.Sequential(*layers).to(device)

    def forward(self, x):
        return self.net(x)

    def load(self, filename: str):
        self.load_state_dict(th.load(filename, map_location=self.device))

    def save(self, filename: str):
        th.save(self.state_dict(), filename)
