import torch
import placo
import json
import numpy as np


class Logs:
    def __init__(self, logs: list[str], state_entries: list[str], action_entries: list[str]) -> None:
        self.collections: list[placo.HistoryCollection] = []
        self.state_entries = state_entries
        self.action_entries = action_entries

        for log in logs:
            self.load_log(log)

    def load_log(self, path: str) -> None:
        with open(path) as f:
            data = json.load(f)

            collection = placo.HistoryCollection()
            for k in range(len(data["timestamps"])):
                t = data["timestamps"][k]
                for entry in self.state_entries:
                    collection.push_number(entry, t, data[entry][k])
                for entry in self.action_entries:
                    collection.push_number(entry, t, data[entry][k])

            self.collections.append(collection)

    def random_collection(self) -> placo.HistoryCollection:
        # Choose a random collection weighted by its duration
        durations = np.array(
            [collection.biggestTimestamp() - collection.smallestTimestamp() for collection in self.collections]
        )
        probabilities = durations / np.sum(durations)
        return np.random.choice(self.collections, p=probabilities)

    def history_state(self, collection: placo.HistoryCollection, t: float) -> list:
        return [collection.number(entry, t) for entry in self.state_entries]

    def history_action(self, collection: placo.HistoryCollection, t: float) -> list:
        return [collection.number(entry, t) for entry in self.action_entries]

    def get_xy_sequence(
        self, collection: placo.HistoryCollection, start_time: float, length: int, dt: float
    ) -> tuple[list, list]:
        x: list = []
        for k in range(length):
            t: float = start_time + k * dt
            x += self.history_state(collection, t)
            x += self.history_action(collection, t)

        y = self.history_state(collection, start_time + length * dt)
        return x, y

    def sample(self, dt: float, length: int) -> list:
        margin = (length * 2) * dt
        collection: placo.HistoryCollection = self.random_collection()
        start_time: float = np.random.uniform(collection.smallestTimestamp() + margin, collection.biggestTimestamp() - margin)

        return self.get_xy_sequence(collection, start_time, length, dt)

    def sample_batch(self, dt: float, length: int, batch_size: int, device: torch.device) -> list[torch.Tensor]:
        x, y = zip(*[self.sample(dt, length) for _ in range(batch_size)])
        return torch.tensor(x, device=device).float(), torch.tensor(y, device=device).float()
