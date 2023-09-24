import placo
import numpy as np
import json

class Logs:
    def __init__(self, logs: list[str]) -> None:
        self.collections: list[placo.HistoryCollection] = []

        for log in logs:
            self.load_log(log)

    def load_log(self, path: str) -> None:
        with open(path) as f:
            data = json.load(f)

            collection = placo.HistoryCollection()
            for k in range(len(data["timestamps"])):
                t = data["timestamps"][k]
                collection.push_number("read_position", t, data["read_position"][k])
                collection.push_number("read_velocity", t, data["read_velocity"][k])
                collection.push_number("goal_pwm", t, data["goal_pwm"][k])

            self.collections.append(collection)

    def random_collection(self) -> placo.HistoryCollection:
        # Choose a random collection weighted by its duration
        durations = np.array(
            [collection.biggestTimestamp() - collection.smallestTimestamp() for collection in self.collections]
        )
        probabilities = durations / np.sum(durations)
        collection = np.random.choice(self.collections, p=probabilities)
        return collection
    
    def history_state(self, collection: placo.HistoryCollection, t: float) -> list:
        return [
            collection.number("read_position", t),
            collection.number("read_velocity", t),
        ]
    
    def history_action(self, collection: placo.HistoryCollection, t: float) -> list:
        return [
            collection.number("goal_pwm", t),
        ]

    def sample(self, dt: float, length: int) -> list:
        margin = (length * 2) * dt
        collection: placo.HistoryCollection = self.random_collection()
        start_time: float = np.random.uniform(collection.smallestTimestamp() + margin, collection.biggestTimestamp() - margin)

        x: list = []
        for k in range(length):
            t: float = start_time + k * dt
            x += self.history_state(collection, t)
            x += self.history_action(collection, t)

        y = self.history_state(collection, start_time + length * dt)

        return x, y


log = Logs(
    [
        "logs/R1_random_pwm_22-14-30.json",
        "logs/R1_random_pwm_22-14-36.json",
    ]
)

x, y = log.sample(0.01, 5)
print(x, y)
