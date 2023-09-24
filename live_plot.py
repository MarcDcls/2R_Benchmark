import matplotlib.pyplot as plt


class LivePlot:
    def __init__(self, title, xlabel: str, ylabel: str):
        self.x = {}
        self.y = {}
        self.title = title
        self.xlabel = xlabel
        self.ylabel = ylabel

    def add_value(self, name, x, y):
        if name not in self.x:
            self.x[name] = []
            self.y[name] = []
        self.x[name].append(x)
        self.y[name].append(y)

    def show(self, block: bool = False):
        plt.clf()
        for key in self.x:
            plt.plot(self.x[key], self.y[key], label=key)
        plt.legend()
        plt.ylim(0, 0.1)
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.grid()

        if block:
            plt.show()
        else:
            plt.pause(0.05)
