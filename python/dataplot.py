#
# dataplot.py — esteso con save() e plot_multiple_save()
#

from matplotlib import pylab
import os


class DataPlotter:

    FIGURE = 1

    def __init__(self):
        self.y_data = {}
        self.y_label = {}
        self.x_data = []
        self.x_label = "x"
        self.__options = ['r-', 'b-', 'g-', 'k--', 'm-']
        DataPlotter.FIGURE = 1

    def set_x(self, label):
        self.x_label = label

    def append_x(self, value):
        self.x_data.append(value)

    def add_y(self, varname, varlabel):
        self.y_data[varname] = []
        self.y_label[varname] = varlabel

    def append_y(self, varname, value):
        self.y_data[varname].append(value)

    def plot(self):
        pylab.figure(DataPlotter.FIGURE)

        i = 0
        for varname in self.y_label:
            pylab.plot(self.x_data, self.y_data[varname],
                       self.__options[i % len(self.__options)],
                       label=self.y_label[varname])
            i += 1
        pylab.xlabel(self.x_label)
        pylab.legend()
        pylab.show()
        DataPlotter.FIGURE += 1

    def save(self, filepath: str, title: str = ""):
        """
        Salva il grafico come file PNG (o altro formato supportato da matplotlib).
        Non chiama show() — adatto all'uso da monitor background.

        :param filepath: percorso completo del file di output (es. 'plots/drone_0/altitude.png')
        :param title: titolo opzionale del grafico
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True) if os.path.dirname(filepath) else None

        fig = pylab.figure(DataPlotter.FIGURE)
        if title:
            pylab.title(title)

        i = 0
        for varname in self.y_label:
            pylab.plot(self.x_data, self.y_data[varname],
                       self.__options[i % len(self.__options)],
                       label=self.y_label[varname])
            i += 1
        pylab.xlabel(self.x_label)
        pylab.legend()
        pylab.tight_layout()
        pylab.savefig(filepath, dpi=100)
        pylab.close(fig)
        DataPlotter.FIGURE += 1


def plot_multiple(plotters, figsize=(20, 18)):
    """
    Plots multiple DataPlotter objects as subplots in a single figure.
    :param plotters: list of DataPlotter objects
    :param figsize: tuple (width, height) of the figure
    """
    num_plots = len(plotters)
    pylab.figure(DataPlotter.FIGURE, figsize=figsize)

    for idx, dp in enumerate(plotters, 1):
        pylab.subplot(num_plots, 1, idx)
        i = 0
        for varname in dp.y_label:
            pylab.plot(dp.x_data, dp.y_data[varname],
                       dp._DataPlotter__options[i % len(dp._DataPlotter__options)],
                       label=dp.y_label[varname])
            i += 1
        pylab.xlabel(dp.x_label)
        pylab.legend()

    pylab.tight_layout()
    pylab.show()
    DataPlotter.FIGURE += 1


def plot_multiple_save(plotters, filepath: str, title: str = "", figsize=(14, 4)):
    """
    Salva più DataPlotter come subplots verticali in un unico file PNG.
    Non chiama show().

    :param plotters: lista di DataPlotter
    :param filepath: percorso completo del file di output
    :param title:    titolo della figura
    :param figsize:  (width, height_per_subplot * n_subplots) — calcolato auto se lasciato default
    """
    num_plots = len(plotters)
    auto_height = max(3 * num_plots, 6)
    fig = pylab.figure(DataPlotter.FIGURE, figsize=(figsize[0], auto_height))

    if title:
        fig.suptitle(title, fontsize=12, fontweight='bold')

    for idx, dp in enumerate(plotters, 1):
        ax = pylab.subplot(num_plots, 1, idx)
        i = 0
        for varname in dp.y_label:
            ax.plot(dp.x_data, dp.y_data[varname],
                    dp._DataPlotter__options[i % len(dp._DataPlotter__options)],
                    label=dp.y_label[varname])
            i += 1
        ax.set_xlabel(dp.x_label)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    pylab.tight_layout()
    if title:
        pylab.subplots_adjust(top=0.94)

    os.makedirs(os.path.dirname(filepath), exist_ok=True) if os.path.dirname(filepath) else None
    pylab.savefig(filepath, dpi=100)
    pylab.close(fig)
    DataPlotter.FIGURE += 1


if __name__ == "__main__":
    import math

    d1 = DataPlotter()
    d1.set_x("time")
    d1.add_y("sin", "Sin(t)")
    t = 0
    while t < 10:
        d1.append_x(t)
        d1.append_y("sin", math.sin(t))
        t += 0.01

    d2 = DataPlotter()
    d2.set_x("time")
    d2.add_y("cos", "Cos(t)")
    t = 0
    while t < 10:
        d2.append_x(t)
        d2.append_y("cos", math.cos(t))
        t += 0.01

    # Test interattivo
    plot_multiple([d1, d2], figsize=(10, 8))

    # Test save
    plot_multiple_save([d1, d2], filepath="/tmp/test_save.png", title="Test sin/cos")
    print("Salvato in /tmp/test_save.png")
