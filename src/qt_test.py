import sys
import matplotlib
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import time


class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=7, height=6, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        self.sc = MplCanvas(self, width=5, height=4, dpi=100)
       

    def update(self, id, position):
        self.sc.axes.plot([0,1,2,3,4], [10,1,20,3,40])
        self.setCentralWidget(self.sc)
        self.show()


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
while True:
    w.update(0, 1)
    time.sleep(0.1)
#app.exec_()