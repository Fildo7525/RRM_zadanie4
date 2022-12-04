import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
 
i = 0

pos1 = []
vel1 = []
acc1 = []
ynk1 = []

pos2 = []
vel2 = []
acc2 = []
ynk2 = []

time = []

def eavenPlot(x, y, yLabel, Title):
    global i
    cubic_interpolation_model = interp1d(x, y, kind = "cubic")
    X_=np.linspace(min(x), max(x), 500)
    Y_=cubic_interpolation_model(X_)
    plt.figure(i)
    plt.plot(X_, Y_);
    plt.xlabel('time (s)')
    plt.ylabel(yLabel)
    plt.title(Title)
    plt.grid(True)
    i+=1

with open('./src/trajectory_visualization/hodnoty.txt') as file:
    pos1 = [file.readline() for _ in range(0,40)]
    file.readline()
    vel1 = [file.readline() for _ in range(0,40)]
    file.readline()
    acc1 = [file.readline() for _ in range(0,40)]
    file.readline()
    ynk1 = [file.readline() for _ in range(0,40)]
    file.readline()
    pos2 = [file.readline() for _ in range(0,40)]
    file.readline()
    vel2 = [file.readline() for _ in range(0,40)]
    file.readline()
    acc2 = [file.readline() for _ in range(0,40)]
    file.readline()
    ynk2 = [file.readline() for _ in range(0,40)]
    file.readline()

time = [i/10 for i in range(0, len(pos2))]

eavenPlot(time, pos1, "Position (rad)", "First joint position");
eavenPlot(time, vel1, "Velocity (rad/s)", "First joint velocity");
eavenPlot(time, acc1, "Acceleration (rad/s^2)", "First joint acceleration");
eavenPlot(time, ynk1, "Yank (rad/s^3)", "First joint yank");
eavenPlot(time, pos2, "Position (rad)", "Third joint position");
eavenPlot(time, vel2, "Velocity (rad/s)", "Third joint position");
eavenPlot(time, acc2, "Acceleration (rad/s^2)", "Third joint position");
eavenPlot(time, ynk2, "Yank (rad/s^3)", "Third joint position");

plt.show()

