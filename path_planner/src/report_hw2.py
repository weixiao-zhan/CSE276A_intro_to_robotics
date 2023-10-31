import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

with open('x_locs' + ".npy", 'rb') as f:
    x = np.load(f)
    #print(x)
with open('y_locs' + ".npy", 'rb') as f:
    y = np.load(f)
    #print(y)

with open('update_type' + ".npy", 'rb') as f:
    update_type = np.load(f)


x_plot = []
y_plot = []
update_type_plot = [] 

for i in range(len(x)):
    if update_type[i] == 1:
        x_plot.append(x[i])
        y_plot.append(y[i])
        update_type_plot.append(update_type[i])
    else:
        if abs(x[i] - 1.1) < 0.1 and abs(y[i]-0) < 0.1:
            x_plot.append(x[i])
            y_plot.append(y[i])
            update_type_plot.append(update_type[i])
        elif (i+1)%20==0:
            x_plot.append(x[i])
            y_plot.append(y[i])
            update_type_plot.append(update_type[i])


df = pd.DataFrame([x_plot, y_plot])

u = np.diff(x_plot)
v = np.diff(y_plot)
pos_x = x_plot[:-1] + u/2
pos_y = y_plot[:-1] + v/2
norm = np.sqrt(u**2+v**2) 

x_plot = np.array(x_plot, dtype=float)
y_plot = np.array(y_plot, dtype = float)
update_type_plot = np.array(update_type_plot, dtype=float)

fig, ax = plt.subplots()
#ax.scatter(x_plot, y_plot, c=update_type_plot, label={1:'Visual Feedback', 0:'Open-Loop Estimate'})
ax.scatter(x_plot[update_type_plot==1], y_plot[update_type_plot==1], c='green', label='Visual-Feedback')
ax.scatter(x_plot[update_type_plot==0], y_plot[update_type_plot==0], c='yellow', label='Open Loop')
ax.quiver(pos_x, pos_y, u/norm, v/norm, angles="xy", scale =55, zorder=5, pivot="mid", color='black')
ax.legend()

# Title and labels
plt.title('Robot Trajectory')
#plt.axis('equal')
plt.xlabel('X Coordinate (cm)')
plt.ylabel('Y Coordinate (cm)')
plt.xticks(np.arange(-0.5, 3, step =0.5))
plt.yticks(np.arange(-0.5, 3, step=0.5))

plt.xlim(-0.5, 2.0)
plt.ylim(-0.5, 2.5)

plt.gca().set_aspect('equal', adjustable='box')

# Display the plot
plt.show()