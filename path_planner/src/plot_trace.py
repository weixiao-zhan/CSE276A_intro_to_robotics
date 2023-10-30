import numpy as np
import matplotlib.pyplot as plt

with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" + 'x_locs' + ".npy", 'rb') as f:
    x = np.load(f)
    print(x)
with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" + 'y_locs' + ".npy", 'rb') as f:
    y = np.load(f)
    print(y)

# Create scatter plot
plt.scatter(x, y)

# Title and labels
plt.title('Scatter Plot Example')
plt.xlabel('X Values')
plt.ylabel('Y Values')

# Display the plot
plt.show()
