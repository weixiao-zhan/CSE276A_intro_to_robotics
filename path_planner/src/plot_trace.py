import pickle
import matplotlib.pyplot as plt

with open('x_locs'  + ".pkl", 'rb') as f:
    x = pickle.load(f)
    print(x)
with open('y_locs'  + ".pkl", 'rb') as f:
    y = pickle.load(f)
    print(y)

# Create scatter plot
plt.scatter(x, y)

# Title and labels
plt.title('Scatter Plot Example')
plt.xlabel('X Values')
plt.ylabel('Y Values')

# Display the plot
plt.show()
