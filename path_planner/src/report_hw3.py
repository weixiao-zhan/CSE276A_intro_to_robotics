import numpy as np
import matplotlib.pyplot as plt

with open('state_vector' + ".npy", 'rb') as f:
    state = np.load(f)
    #print(x)
with open('order_tags' + ".npy", 'rb') as f:
    order = np.load(f)
    #print(y)

with open('order_tags' + ".npy", 'rb') as f:
    order = np.load(f)
    #print(y)

print(state)
print(order)

robot = state[:3]
lm_x = state[3::2]
lm_y = state[4::2]
print(robot, lm_x, lm_y)
plt.scatter(lm_x, lm_y, c = order)
plt.show