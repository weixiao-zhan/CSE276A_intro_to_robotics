import pickle

with open("/root/rb5_ws/src/rb5_ros/path_planner/src/"+ 'x_locs'  + ".pkl", 'wb') as f:
    pickle.dump([1,2,3], f)
with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" + 'y_locs'  + ".pkl", 'wb') as f:
    pickle.dump([1,2,3], f)
    #/root/rb5_ws/src/rb5_ros/path_planner/src/trace_test.py

with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" + 'y_locs'  + ".pkl", 'rb') as f:
    ab = pickle.load(f)
    print(ab)

