import numpy as np
import pickle
import rospy
import sys

#durations = [5, 3, 0, 5, 5.5, 5.5, 9.5, 5, 5.5, 6.5, 4, 0, 5, 5]
durations = [10, 10, 0, 10, 10, 10, 15, 10, 10, 10, 10, 0, 10, 10]
start_times = np.cumsum(durations)

def add_time(action, start, duration):
    n = len(action['msg'].points)
    times = np.cumsum([0] + (n-1)*[float(duration)/(n-1)])

    for i in range(n):
        action['msg'].points[i].time_from_start = rospy.Duration.from_sec(times[i])

if __name__ == "__main__":
    in_file = sys.argv[1]
    out_file = sys.argv[2]

    data = pickle.load(open(in_file))

    for objs in data:
        for obj_idx in range(len(data[objs])):
            for bin in data[objs][obj_idx]:
                for i, action in enumerate(data[objs][obj_idx][bin]):
                    if action['type'] == 'arm':
                        add_time(action, start_times[i], durations[i])

    pickle.dump(data, open(out_file, 'w'))

