#!/usr/bin/env python
import argparse
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy.linalg
import sys
import json

COLOR_POINTS = 'b'
COLOR_TRAJ = 'g'
COLOR_RIGS = 'r'

def load_sparse_map(filename):
    # Set the keys.
    key_points = "points"        # sparse_map[key_points]
    key_loc = "location"         # sparse_map[key_points][point_id][key_loc]
    key_rigs = "rigs"            # sparse_map[key_rigs]
    key_time = "timestamp"       # sparse_map[key_time]
    key_pose = "pose"            # sparse_map[key_rigs][rig_id][key_pose]
    key_rig_T_map = "rig_T_map"  # sparse_map[key_rigs][rig_id][key_pose][key_rig_T_map]

    sparse_map = json.loads(open(filename).read())

    # Collect points.
    assert key_points in sparse_map.keys()
    assert isinstance(sparse_map[key_points], dict)

    points = list()
    for point_id, point_spec in sparse_map[key_points].items():
        # Get the location of this point.
        assert isinstance(point_spec, dict)
        assert key_loc in point_spec.keys()
        loc = point_spec[key_loc]
        assert isinstance(loc, list)
        assert len(loc) == 3

        # str --> float
        loc = [float(v) for v in loc]
        points.append(loc)


    # Collect rigs.
    assert key_rigs in sparse_map.keys()
    assert isinstance(sparse_map[key_rigs], dict)

    rig_locs = dict()  # timestamp -> (x, y, z)
    timestamps = list()
    for rig_id, rig_spec in sparse_map[key_rigs].items():
        assert isinstance(rig_spec, dict)

        # Get timestamp.
        assert key_time in rig_spec.keys()
        timestamp = int(rig_spec[key_time])

        assert key_pose in rig_spec.keys()
        assert isinstance(rig_spec[key_pose], dict)
        assert key_rig_T_map in rig_spec[key_pose].keys()
        rig_T_map_str = rig_spec[key_pose][key_rig_T_map]
        assert isinstance(rig_T_map_str, list)
        assert len(rig_T_map_str) == 4

        # str --> float
        rig_T_map = list()
        for rig_T_map_row in rig_T_map_str:
            assert isinstance(rig_T_map_row, list)
            assert len(rig_T_map_row) == 4
            rig_T_map.append([float(v) for v in rig_T_map_row])

        # rig_T_map --> map_T_rig
        map_T_rig = np.linalg.inv(np.array(rig_T_map))
        # Get rig location in the map coordinate.
        rig_locs[timestamp] = [map_T_rig[0][3], map_T_rig[1][3], map_T_rig[2][3]]
        timestamps.append(timestamp)

    # Sort the rigs according to timestamp
    timestamps.sort()
    rigs = [rig_locs[t] for t in timestamps]

    return points, rigs

def load_trajectory(filename):
    traj = np.loadtxt(filename, delimiter=" ")
    return traj.tolist()

def main(filename_sparse_map, filename_traj):
    points, rigs = load_sparse_map(filename_sparse_map)
    print "Number of Points: " + str(len(points))
    print "Number of Rigs: " + str(len(rigs))
    traj = None
    if filename_traj is not None:
      traj = load_trajectory(filename_traj)

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Plot points.
    x = np.array([pt[0] for pt in points])
    y = np.array([pt[1] for pt in points])
    z = np.array([pt[2] for pt in points])
    ax.scatter(x, y, z, c = COLOR_POINTS, s = 1)

    # Plot rigs.
    x = np.array([loc[0] for loc in rigs])
    y = np.array([loc[1] for loc in rigs])
    z = np.array([loc[2] for loc in rigs])
    ax.plot(x, y, z, COLOR_RIGS)

    # Plot trajectory.
    if traj is not None:
        x = np.array([loc[0] for loc in traj])
        y = np.array([loc[1] for loc in traj])
        z = np.array([loc[2] for loc in traj])
        ax.plot(x, y, z, COLOR_TRAJ)

    ax.set(adjustable='box-forced', aspect='equal')
    ax.auto_scale_xyz([-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0])
    ax.legend()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Visualize a sparse map JSON file.")
    parser.add_argument('filename', type = str, help = "Path to the sparse map JSON file.")
    parser.add_argument('--traj', type = str, help = "Path to the tracking location .csv file.")
    args = parser.parse_args()

    main(args.filename, args.traj)
