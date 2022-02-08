#!/usr/bin/env python3

import rospkg
import numpy as np

ALTITUDE = 10
AIRSPEED = 3
POINTS = np.array([[0, 0],
                   [20, 20],
                   [40, 0],
                   [20, -20],
                   [0, 0],
                   [-20, 20],
                   [-40, 0],
                   [-20, -20]])

VELOCITIES = np.row_stack((POINTS[1, :] - POINTS[-1, :],
                           np.array([POINTS[i + 1, :] - POINTS[i - 1, :] for i in range(1, POINTS.shape[0] - 1)]),
                           POINTS[0, :] - POINTS[-2, :]))
NORMALIZED_VELOCITIES = np.array([AIRSPEED / np.linalg.norm(VEL) * VEL for VEL in VELOCITIES])
CSV = np.column_stack((POINTS, NORMALIZED_VELOCITIES, ALTITUDE * np.ones(POINTS.shape[0]))).transpose()


def main():
    directory = rospkg.RosPack().get_path("bmb_controllers")
    np.savetxt(f"{directory}/config/reference_trajectory_loop.csv", CSV, delimiter=",")


if __name__ == '__main__':
    main()
