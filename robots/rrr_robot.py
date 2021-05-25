"""
RRRRobot robot class definition
"""
import pickle
from pathlib import Path

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

from robots.robot import Robot
from utils.robo_math import SymbolicTransformation as st
from utils.plot_utils import TransformationPlotter


class RRRRobot(Robot):
    """
    RRRRobot manipulator Forward Kinematics (FK) and Inverse Kinematics(IK)
    calculator class

    Attributes:
        d (float): Length parameter from TxTz substitution
        dq (float): Angle parameter from TxTz substitution
        fk_data_path (pathlib.Path): Path to pickle forward kinematics data
            Used to speedup FK calculation
        ik_data_path (pathlib.Path): Path to pickle inverse kinematics data
            Used to speedup FK calculation
        ls (tuple): Lengths of links
        qs_lim_deg (tuple of tuples): Joint limits in degrees
        qs_lim_rad (tuple of tuples): Joint limits in radians
        T_base (4x4 array like): Transformation from world frame to the base
        T_tool (4x4 array like): Transformation from end-effector frame to the
            tool frame
    """

    ls = (5,5,5,5)
    qs_lim_deg = ((-360.0, 360.0),
                  (-360.0, 360.0),
                  (-360.0, 360.0))

    def __init__(self, T_base=None, T_tool=None):
        """
        Prepares all necessary values and loads pickled matrices

        Args:
            T_base (None, optional): Transformation from the world frame
                to the base frame
            T_tool (None, optional): Transformation from the end-effector
                frame to the tool frame
        """
        self.set_transforms(T_base, T_tool)

        self._generate_value_pairs()
        self._calculate_limits_radians()

        self.fk_data_path = Path("robots/data/rrr_forward_kinematics.pkl")
        self._precalculate_data()

        self._tp = TransformationPlotter()

    def _generate_value_pairs(self):
        """
        Generates name-value tuples for sympy substitution
        """
        value_pairs = []
        for i in range(len(self.ls)):
            value_pairs.append((f"l_{i}", self.ls[i]))

        self._value_pairs = value_pairs

    def _calculate_limits_radians(self):
        """
        Converts joint limits from degrees to radians
        """
        self.qs_lim_rad = tuple(
            (np.deg2rad(x[0]), np.deg2rad(x[1])) for x in self.qs_lim_deg)

    def _precalculate_data(self):
        """
        Precalculates and pickles constant matrices
        """
        '''
        if self.fk_data_path.is_file():
            with open(self.fk_data_path, 'rb') as input:
                self._Ts = pickle.load(input)
        else:
            self._Ts = st("RzTzRyTxRyTx",
                          ['q_0', 'l_0', 'q_1', 'l_1', 'q_2', 'l_2'])

            self._Ts.substitute(self._value_pairs)
        '''
        self._Ts = st("RzTzRyTxRyTx",
                        ['q_0', 'l_0', 'q_1', 'l_1', 'q_2', 'l_2'])

        self._Ts.substitute(self._value_pairs)        
        # Save data

        '''
        if not self.fk_data_path.is_file():
            with open(self.fk_data_path, 'wb') as output:
                pickle.dump(self._Ts, output, pickle.HIGHEST_PROTOCOL)
        '''
    def forward_kinematics(self, q_values, plot=True):
        """
        Calculates forward kinematics of the tool pose given values of joints

        Args:
            q_values (list of float): Values of joints
            plot (bool, optional): Flag to plot the result

        Returns:
            4x4 np.ndarray: Homogeneous tool pose
        """
        qs_dict = {}
        for i in range(len(q_values)):
            qs_dict[sp.symbols(f"q_{i}")] = q_values[i]

        self._numeric_frames = []
        for frame in self._Ts.frames:
            self._numeric_frames.append(frame.evalf(subs=qs_dict))

        T = self.T_base * self._numeric_frames[-1] * self.T_tool

        if plot:
            self._show_fk()
        return np.array(T, dtype=np.float)

    def _show_fk(self):
        """
        Plots current pose of the robot
        """
        frames = [self.T_base]

        for frame, var in zip(self._numeric_frames, self._Ts.variables):
            if var[0] == 'q':
                frames.append(self.T_base * frame)

        frames.append(self.T_base * self._numeric_frames[-1])
        frames.append(frames[-1] * self.T_tool)

        self._tp.plot_numeric_frames(frames, axis_len=self.ls[0] / 4)

    def inverse_kinematics(self, T, m=1, k=1):
        """
        Calculates inverse kinematics joint values qs from pose T

        Args:
            T (4x4 array like): Homogeneous pose matrix
            m (int, optional): Elbow up flag. Should be -1 or 1
            k (int, optional): Square root sign flag. Should be -1 or 1

        Returns:
            np.ndarray: Joint values, corresponding to T
                or zeros in case of failure
        """
        if abs(m) != 1:
            print("[WARNING] m can only be -1 or 1. Defaulting to 1")
            m = 1

        if abs(k) != 1:
            print("[WARNING] k can only be -1 or 1. Defaulting to 1")
            k = 1

        T = sp.Matrix(T)
        T_0 = self.T_base.inv() * T * self.T_tool.inv()

        x, y, z = float(T_0[0, 3]), float(T_0[1, 3]), float(T_0[2, 3])
        x_prime = k * np.sqrt(x**2 + y**2)
        z_prime = self.ls[0] - z

        arccos_numerator = x_prime**2 + \
            z_prime**2 - self.ls[1]**2 - self.ls[2]**2
        arccos_denominator = 2.0 * self.ls[1] * self.ls[2]
        arccos = arccos_numerator / arccos_denominator

        # Check if the given position is reachable
        if abs(arccos) > 1:
            print("[INFO] The configuration is not reachable")
            return np.array([0.0, 0.0, 0.0])

        q_2 = m * np.arccos(arccos)

        beta = np.arctan2(self.ls[2] * np.sin(m * q_2),
                          self.ls[1] + self.ls[2] * np.cos(q_2))
        q_1 = np.arctan2(z_prime, x_prime) - m * beta

        A = self.ls[1] * np.cos(q_1) + self.ls[2] * np.cos(q_1 + q_2)

        if abs(A) < self.epsilon:
            print("[INFO] Up position case. q_0 - any angle. Setting it to 0")
            q_0 = 0.0
        else:
            q_0 = np.arctan2(y, x)

        qs = np.array([q_0, q_1, q_2])

        out_of_limits = False
        for i in range(len(qs)):
            if qs[i] < self.qs_lim_rad[i][0] or qs[i] > self.qs_lim_rad[i][1]:
                print(f"[INFO] q_{i} = {np.rad2deg(qs[i]):.3f} "
                      f"(degrees) is out of limits")
                out_of_limits = True

        if out_of_limits:
            return np.array([0.0, 0.0, 0.0])

        return qs

    def move_joints(self, qs):
        """
        Iteratively moves joints according to the trajectorties in qs

        Args:
            qs (np.ndarray): nx3 array, where n - is the number of points
        """
        #plt.ion()
        Ts = []
        for q in qs:
            #for T in Ts:
            #    self._tp.plot_position(T)

            T = self.forward_kinematics(q, plot=False)
            Ts.append(T)

            #plt.pause(1e-9)
            #self._tp.ax.cla()
        plt.ioff()
        for T in Ts[:-1]:
            self._tp.plot_position(T, show=False)
        self.forward_kinematics(qs[-1], plot=False)

    def move_via_points(self, pts):
        """
        Iteratively moves joints via cartesian points using IK

        Args:
            pts (np.ndarray): nx3 array, where n - is the number of points

        Returns:
            np.ndarray: Array of corresponding joint configurations
        """
        plt.ion()
        Ts = []
        qs = []
        for pt in pts:
            for T in Ts:
                self._tp.plot_position(T)

            self._tp.ax.scatter(
                pts[0][0],
                pts[0][1],
                pts[0][2],
                c='red',
                s=40,
                alpha=0.6,
            )

            self._tp.ax.scatter(
                pts[-1][0],
                pts[-1][1],
                pts[-1][2],
                c='red',
                s=40,
                alpha=0.6,
            )

            T_IK = np.array([
                [1, 0, 0, pt[0]],
                [0, 1, 0, pt[1]],
                [0, 0, 1, pt[2]],
                [0, 0, 0, 1]
            ])

            q = self.inverse_kinematics(T_IK)
            qs.append(q)
            self.getJointCoordinates(q)
            #T = self.forward_kinematics(q, plot=True)
            #Ts.append(T)

            #plt.pause(1e-9)
            #self._tp.ax.cla()
        plt.ioff()
        #for T in Ts[:-1]:
        #    self._tp.plot_position(T, show=False)
        #self.forward_kinematics(q, plot=True)

        return np.array(qs).T

    def getJointCoordinates(self, q_values):

        # Account for the opposite rotation directions and offsets
        #q_values = np.multiply(q_values, self.qs_directions) + self.qs_offsets

        qs_dict = {}
        for i in range(len(q_values)):
            qs_dict[sp.symbols(f"q_{i}")] = q_values[i]

        self._numeric_frames = []

        for frame in self._Ts.frames:
            self._numeric_frames.append(frame.evalf(subs=qs_dict))

        T = self.T_base * self._numeric_frames[-1] * self.T_tool

        frames = [self.T_base]

        for frame, var in zip(self._numeric_frames, self._Ts.variables):
            if var[0] == 'q':
                frames.append(self.T_base * frame)

        frames.append(self.T_base * self._numeric_frames[-1])
        frames.append(frames[-1] * self.T_tool)

        joint_points = []
        joint_points.append(np.array([[0], [0], [0]]))

        for frame in frames:
            joint_points.append(frame[:3, 3])
        joint_points = np.array(joint_points, dtype=float)
        margin = 1
        fixed_scale = False
        center = 0
        max_range = np.array([
            joint_points[:, 0, 0].max() - joint_points[:, 0, 0].min(),
            joint_points[:, 1, 0].max() - joint_points[:, 1, 0].min(),
            joint_points[:, 2, 0].max() - joint_points[:, 2, 0].min()
        ]).max() * margin

        mid_x = (joint_points[:, 0, 0].max() + joint_points[:, 0, 0].min()) / 2
        mid_y = (joint_points[:, 1, 0].max() + joint_points[:, 1, 0].min()) / 2
        mid_z = (joint_points[:, 2, 0].max() + joint_points[:, 2, 0].min()) / 2

        if fixed_scale:
            max_range = margin
            mid_x = mid_y = mid_z = center

        #self._tp.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        #self._tp.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        #self._tp.ax.set_zlim(mid_z - max_range, mid_z + max_range)

        for frame in frames:
            vector_extractor = np.array([
                [0.0, 1, 0.0, 0.0],
                [0.0, 0.0, 1, 0.0],
                [0.0, 0.0, 0.0, 1],
                [1.0, 1.0, 1.0, 1.0]
            ])
            vectors = np.dot(np.array(frame), vector_extractor)
            origin = vectors[:, 0]
            self._tp.ax.scatter(origin[0],origin[1],origin[2],c='Violet',s=40)

        self._tp.ax.plot3D(
            np.ndarray.flatten(joint_points[:, 0]),
            np.ndarray.flatten(joint_points[:, 1]),
            np.ndarray.flatten(joint_points[:, 2]),
            linewidth=3,
            c='Black',
            alpha=0.4
        )