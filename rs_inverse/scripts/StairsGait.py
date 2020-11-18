import numpy as np
from scipy.optimize import fsolve
import copy
import time
from collections import OrderedDict


class StairsGait:
    def __init__(self, h_raise, w_run, beta, delta_h, t_swing, T_bf0, time_step=0.01):
        """
        :param h_raise: height of each stair
        :param w_run: length of each stair
        :param beta: duty factor
        :param delta_h: is  a  small  positive number
        :param t_swing: is  a  small  positive number
        :param time_step: is  sample time
        """
        self.h_raise = h_raise
        self.w_run = w_run
        self.beta = beta
        self.delta_h = delta_h
        self.t_swing = t_swing
        # self.xf = 2.0 * self.w_run
        # self.zf = 2.0 * self.h_raise
        self.xf = 1.0 * self.w_run
        self.zf = 1.0 * self.h_raise
        self.ds = self.xf-0.1
        self.ts = fsolve(
            lambda t: (6 * self.xf / self.t_swing ** 5) * t ** 5 - (15 * self.xf / self.t_swing ** 4) * t ** 4 + (
                    10 * self.xf / self.t_swing ** 3) * t ** 3 - self.ds, 0.1)
        self.time = 0
        self.time_step = time_step
        self.index_legs = 0
        self.legs = ["FL", "FR", "BL", "BR"]
        # self.legs = ["FL", "BR", "FR", "BL"]
        self.T_bf0 = copy.deepcopy(T_bf0)
        self.T_bf = copy.deepcopy(T_bf0)
        self.Legs_pose = OrderedDict()
        self.Legs_pose["FL"] = np.array([0, 0, 0])
        self.Legs_pose["FR"] = np.array([0, 0, 0])
        self.Legs_pose["BL"] = np.array([0, 0, 0])
        self.Legs_pose["BR"] = np.array([0, 0, 0])
        self.phase_index = 0


    def reset(self):
        self.time = 0

    def coordinate(self):
        x = (6 * self.xf / self.t_swing ** 5) * self.time ** 5 - (15 * self.xf / self.t_swing ** 4) * self.time ** 4 + (
                10 * self.xf / self.t_swing ** 3) * self.time ** 3 - self.xf/2

        if self.time <= self.ts:
            z = (6 * (self.h_raise + self.delta_h) / self.ts ** 5) * self.time ** 5 - (
                    15 * (self.h_raise + self.delta_h) / self.ts ** 4) * self.time ** 4 + (
                        10 * (self.h_raise + self.delta_h) / self.ts ** 3) * self.time ** 3
        else:
            z = (6 * (self.zf - (self.h_raise + self.delta_h))) / (self.t_swing - self.ts) ** 5 * (
                        self.time - self.ts) ** 5 - (15 * (self.zf - (self.h_raise + self.delta_h))) / (
                            self.t_swing - self.ts) ** 4 * (self.time - self.ts) ** 4 + (
                            10 * (self.zf - (self.h_raise + self.delta_h))) / (self.t_swing - self.ts) ** 3 * (
                            self.time - self.ts) ** 3 + (self.h_raise + self.delta_h)
        y = 0.0
        coord = np.array([x, y, z])
        return coord

    def legs_pose(self):
        if self.phase_index == 0:
            self.Legs_pose["FL"] = np.array([-self.xf/2, 0, 0])
            self.Legs_pose["FR"] = np.array([-self.xf/2, 0, 0])
            self.Legs_pose["BL"] = np.array([-self.xf/2, 0, 0])
            self.Legs_pose["BR"] = np.array([-self.xf/2, 0, 0])
            print("phase0")
        elif self.phase_index == 1:
            self.index_legs = 0
            coord = self.coordinate()
            self.Legs_pose["FL"] = coord
            print("phase1")
        elif self.phase_index == 2:
            self.index_legs = 1
            coord = self.coordinate()
            self.Legs_pose["FR"] = coord
            print("phase2")
        elif self.phase_index == 3:
            self.index_legs = 0
            coord = self.coordinate()
            self.Legs_pose["FL"] = coord + np.array([self.xf, 0, 0])
            self.Legs_pose["FR"] = np.array([-self.xf, 0, 0])
            self.Legs_pose["BL"] = np.array([-self.xf, 0, 0])
            self.Legs_pose["BR"] = np.array([-self.xf, 0, 0])
            print("phase3")
        elif self.phase_index == 4:
            self.index_legs = 1
            coord = self.coordinate()
            self.Legs_pose["FR"] = coord + np.array([self.xf, 0, 0])
            self.Legs_pose["BL"] = np.array([-self.xf, 0, 0])
            self.Legs_pose["BR"] = np.array([-self.xf, 0, 0])
            print("phase4")
        # elif self.phase_index == 4:
        #     self.index_legs = 3
        #     coord = self.coordinate()
        #     self.Legs_pose["BR"] = coord
        #     print("phase3")


    def GenerateTrajectory(self, contacts):
        self.time += self.time_step
        self.legs_pose()
        T_bf = copy.deepcopy(self.T_bf0)
        for i, (key, Tbf_in) in enumerate(T_bf.items()):
            T_bf[key][0, 3] = Tbf_in[0, 3] + self.Legs_pose[key][0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + self.Legs_pose[key][1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + self.Legs_pose[key][2]
        if self.phase_index == 0 and self.time >= self.t_swing:
            self.reset()
            self.phase_index += 1
            time.sleep(1.3)
        if self.phase_index > 0 and self.time > self.t_swing/3 and contacts[self.index_legs] == 1:
            self.reset()
            self.phase_index += 1
        return T_bf
