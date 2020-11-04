import numpy as np
from scipy.optimize import fsolve
import copy
import time


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
        self.xf = 1.1 * self.w_run
        self.zf = 1.0 * self.h_raise
        self.ds = self.xf - self.w_run / 2.0
        self.ts = fsolve(
            lambda t: (6 * self.xf / self.t_swing ** 5) * t ** 5 - (15 * self.xf / self.t_swing ** 4) * t ** 4 + (
                    10 * self.xf / self.t_swing ** 3) * t ** 3 - self.ds, 0.01)
        self.time = 0
        self.time_step = time_step
        self.index_legs = 0
        # self.legs = ["FL", "FR", "BL", "BR"]
        self.legs = ["FL", "BR", "FR", "BL"]
        self.T_bf0 = copy.deepcopy(T_bf0)
        self.T_bf = copy.deepcopy(T_bf0)
        self.init = 0

    def reset(self):
        self.time = 0

    def x_coordinate(self):
        x = (6 * self.xf / self.t_swing ** 5) * self.time ** 5 - (15 * self.xf / self.t_swing ** 4) * self.time ** 4 + (
                10 * self.xf / self.t_swing ** 3) * self.time ** 3
        return x

    def z_coordinate(self):
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
        return z

    def GenerateTrajectory(self, contacts):
        # if self.init == 0:
        #     self.init = 1
        #     T_bf = copy.deepcopy(self.T_bf0)
        #     T_bf["FL"][0, 3] += self.xf
        #     T_bf["FR"][0, 3] += 0.1
        #     T_bf["BL"][0, 3] += -self.xf
        #     T_bf["BR"][0, 3] += -0.1
        #     return T_bf

        self.time += self.time_step
        x = self.x_coordinate()
        z = self.z_coordinate()
        T_bf = copy.deepcopy(self.T_bf0)
        coord = np.array([x, 0.0, z])
        T_bf[self.legs[self.index_legs]][0, 3] += coord[0]
        T_bf[self.legs[self.index_legs]][1, 3] += coord[1]
        T_bf[self.legs[self.index_legs]][2, 3] += coord[2]

        if x >= self.xf and z >= self.zf:
            self.reset()
            self.index_legs += 1
            if self.index_legs == 4:
                self.index_legs = 0
                self.T_bf = copy.deepcopy(self.T_bf0)
        return T_bf
