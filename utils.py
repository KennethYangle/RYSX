import numpy as np

class Utils(object):
    def __init__(self, P=1, D=0.5):
        self.P = P
        self.D = D

    def sat(a: np.array, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

    def PostionController(self, pos_info):
        cmd_vel = self.sat(self.P*(np.array(pos_info["home_pos"]) - np.array(pos_info["mav_pos"])), 20)
        cmd_yawrate = self.sat(self.P*(0-pos_info["mav_yaw"]), 2)
        return cmd_vel, cmd_yawrate

    def DockingController(pos_info, car_velocity):
        cmd_vel = self.sat(self.P*(np.array(pos_info["rel_pos"])-np.array([2,0,2])) + self.D*np.array(pos_info["rel_vel"]), 2*car_velocity)
        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        return cmd_vel, cmd_yawrate