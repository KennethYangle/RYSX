import numpy as np

class Utils(object):
    def __init__(self, P=0.5, D=0.5, P_i=0.005):
        self.P = P
        self.D = D
        self.P_i = P_i

    def sat(self, a: np.array, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

    def PostionController(self, pos_info):
        cmd_vel = self.sat(self.P*(np.array(pos_info["home_pos"]) - np.array(pos_info["mav_pos"])), 30)
        cmd_yawrate = self.sat(self.P*(0-pos_info["mav_yaw"]), 2)
        return [cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yawrate]

    def DockingController(self, pos_info, pos_i, car_velocity):
        if pos_info["rel_pos"][0] > 5:
            self.P = 2
        elif pos_info["rel_pos"][0] > 2:
            self.p = 1
        elif pos_info["rel_pos"][0] > 1:
            self.P = 0.5
        else:
            self.P = 0.1
        cmd_vel = self.sat(self.P*(np.array(pos_info["rel_pos"])-np.array([-1,0,3])) + self.D*np.array(pos_info["rel_vel"]), 3*car_velocity)
        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        if pos_i[0] != -1:
            v_zi = self.P_i * (pos_i[1] - 202.5)
            print("pos_i: {}".format(pos_i))
            return [cmd_vel[0], cmd_vel[1], v_zi, cmd_yawrate]
        else:
            return [cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yawrate]

    def IsInFence(self, pos, geo_fence):
        if pos[0] > geo_fence[0] and pos[0] < geo_fence[2] and pos[1] > geo_fence[1] and pos[1] < geo_fence[3]:
            return True
        else:
            return False