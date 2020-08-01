import numpy as np
import time
from utils import Utils

# 初始化状态
class InitializeState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "InitializeState"
    def waitForInitialize(self):
        time.sleep(1)
    def initializeFinished(self):
        self.stateMachine.setState(self.stateMachine.getIdleState())
    def takeoff(self, pos_info):
        print("cannot takeoff, waiting for initialize.")
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())


# 空闲状态
class IdleState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "IdleState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getTakeoffState())
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())

# 起飞状态
class TakeoffState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "TakeoffState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        """
        return: [vel_cmd], is_takeoff_finish
        """
        # sendTakeoffCmdAsyn()
        if pos_info["mav_pos"][2] > -2:
            return [0,0,-1,0]
        else:
            self.stateMachine.setState(self.stateMachine.getDockingState())
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())


# 对接状态
class DockingState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "DockingState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        pass
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def approach(self, pos_info, car_velocity):
        cmd = self.stateMachine.util.DockingController(pos_info, car_velocity)
        return cmd


# 返航状态
class HomewardState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "HomewardState"
        self.homeward = True
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        pass
    def go_home(self, pos_info):
        if self.homeward and np.linalg.norm(np.array(pos_info["mav_pos"]) - np.array(pos_info["home_pos"])) >= 1:
            cmd = self.stateMachine.util.PostionController(pos_info)
            return cmd
        else:
            self.homeward = False
            return [0,0,1,0]


# 状态机
class StateMachine(object):
    def __init__(self):
        self.ch5 = -1
        self.ch6 = -1
        self.ch9 = -1
        self.ch10 = -1
        self.is_initialize_finish = False

        self.start_key = -1
        self.homeward_key = -1

        self.initialize_state = InitializeState(self)
        self.idle_state = IdleState(self)
        self.takeoff_state = TakeoffState(self)
        self.docking_state = DockingState(self)
        self.homeward_state = HomewardState(self)

        self.state = self.initialize_state
        self.state_name = "InitializeState"

        self.util = Utils()

    def getInitializeState(self):
        return self.initialize_state
    def getIdleState(self):
        return self.idle_state
    def getTakeoffState(self):
        return self.takeoff_state
    def getDockingState(self):
        return self.docking_state
    def getHomewardState(self):
        return self.homeward_state
    def setState(self, state):
        self.state = state
        self.state_name = state.state_name

    def update(self, keys, is_initialize_finish, pos_info, car_velocity):
        """
        - keys: 按键状态
        - is_initialize_finish: 外部订阅的
        - pos_info: a dict contain absolute position and relative position, 
        {"mav_pos": mav_pos, "mav_yaw": mav_yaw, "home_pos": [0,0,0], "rel_pos": dlt_pos, "rel_vel": dlt_vel, "rel_yaw": dlt_yaw}
        - car_velocity: use the velocity of car as base value
        """
        self.ch5, self.ch6, self.ch9, self.ch10 = keys
        self.start_key = self.ch10
        self.homeward_key = self.ch9
        self.offboard_key = self.ch5

        if self.homeward_key:
            return self.state.go_home(pos_info)
        self.is_initialize_finish = is_initialize_finish
        if not self.is_initialize_finish:
            self.state.waitForInitialize()
        else:
            self.state.initializeFinished()
        if self.start_key == 1:
            cmd = self.state.takeoff(pos_info)
            if cmd is not None:
                return cmd
        if self.state_name == "DockingState":
            return self.state.approach(pos_info, car_velocity)
