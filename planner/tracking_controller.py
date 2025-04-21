import numpy as np
import copy


class PIDController:
    # PID轨迹跟踪控制器
    def __init__(self, kp, ki, kd, min_output, max_output):
        # PID参数项
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.err = 0        # 比例项
        self.err_all = 0    # 积分项
        self.err_last = 0   # 微分项

        # 其他参数
        self.min_output = min_output
        self.max_output = max_output

    def update(self, state, target):
        self.err = target - state
        self.err_all += self.err
        self.err_last = self.err - self.err_last

        output = self.kp * self.err + self.ki * self.err_all + self.kd * self.err_last

        if output > self.max_output:
            output = self.max_output
        if output < self.min_output:
            output = self.min_output

        return output
