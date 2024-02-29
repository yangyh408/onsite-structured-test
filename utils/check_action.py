import numpy as np

def check_action(dt, prev_action, new_action):
    ACC_LIMIT = 9.8         # m/s^2
    JERK_LIMIT = 49.0       # m/s^3
    ROT_LIMIT = 0.7         # rad
    ROT_RATE_LIMIT = 1.4    # rad/s

    checked_acc, checked_rot = new_action

    if not np.isnan(prev_action[0]):
        # 检验加速度
        jerk = (new_action[0] - prev_action[0]) / dt
        if abs(jerk) > JERK_LIMIT:
            jerk = np.clip(jerk, -JERK_LIMIT, JERK_LIMIT)
            checked_acc = prev_action[0] + jerk * dt

    if not np.isnan(prev_action[1]):
        # 检验前轮转角
        rot_rate = (new_action[1] - prev_action[1]) / dt
        if abs(rot_rate) > ROT_RATE_LIMIT:
            rot_rate = np.clip(rot_rate, -ROT_RATE_LIMIT, ROT_RATE_LIMIT)
            checked_rot = prev_action[1] + rot_rate * dt

    return [np.clip(checked_acc, -ACC_LIMIT, ACC_LIMIT), np.clip(checked_rot, -ROT_LIMIT, ROT_LIMIT)]

if __name__ == '__main__':
    dt = 0.1  # 假设时间间隔为1秒
    prev_action = [5.0, 0.6]
    new_action = [2, 0.71]

    result = check_action(dt,prev_action, new_action)
    print(result)