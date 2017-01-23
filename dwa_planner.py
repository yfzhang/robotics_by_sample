import numpy as np
import matplotlib.pyplot as plt
import math


class DWAPlanner:
    """
    naive immplementation of Dynamic Window Approach planning algorithm
    """

    def __init__(self):
        self.state = np.array([0, 0, math.pi, 0, 0])  # [x, y, yaw, v_linear, v_angular]
        self.obstacle = np.array([])
        self.obstacle_radius = 0.5  # obstacle radius used to collision check
        self.goal = np.array([10,10])
        self.dt = 0.1
        self.max_v = 1.0
        self.max_w = math.radians(20.0)
        self.max_acc_linear = 0.2
        self.max_acc_angular = math.radians(1.0)
        self.v_res = 0.1
        self.w_res = math.radians(1)
        self.eval_time = 3.0
        self.cost_param = [0.1, 0.2, 0.1]

    def calc_dynamic_window(self):
        """
        calculate dynamic window based on current state and dynamic model
        :return: [vmin, vmax, wmin, wmax]
        """
        vs = np.array([0, self.max_v, -1 * self.max_w, self.max_w])
        vd = np.array([self.state[3] - self.max_acc_linear * self.dt, self.state[3] + self.max_acc_linear * self.dt,
                       self.state[4] - self.max_acc_angular * self.dt, self.state[4] + self.max_acc_angular * self.dt])
        vtmp = np.array([vs, vd])
        vr = np.array([vtmp[:, 0].max(), vtmp[:, 1].min(), vtmp[:, 2].max(), vtmp[:, 3].min()])
        return vr

    def generate_trajectory(self, v, w, state):
        cmd = np.array([v, w])
        t = 0.0
        traj = state
        while t < self.eval_time:
            t += self.dt
            state = self.move(state, cmd)
            traj = np.vstack((traj, state))
        return traj, state

    def move(self, current_state, cmd):
        """
        calcute the next robot state based current state and command
        x = A*x + B*u
        :param current_state:
        :param command: [v, w]
        :return: next_state
        """
        A = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
        B = np.array(
            [[self.dt * math.cos(current_state[2]), 0], [self.dt * math.sin(current_state[2]), 0], [0, self.dt],
             [1, 0], [0, 1]])
        next_state = np.dot(A, current_state) + np.dot(B, cmd)
        return next_state

    def run_dwa(self):
        vr = self.calc_dynamic_window()
        for v in range(vr[0], vr[1], self.v_res):
            for w in range(vr[2], vr[3], self.w_res):
                [traj, end_state] = self.generate_trajectory(v,w,planner.state)


        return cmd, traj

    def calc_heading_cost(self, state):
        theta = math.degrees(state[2])
        # TODO: compute correct goal theta
        goal_theta = math.degrees(math.atan2((self.goal[1]-state[1]),(self.goal[0]-state[0])))
        if goal_theta > theta:
            cost = 180.0 - (goal_theta - theta)
        else:
            cost = 180.0 - (theta - goal_theta)
        return cost

    def calc_dist_cost(self, state):




if __name__ == "__main__":
    planner = DWAPlanner()
    odom_path = np.array([])
    for i in xrange(1000):
        odom_path = np.vstack((odom_path, planner.state))
        [cmd, traj] = planner.run_dwa()
        planner.state = planner.move(planner.state, cmd)
        if np.linalg.norm(planner.goal-planner.state[:2]) < 1.0:
            print("goal reached")
            break
        print(planner.state)

