import numpy as np
import matplotlib.pyplot as plt
import math


class DWAPlanner:
    """
    naive implementation of "the dynamic window approach to collision avoidance" 1997
    """
    # TODO: add rotate away recovery behavior

    def __init__(self):
        self.state = np.array([0, 0, math.pi / 4, 0, 0])  # [x, y, yaw, v_linear, v_angular]
        self.obstacles = np.array([[0, 2], [4, 2], [4, 4], [5, 4], [5, 5], [5, 6], [5, 9], [8, 8], [8, 9], [7, 9]])
        # self.obstacles = np.array([[4, 3], [4, 3.5], [4, 4], [4, 4.5], [4, 5]])
        self.obstacle_radius = 0.5  # obstacle radius used to collision check
        self.goal = np.array([10, 10])
        self.dt = 0.1
        self.max_v = 1.0
        self.max_w = math.radians(20.0)
        self.max_acc_linear = 0.2
        self.max_acc_angular = math.radians(40.0)
        self.v_res = 0.01
        self.w_res = math.radians(1)
        self.eval_time = 3.0
        self.cost_param = [0.1, 0.2, 0.1]
        self.alpha = 0.15  # heading weight
        self.beta = 0.2  # clearance weight
        self.gamma = 0.1  # velocity weight

    def calc_dynamic_window(self):
        """
        calculate dynamic window based on current state and dynamic model
        :return: [vmin, vmax, wmin, wmax]
        """
        vs = np.array([0, self.max_v, -1 * self.max_w, self.max_w])  # static
        vd = np.array([self.state[3] - self.max_acc_linear * self.dt, self.state[3] + self.max_acc_linear * self.dt,
                       self.state[4] - self.max_acc_angular * self.dt,
                       self.state[4] + self.max_acc_angular * self.dt])  # dynamic
        vtmp = np.vstack((vs, vd))
        vr = np.array([vtmp[:, 0].max(), vtmp[:, 1].min(), vtmp[:, 2].max(), vtmp[:, 3].min()])
        # print(vr)
        return vr

    def generate_trajectory(self, v, w, state):
        t = 0.0
        traj = state
        while t < self.eval_time:
            t += self.dt
            state = self.move(state, np.array([v, w]))
            traj = np.vstack((traj, state))
        return traj, state

    def move(self, current_state, cmd):
        """
        calculate the next robot state based current state and command
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
        objectives = np.empty([0, 5], dtype=np.float64)
        for v in np.arange(vr[0], vr[1], self.v_res):
            for w in np.arange(vr[2], vr[3], self.w_res):
                [traj, end_state] = self.generate_trajectory(v, w, planner.state)
                heading = self.calc_heading_objective(end_state)
                dist = self.calc_dist_objective(end_state)
                vel = abs(v)
                break_dist = self.calc_break_dist(vel)
                if dist > break_dist:
                    objectives = np.vstack((objectives, np.array([v, w, heading, dist, vel])))

        print(objectives.shape)
        objectives_normalized = self.normalize_objective(objectives)
        objectives_weighted = self.weight_objective(objectives_normalized)
        # TODO: refactor the code below in a pythonic way
        objective_max = 0
        best_cmd = np.array([0, 0])
        for row in objectives_weighted:
            if np.sum(row[2:]) > objective_max:
                best_cmd = row[:2]
                objective_max = np.sum(row[2:])
        return best_cmd

    def weight_objective(self, objectives):
        objectives[:, 2] = objectives[:, 2] * self.alpha
        objectives[:, 3] = objectives[:, 3] * self.beta
        objectives[:, 4] = objectives[:, 4] * self.gamma
        return objectives

    def normalize_objective(self, objectives):
        total = np.sum(objectives, axis=0)
        if total[2] == 0:
            print("divide by 0 (total[2]), runtime error")
        elif total[3] == 0:
            print("divide by 0 (total[3]), runtime error")
        elif total[4] == 0:
            print("divide by 0 (total[4]), runtume error")
        objectives[:, 2] = objectives[:, 2] / total[2]
        objectives[:, 3] = objectives[:, 3] / total[3]
        objectives[:, 4] = objectives[:, 4] / total[4]
        return objectives

    def calc_break_dist(self, v):
        dist = 0.0
        while v > 0:
            dist = dist + v * self.dt
            v -= self.max_acc_linear
        return dist

    def calc_heading_objective(self, state):
        theta = math.degrees(state[2])
        # TODO: check the math
        # use atan2 which takes input sign into consideration and limits output between -PI and PI
        goal_theta = math.degrees(math.atan2((self.goal[1] - state[1]), (self.goal[0] - state[0])))
        # print("[theta, goal_theta]: [%f, %f]", theta, goal_theta)
        objective = 180.0 - abs(goal_theta - theta)
        return objective

    def calc_dist_objective(self, state):
        # TODO: find the minimal distance between obstacle and the curvature (not the end point)
        objective = 10
        # find the minimum distance
        for obstacle in self.obstacles:
            tmp = np.linalg.norm(obstacle - state[:2]) - self.obstacle_radius
            if tmp < objective:
                objective = tmp
        # print(objective)
        return objective


if __name__ == "__main__":
    planner = DWAPlanner()
    odom_path = planner.state
    for i in xrange(1000):
        cmd = planner.run_dwa()
        planner.state = planner.move(planner.state, cmd)
        odom_path = np.vstack((odom_path, planner.state))
        if np.linalg.norm(planner.goal - planner.state[:2]) < 0.5:
            print("goal reached")
            break
            # print(planner.state)
    print odom_path.shape
    plt.scatter(planner.obstacles[:, 0], planner.obstacles[:, 1])
    plt.scatter(odom_path[:, 0], odom_path[:, 1], c='r')
    plt.xlim([-1, 15])
    plt.ylim([-1, 15])
    plt.grid()
    plt.show()
