import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time


class DijkstraPlanner(object):
    """
    implementation of Dijkstra planning algorithm on a grid map
    """

    def __init__(self):
        self.start = np.array([1, 1])
        self.goal = np.array([29, 1])
        self.x_axis_max = 30
        self.y_axis_max = 30
        self.num_obstacles = 300
        self.obstacles = self.generate_obstacles(self.num_obstacles)
        self.plan()

    def generate_obstacles(self, num_obs):
        """
        randomly generate a certain number of obstacles within the grid map
        :param num_obs: number of obstacles
        :return: obstacles list
        """
        obs_list = []
        while len(obs_list) < num_obs:
            obs = [random.randint(1, self.x_axis_max - 1), random.randint(1, self.y_axis_max - 1)]
            if obs in obs_list:
                continue
            elif obs[0] == self.goal[0] and obs[1] == self.goal[1]:
                continue
            elif obs[0] == self.start[0] and obs[1] == self.start[1]:
                continue
            else:
                obs_list.append(obs)

        # set grid map boundary as obstacles as well
        for x in range(1, self.x_axis_max):
            obs_list.append([x, 0])
            obs_list.append([x, self.y_axis_max])
        for y in range(1, self.y_axis_max):
            obs_list.append([0, y])
            obs_list.append([self.x_axis_max, y])
        obs_list.append([0, 0])
        obs_list.append([0, self.y_axis_max])
        obs_list.append([self.x_axis_max, 0])
        obs_list.append([self.x_axis_max, self.y_axis_max])

        # print(obs_list)
        # print(len(obs_list))
        return obs_list

    def plan(self):
        """
        find the shortest path
        :return:
        """
        start = time.time()

        # [x,y,cost,parent_x, parent_y]
        open = [np.array([self.start[0], self.start[1], 0, self.start[0], self.start[1]])]
        close = []
        next_move = self.motion_model()
        find_goal_flag = False
        path = []
        ig = 0
        while find_goal_flag == False:
            num_open = len(open)
            if num_open == 0:
                print("no valid path to goal point")
                return path

            for open_id in range(num_open):
                # check if goal point is in open list
                if np.array_equal(open[open_id][:2], self.goal):
                    print("found goal point")
                    close = open + close
                    find_goal_flag = True
                    ig = open_id
                    break

                # iterate through possible movement in 8 directions
                for move_id in range(len(next_move)):
                    # position and cost for adjacent node [x, y, cost]
                    next_node = open[open_id][:3] + next_move[move_id]

                    # check if the adjacent node is obstacle
                    if self.collision_check(next_node, self.obstacles):
                        continue

                    if self.find_list(next_node, open) != -1:
                        # next node is in open list
                        id = self.find_list(next_node, open)
                        if next_node[2] < open[id][2]:
                            # update the new cost and rewire the parent node
                            open[id][2] = next_node[2]
                            open[id][3] = open[open_id][0]
                            open[id][4] = open[open_id][1]
                    elif self.find_list(next_node, close) != -1:
                        # next node is in close list
                        id = self.find_list(next_node, close)
                        if next_node[2] < close[id][2]:
                            close[id][3] = open[open_id][0]
                            close[id][4] = open[open_id][1]
                            open.append(close[id])
                            close.pop(id)
                    else:
                        # next node is in neither open or close list
                        # add the next node into open list
                        open.append(np.append(next_node, open[open_id][:2]))
                ig = open_id

            if find_goal_flag == False:
                # move the computed node from open list to close list
                for open_id in range(num_open):
                    close.append(open[open_id])
                for open_id in range(num_open):
                    # print(open_id, num_open, len(open))
                    open.pop(0)  # pop out the front element

        path = self.get_path(close, ig)
        # print(path)
        end = time.time()
        print("planning time: ", end - start)
        self.visualize(path)

    def visualize(self, path):
        pass
        for item in self.obstacles:
            plt.scatter(item[0], item[1], s=80)
        for item in path:
            plt.scatter(item[0], item[1], c='red')
        plt.scatter(self.start[0], self.start[1], marker="*", c="yellow", s=60)
        plt.scatter(self.goal[0], self.goal[1], marker="*", c="yellow", s=60)
        plt.xlim([-1, self.x_axis_max + 1])
        plt.ylim([-1, self.y_axis_max + 1])
        plt.show()

    def get_path(self, close, ig):
        path = []
        id = ig
        # reverse back from goal point to start point
        while True:
            path.append(close[id][:2])

            # check if start point is reached
            if np.array_equal(close[id][:2], self.start):
                break

            for close_id in range(len(close)):
                if np.array_equal(close[close_id][:2], close[id][3:]):
                    id = close_id
                    break
        return path

    def find_list(self, node, list):
        id = -1
        # is the input node in the list?
        for i in range(len(list)):
            if np.array_equal(node[:2], list[i][:2]):
                id = i
                break
        return id

    def motion_model(self):
        # [x_move, y_move, cost]
        # 8-direction or 4-direction for grid map
        next_move = np.array(
            [[1, 1, 1.414], [1, 0, 1], [1, -1, 1.414], [0, 1, 1], [0, -1, 1], [-1, 1, 1.414], [-1, 0, 1],
             [-1, -1, 1.414]])
        return next_move

    def collision_check(self, node, obstacle_list):
        """
        check if the input node position collides with the obstacle positions
        :param node:
        :param obstacle_list:
        :return: True if collides, otherwise False
        """
        for i in range(len(obstacle_list)):
            if node[0] == obstacle_list[i][0]:
                if node[1] == obstacle_list[i][1]:
                    return True
        return False


if __name__ == "__main__":
    planner = DijkstraPlanner()
