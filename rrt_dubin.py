import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import dubins_path


class Obstacle(object):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def plot(self, color='k'):
        theta = np.linspace(0, np.pi*2, num=30)
        x = self.x + self.r * np.cos(theta)
        y = self.y + self.r * np.sin(theta)

        plt.plot(x, y, color=color)

    def is_inside(self, x, y):
        dist = np.hypot(x - self.x, y - self.y)
        if dist <= self.r:
            return True
        else:
            return False

def SampleFree(space, goal):
    min_x, max_x, min_y, max_y = space
    if np.random.rand() > 0.2:
        rand_x = np.random.uniform(min_x, max_x)
        rand_y = np.random.uniform(min_y, max_y)
    else:
        rand_x = goal[0]
        rand_y = goal[1]

    node_rand = {"x": rand_x, "y": rand_y}
    return node_rand

def Nearest(G, node_rand):
    min_dist = 1e9
    for v in G.nodes:
        node = G.nodes[v]
        dist = np.hypot(node_rand["x"] - node["x"], node_rand["y"] - node["y"])
        if dist < min_dist:
            node_nearest = node
            node_nearest_id = v
            min_dist = dist
    return node_nearest, node_nearest_id

def Steer(node_nearest, node_rand):
    # step = 3.0
    step = np.random.uniform(1.0, 5.0)

    dx = node_rand["x"] - node_nearest["x"]
    dy = node_rand["y"] - node_nearest["y"]
    magnitude = np.hypot(dx, dy)

    # node_new = node_nearest + dir * step
    x_new = node_nearest["x"] + dx/magnitude * step
    y_new = node_nearest["y"] + dy/magnitude * step

    return {"x": x_new, "y": y_new}

def ObstacleFree(node_nearest, node_new, obstacles, path_x, path_y):
    step = 0.2
    dx = node_new["x"] - node_nearest["x"]
    dy = node_new["y"] - node_nearest["y"]
    magnitude = np.hypot(dx, dy)

    # u = 0.0 --> node_nearest
    # u = magnitude --> node_new
    u = 0.0
    us = []
    while u < magnitude:
        us.append(u)
        u += step
    us.append(magnitude)

    for i in range(len(path_x)):
        x_new = path_x[i]
        y_new = path_y[i]
        for obstacle in obstacles:
            collide = obstacle.is_inside(x_new, y_new)
            if collide:
                return False
    return True

def IsGoal(node_new, goal):
    dist = np.hypot(node_new["x"] - goal[0], node_new["y"] - goal[1])
    if dist < 1.0:
        return True
    else:
        return False


if __name__ == '__main__':
    dubins = dubins_path.Dubins()
    kappa_ = 1. / 6
    ## environment setup
    np.random.seed(83824732)
    min_x, max_x = -20, 20
    min_y, max_y = -20, 20

    space = [min_x, max_x, min_y, max_y]

    start = np.random.uniform(low=-20, high=-5, size=2)
    goal = np.random.uniform(low=5, high=20, size=2)

    obstacles = []
    for i in range(25):
        x = np.random.uniform(low=min_x, high=max_x, size=1)
        y = np.random.uniform(low=min_y, high=max_y, size=1)
        r = np.random.uniform(low=1.0, high=5.0)
        obstacle = Obstacle(x, y, r)

        if not obstacle.is_inside(start[0], start[1]) and not obstacle.is_inside(goal[0], goal[1]):
            obstacles.append(obstacle)

    ## algorithm
    G = nx.DiGraph()
    G.add_nodes_from([
        (-1, {"x": start[0], "y": start[1]})
    ])

    max_iterations = 500
    goal_node_id = None
    path_xs = []
    path_ys = []
    for i in range(max_iterations):
        node_rand = SampleFree(space, goal)
        node_nearest, node_nearest_id = Nearest(G, node_rand)

        start_state = [node_nearest["x"], node_nearest["y"], np.random.uniform(0, np.pi * 2)]
        goal_state = [node_rand["x"],node_rand["y"], np.random.uniform(0, np.pi * 2)]
        cartesian_path, controls, dubins_path = dubins.plan(start_state, goal_state, kappa_)
        path_x, path_y, path_yaw = cartesian_path

        if ObstacleFree(node_nearest, node_rand, obstacles,path_x,path_y):
            G.add_nodes_from([
                (i, node_rand)
            ])
            G.add_edge(node_nearest_id, i)

            path_xs.append(path_x)
            path_ys.append(path_y)


            if IsGoal(node_rand, goal):
                goal_node_id = i
                break

    if goal_node_id is None:
        print(" [!] Cannot find path")
    else:
        path = nx.shortest_path(G, source=-1, target=goal_node_id)

    ## visualize
    for obs in obstacles:
        obs.plot()

    plt.plot(start[0], start[1], 'ro', ms=8)
    plt.plot(goal[0], goal[1], 'rx', ms=10)
    for v in G.nodes:
        plt.plot(G.nodes[v]["x"], G.nodes[v]["y"], 'b.')
        plt.text(G.nodes[v]["x"], G.nodes[v]["y"], v)
    for k in range(len(path_xs)):
        plt.plot(path_xs[k], path_ys[k], 'r-')


    # for e in G.edges:
    #     # e = [v_from, v_to]
    #     v_from = G.nodes[e[0]]
    #     v_to = G.nodes[e[1]]
    #
    #     plt.plot([v_from["x"], v_to["x"]], [v_from["y"], v_to["y"]], 'b-')

    if goal_node_id is not None:
        path_x = []
        path_y = []
        for v in path:
            path_x.append(G.nodes[v]["x"])
            path_y.append(G.nodes[v]["y"])
        #plt.plot(path_x, path_y, 'r-')

    plt.axis("equal")
    plt.show()