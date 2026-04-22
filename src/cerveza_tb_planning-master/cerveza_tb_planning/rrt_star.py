#!/usr/bin/env python3
import numpy as np
import math


class Point:
    """ Simple 2D point class """
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    """ Euclidean distance between two points """
    def dist(self, p):
        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    """ Unit direction (length 1) """
    def unit_vector(self, p):
        v = Point(p.x - self.x, p.y - self.y)
        n = v.norm()
        if n == 0:
            return Point(0, 0)
        return Point(v.x / n, v.y / n)

    """ Vector length """
    def norm(self):
        return math.sqrt(self.x**2 + self.y**2)

    """ Point + Point """
    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    """ Point * scalar """
    def __mul__(self, s):
        return Point(self.x * s, self.y * s)

    """ scalar * Point """
    def __rmul__(self, s):
        return self.__mul__(s)

    """ Point1 == Point2 """
    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    """ Point as key """
    def __hash__(self):
        return hash((self.x, self.y))

    """ print(Point) """
    def __str__(self):
        return f"({self.x:.2f}, {self.y:.2f})"


class RRTStar:
    """
    RRT* path planner.
    Works with an external is_valid function provided by the ROS node.
    All coordinates are in world frame (meters).
    
    Usage:
        planner = RRTStar(is_valid_fn, x_min, x_max, y_min, y_max)
        path = planner.plan(start=(x, y), goal=(x, y))
        # path = [(x1,y1), (x2,y2), ...] or None
    """

    def __init__(self, is_valid_fn, x_min, x_max, y_min, y_max,
                 delta_q=0.3, radius=1.0, goal_bias=0.1, max_iter=3000, goal_tolerance=0.2):
        """
        Args:
            is_valid_fn: function(x, y) -> bool. True if position is free.
            x_min, x_max, y_min, y_max: map bounds in meters.
            delta_q: step size in meters.
            radius: rewire radius in meters.
            goal_bias: probability of sampling the goal.
            max_iter: maximum iterations.
            goal_tolerance: distance to consider goal reached.
        """
        self.is_valid = is_valid_fn
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.delta_q = delta_q
        self.radius = radius
        self.goal_bias = goal_bias
        self.max_iter = max_iter
        self.goal_tolerance = goal_tolerance

    def set_map_min_max(self, x_min, x_max, y_min, y_max):
        """ Update the map bounds for sampling """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        Args:
            start: tuple (x, y) in meters
            goal: tuple (x, y) in meters

        Returns:
            list of (x, y) tuples representing the path, or None if no path found.
        """
        q_start = Point(start[0], start[1])
        q_goal = Point(goal[0], goal[1])

        # Graph
        V = [q_start]   # Tree vertices
        E = {}          # Tree edges (child:parent)
        J = {0: 0.0}    # Cost (from start to each node)

        best_path = None
        best_cost = float('inf')

        # Each iteration aims to grow the tree one step
        for i in range(self.max_iter):
            # Sample random point
            q_rand = self._random_conf(q_goal)

            # Find nearest vertex
            idx_near, q_near = self._nearest_vertex(V, q_rand)

            # Steer towards random point
            q_new = self._new_conf(q_near, q_rand)

            # Check if segment is collision free
            if not self._is_segment_free(q_near, q_new):
                continue

            # Add new vertex, edge and cost
            idx_new = len(V)
            V.append(q_new)
            E[idx_new] = idx_near
            J[idx_new] = J[idx_near] + q_near.dist(q_new)

            # Find neighbors within radius
            neighbors = self._vertices_within_r(V, q_new)

            # Rewire: find best parent for q_new (by cost)
            best_parent = idx_near
            best_parent_cost = J[idx_new]

            for idx_nei in neighbors:
                if idx_nei == idx_new:
                    continue
                if not self._is_segment_free(V[idx_nei], q_new):
                    continue
                cost = J[idx_nei] + V[idx_nei].dist(q_new)
                if cost < best_parent_cost:
                    best_parent_cost = cost
                    best_parent = idx_nei

            E[idx_new] = best_parent        # Update parent
            J[idx_new] = best_parent_cost   # Update cost

            # Rewire: check if neighbors benefit from going through q_new
            for idx_nei in neighbors:
                if idx_nei == idx_new:
                    continue
                if not self._is_segment_free(q_new, V[idx_nei]):
                    continue
                alt_cost = J[idx_new] + q_new.dist(V[idx_nei])  # Alternative cost
                if alt_cost < J[idx_nei]:
                    E[idx_nei] = idx_new    # Update parent
                    J[idx_nei] = alt_cost   # Update cost

            # Check if goal is reached and update the tree and cost
            if q_new.dist(q_goal) < self.goal_tolerance:
                idx_goal = len(V)
                V.append(q_goal)
                E[idx_goal] = idx_new
                J[idx_goal] = J[idx_new] + q_new.dist(q_goal)

                # Check if it is the best path
                if J[idx_goal] < best_cost:
                    best_cost = J[idx_goal]
                    best_path = self._extract_path(V, E, idx_goal)

        return best_path

    def _random_conf(self, q_goal):
        """ Sample a random point, with goal_bias probability of sampling the goal """
        if np.random.rand() < self.goal_bias:
            return q_goal
        x = np.random.uniform(self.x_min, self.x_max)   # Use map bounds
        y = np.random.uniform(self.y_min, self.y_max)
        return Point(x, y)

    def _nearest_vertex(self, V, q_rand):
        """ Find the nearest vertex in the tree to q_rand """
        min_dist = float('inf')
        idx_near = 0
        for i, v in enumerate(V):
            d = v.dist(q_rand)
            if d < min_dist:
                min_dist = d
                idx_near = i
        return idx_near, V[idx_near]

    def _new_conf(self, q_near, q_rand):
        """ Steer from q_near towards q_rand by delta_q """
        if q_near.dist(q_rand) <= self.delta_q:     # Check if q_rand is the nearest
            return q_rand
        uv = q_near.unit_vector(q_rand)
        return Point(q_near.x + uv.x * self.delta_q, q_near.y + uv.y * self.delta_q)

    def _is_segment_free(self, q1, q2):
        """ Check if segment between q1 and q2 is collision free """
        dist = q1.dist(q2)
        if dist == 0:
            return self.is_valid(q1.x, q1.y)    # Is already there

        # Check points along the segment every 5cm
        n_checks = max(int(dist / 0.05), 2)
        for i in range(n_checks + 1):
            t = i / n_checks    # Interpolate between q1 and q2
            x = q1.x + t * (q2.x - q1.x)
            y = q1.y + t * (q2.y - q1.y)
            if not self.is_valid(x, y):
                return False
        return True

    def _vertices_within_r(self, V, q):
        """ Find all vertices within radius r of q """
        indices = []
        for i, v in enumerate(V):
            if v.dist(q) <= self.radius:
                indices.append(i)
        return indices

    def _extract_path(self, V, E, idx_goal):
        """ Extract path as list of (x, y) tuples from start to goal """
        path = []
        current = idx_goal  # Start with the goal

        # While the node has a parent:
        while current in E:
            path.append((V[current].x, V[current].y))
            current = E[current]        # Update parent
        path.append((V[0].x, V[0].y))   # Add start
        path.reverse()                  # Invert: start -> goal
        return path
    
    def smooth_path(self, path):
        """ Remove unnecessary waypoints if direct connection is free """
        if path is None or len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current = 0
        
        while current < len(path) - 1:
            # Try to connect to the farthest reachable waypoint
            farthest = current + 1
            for i in range(len(path) - 1, current, -1):
                q1 = Point(path[current][0], path[current][1])
                q2 = Point(path[i][0], path[i][1])
                if self._is_segment_free(q1, q2):
                    farthest = i
                    break
            smoothed.append(path[farthest])
            current = farthest
        
        return smoothed