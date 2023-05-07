import numpy as np
from collections import namedtuple

PI = np.pi
INF = np.inf

Interval = namedtuple('Interval', ['lower', 'upper']) # AABB
UNIT_LIMITS = Interval(0., 1.)

resolutions= [0.5,0.5,0.5,0.5,0.5,0.5]

def uniform_generator(d):
    while True:
        yield np.random.uniform(size=d)

def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)

def interval_generator(lower, upper):
    assert len(lower) == len(upper)
    assert np.less_equal(lower, upper).all()
    if np.equal(lower, upper).all():
        return iter([lower])
    return (convex_combination(lower, upper, w=weights) for weights in uniform_generator(d=len(lower)))

def get_sample_fn():
    lower_limits, upper_limits = [-360,-180,-130,-360,-360,-360], [360,0,130,360,360,360]
    generator = interval_generator(lower_limits, upper_limits)
    def fn():
        return tuple(next(generator))
    return fn

def get_aabb_extent(aabb):
    lower, upper = aabb
    return np.array(upper) - np.array(lower)

def circular_interval(lower=-PI): # [-np.pi, np.pi)
    return Interval(lower, lower + 2*PI)

def wrap_interval(value, interval=UNIT_LIMITS):
    lower, upper = interval
    if (lower == -INF) and (+INF == upper):
        return value
    assert -INF < lower <= upper < +INF
    print(value,lower,upper)
    return (value - lower) % (upper - lower) + lower

def circular_difference(theta2, theta1, **kwargs):
    interval = circular_interval(**kwargs)
    #extent = get_interval_extent(interval) # TODO: combine with motion_planners
    extent = get_aabb_extent(interval)
    diff_interval = Interval(-extent/2, +extent/2)
    difference = wrap_interval(theta2 - theta1, interval=diff_interval)
    #difference = interval_difference(theta2, theta1, interval=interval)
    return difference

def get_difference_fn():
    def fn(q2, q1):
        return tuple(circular_difference(value2, value1) for value2, value1 in zip(q2, q1))
    return fn

def get_distance_fn(weights=None, norm=2):
    difference_fn = get_difference_fn()
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        if norm == 2:
            return np.sqrt(np.dot(weights, diff * diff))
        return np.linalg.norm(np.multiply(weights, diff), ord=norm)
    return fn

def wrap_angle(theta, **kwargs):
    return wrap_interval(theta, interval=circular_interval(**kwargs))

def wrap_position(position):
    wrap_angle(position)

def wrap_positions(positions):
    assert 6 == len(positions)
    return [wrap_position(position)
            for position in zip(positions)]

def get_refine_fn(num_steps=0):
    difference_fn = get_difference_fn()
    num_steps = num_steps + 1
    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            positions = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            #q = tuple(positions)
            q = tuple(wrap_positions(positions)) # TODO: possible issue with adjust path
            yield q
    return fn

def get_extend_fn(resolutions, norm=2):
    # norm = 1, 2, INF
    difference_fn = get_difference_fn()
    def fn(q1, q2):
        #steps = int(np.max(np.abs(np.divide(difference_fn(q2, q1), resolutions))))
        steps = int(np.linalg.norm(np.divide(difference_fn(q2, q1), resolutions), ord=norm))
        refine_fn = get_refine_fn(num_steps=steps)
        return refine_fn(q1, q2)
    return fn

from random import random
import time

# from .utils import irange, argmin, RRT_ITERATIONS, apply_alpha, RED, INF, elapsed_time
RRT_ITERATIONS = 20

def apply_alpha(color, alpha=1.):
   return tuple(color[:3]) + (alpha,)


def irange(start, stop=None, step=1):  # np.arange
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent

    #def retrace(self):
    #    if self.parent is None:
    #        return [self]
    #    return self.parent.retrace() + [self]

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__

def elapsed_time(start_time):
    return time.time() - start_time

def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))

def argmin(function, sequence):
    # TODO: use min
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.2, max_iterations=RRT_ITERATIONS, max_time=INF):

    """
    :param start: Start configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """

    start_time = time.time()
    if collision_fn(start):
        print("The start pos is in collision:")
        return None
    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)]
    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        goal = random() < goal_probability or i == 0
        s = goal_sample() if goal else sample_fn()

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        print("the 2 configs are",last.config,s)
        for q in extend_fn(last.config, s):
            if collision_fn(q):
                break
            last = TreeNode(q, parent=last)
            nodes.append(last)
            if goal_test(last.config):
                return configs(last.retrace())
        else:
            if goal:
                return configs(last.retrace())
    return None