import numpy as np
import cv2 as cv
import math
import random

IM_DIR = '/home/ctlattez/python_repos/logo_pathplanning/northwestern-n.jpg'

BGR_IM = cv.imread(IM_DIR, cv.IMREAD_COLOR)  # BGR
BIN_IM = cv.cvtColor(BGR_IM, cv.COLOR_BGR2GRAY)
# keep track of obstacles and previously drawn lines
cv.threshold(BIN_IM, 125, 255, cv.THRESH_BINARY, BIN_IM)
cv.namedWindow('final im', cv.WINDOW_NORMAL)

# dict : {point: {parents}}
ITER_MAX = 500
i = 0
H, W = BIN_IM.shape
CLIP_D = float(30)

random.seed()  # random generator from current time


def isObstacle(point: tuple) -> bool:
    return BIN_IM[point[0], point[1]] == 0

# give a tuple of a new point. It is not obstacle or added point by getting verified in bin_map.


def getAvailableRandomPoint() -> tuple:

    while (1):  # should have error check inside or random point limit
        # random the point within the image
        y = random.randint(0, H)
        x = random.randint(0, W)

        # check the availability
        if not isObstacle((y, x)):
            return (y, x)


def markBinMap(point: tuple, mark_black: bool = True):
    BIN_IM[point[0], point[1]] = 255 * (int)(not mark_black)


def markBGRMap(point: tuple, bgr_color: list):
    BGR_IM[point[0], point[1], 0] = bgr_color[0]
    BGR_IM[point[0], point[1], 1] = bgr_color[1]
    BGR_IM[point[0], point[1], 2] = bgr_color[2]


BGR_YELLOW = [0, 255, 255]
BGR_RED = [0, 0, 255]
BGR_BLACK = [0, 0, 0]


END_POINT = (190,190)
START_POINT = (10,10)
# initialize different end_point and next_point
# end_point = getAvailableRandomPoint()
end_point = END_POINT
markBinMap(end_point)
markBGRMap(end_point, BGR_YELLOW)
next_point = end_point  # init dummy next point
marked_points = dict()


def getClosestPoint(point: tuple):
    min_d = float('inf')
    min_d_point = tuple()

    if len(marked_points) == 0:
        raise RuntimeError('marked_points list is empty, can\'t be iterated.')

    for prev_point in marked_points:
        d_y = point[0] - prev_point[0]
        d_x = point[1] - prev_point[1]
        d_norm = math.sqrt(d_y*d_y + d_x*d_x)
        if d_norm < min_d:
            min_d = d_norm
            min_d_point = prev_point
    return min_d_point, min_d

# return a path: a list of tuples from start_point to end_point
def getPath(start_point: tuple, end_point: tuple):

    if start_point == end_point:
        raise RuntimeError('get same start and endpoint')

    x1 = start_point[0]
    y1 = start_point[1]
    x2 = end_point[0]
    y2 = end_point[1]

    if x1 == x2:  # vertical line
        step = 1 if y1 < y2 else -1
        return [(x1, v) for v in range(y1, y2 + step, step)]
    elif y1 == y2:  # horizontal line
        step = 1 if x1 < x2 else -1
        return [(v, y1) for v in range(x1, x2 + step, step)]

    m = (y2 - y1) / (x2 - x1)
    b = -y1 + m * x1

    long_x = abs(m) < 1
    step = 1 # init
    if (long_x and x2 < x1) or (not long_x and y2 < y1):
        step = -1
    
    ret_list = []
    if long_x:
        for x in range(x1, x2 + step, step):
            y = m * x + b
            y = int(math.floor(y + 0.5))
            ret_list.append((x, y))
    else:
        for y in range(y1, y2 + step, step):
            x = (y - b) / m
            x = int(math.floor(x + 0.5))
            ret_list.append((x, y))
    return ret_list

def getUnblockedPath(path):
    index = 0
    while index < len(path):
        pt = path[index] 
        # print(pt)
        if BIN_IM[pt[0]][pt[1]] == 0: # obstacle
            return path[:index]
        index += 1
    return path


# RRT process
while i < ITER_MAX:

    if i == 0:
        while next_point != end_point:
            next_point = getAvailableRandomPoint()
        marked_points[next_point] = None  # None == no parent == starting point

    i += 1

    # get the next point
    next_point = getAvailableRandomPoint()
    closest_point, closest_d = getClosestPoint(next_point)

    # If over, clip the distance of the closest and the new point to be within desired radius
    # if closest_d > CLIP_D:
    #     clip_d_ratio = CLIP_D / closest_d
    #     dy = next_point[0] - closest_point[0]
    #     dx = next_point[1] - closest_point[1]
    #     next_point = (closest_point[0] + int(clip_d_ratio * dy), closest_point[1] + int(clip_d_ratio * dx))
    #     # next_point[1] = )
    #     closest_d = CLIP_D

    # check if the path is blocked by the obstacle, if block, re-random
    # path = getPath(closest_point, next_point)
    # blocked_path, unblocked_path = isBlockedPath(path)
    unblocked_path = getUnblockedPath(getPath(closest_point, next_point))
    if len(unblocked_path) <= 1: # edge case where the whole path except the first point is blocked
        continue
    next_point = unblocked_path[-1] # re-specify the next_point based on the last point of the path

    # drawLine(closest_point, next_point)
    drawLine(path) # draw 
    markBinMap(next_point)
    savePoint(cur_pt=next_point, parent_p=closest_point)

    if next_point == end_point:
        drawFinalPath()
        break

if i == ITER_MAX:
    print('could not get path within %d counts', ITER_MAX)

cv.imshow('final im', BGR_IM)
