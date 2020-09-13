import numpy as np
import cv2 as cv
import math
import random
import rrt_functions as rf

IM_DIR = '/home/ctlattez/python_repos/logo_pathplanning/northwestern-n.jpg'

BGR_IM = cv.imread(IM_DIR, cv.IMREAD_COLOR)  # BGR
BGR_IM = cv.resize(BGR_IM, dsize = (180, 195))
BIN_IM = cv.cvtColor(BGR_IM, cv.COLOR_BGR2GRAY)
# keep track of obstacles and previously drawn lines
cv.threshold(BIN_IM, 125, 255, cv.THRESH_BINARY, BIN_IM)
cv.namedWindow('win', cv.WINDOW_NORMAL)

# dict : {point: {parents}}
H, W = BIN_IM.shape
print('Height - y: ', H, "Width - x: ", W)
# CLIP_D = float(30)

random.seed()  # random generator from current time


# def isObstacle(point: tuple) -> bool:
#     return BIN_IM[point[0], point[1]] == 0

# give a tuple of a new point. It is not obstacle or added point by getting verified in bin_map.


# def getAvailableRandomPoint() -> tuple:

#     while (1):  # should have error check inside or random point limit
#         # random the point within the image
#         y = random.randint(0, H)
#         x = random.randint(0, W)

#         # check the availability
#         if not isObstacle((y, x)):
#             return (y, x)


def markBinMap(point: tuple, mark_black: bool = True):
    BIN_IM[point[0], point[1]] = 255 * (int)(not mark_black)


def markBGRMap(point: tuple, bgr_color: list):
    BGR_IM[point[0], point[1], 0] = bgr_color[0]
    BGR_IM[point[0], point[1], 1] = bgr_color[1]
    BGR_IM[point[0], point[1], 2] = bgr_color[2]


BGR_YELLOW = [0, 255, 255]
BGR_RED = [0, 0, 255]
BGR_BLACK = [0, 0, 0]


END_POINT = (H - 10, W - 10)
START_POINT = (10, 10)
# initialize different end_point and next_point
# end_point = getAvailableRandomPoint()
end_point = END_POINT
markBinMap(end_point)
markBGRMap(START_POINT, BGR_RED)  # Yellow for endpoint
markBGRMap(end_point, BGR_RED)  # Yellow for endpoint
next_point = end_point  # init dummy next point
point_parent_dict = dict()


# return a path: a list of tuples from start_point to end_point

# RRT process
ITER_MAX = 500
i = 0
# while i < ITER_MAX:
while True:

    if i == 0:
        # while next_point != end_point:
        #     next_point = getAvailableRandomPoint()
        # marked_points[next_point] = None  # None == no parent == starting point
        next_point = START_POINT
        point_parent_dict.update({next_point: None})

    i += 1

    # get the next point
    # next_point = getAvailableRandomPoint()
    next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
    while (next_point in point_parent_dict):
        next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
    closest_point, closest_d = rf.getClosestPoint(next_point, point_parent_dict)

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

    # closest_point = (10, 10)
    # next_point = (10, 50)

    unblocked_path = rf.getUnblockedPath(rf.getPath(closest_point, next_point), BIN_IM)
    if len(unblocked_path) <= 1:  # edge case where the whole path except the first point, the closest part, is blocked
        print('whole path is blocked except the first point')
        continue
    # re-specify the next_point based on the last point of the path
    next_point = unblocked_path[-1]

    # drawLine(closest_point, next_point)
    # markBinMap(next_point) 
    # rf.savePoint(cur_pt=next_point, parent_p=closest_point)
    point_parent_dict.update({next_point : closest_point})
    # rf.drawLine(path, BGR_IM)  # draw
    cv.line(BGR_IM, (next_point[1], next_point[0]), (closest_point[1], closest_point[0]) , (0, 0, 0))

    # if next_point == end_point:
    #     cv.line(BGR_IM, next_point, closest_point, (0, 0, 255))
    #     # drawFinalPath()
    #     break

    cv.imshow('win', BGR_IM)
    cv.waitKey(0)
    

if i == ITER_MAX:
    print('could not get path within %d counts', ITER_MAX)

cv.imshow('win', BGR_IM)
cv.waitKey(0)
