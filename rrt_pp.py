import numpy as np
import cv2 as cv
import math
import random
import rrt_functions as rf


def run_rrt():

    IM_DIR = '/home/ctlattez/python_repos/logo_pathplanning/northwestern-n.jpg'
    # IM_DIR = '/home/ctlattez/python_repos/Path-Planning/spartan-helmet-og.png'

    BGR_IM = cv.imread(IM_DIR, cv.IMREAD_COLOR)  # BGR
    BGR_IM = cv.resize(BGR_IM, dsize=(750, 750))  # (width, height)
    BIN_IM = cv.cvtColor(BGR_IM, cv.COLOR_BGR2GRAY)
    # keep track of obstacles and previously drawn lines
    cv.threshold(BIN_IM, 125, 255, cv.THRESH_BINARY, BIN_IM)
    cv.namedWindow('win', cv.WINDOW_NORMAL)

    # dict : {point: {parents}}
    H, W = BIN_IM.shape
    print('Height - y: ', H, "Width - x: ", W)
    # CLIP_D = float(30)

    random.seed()  # random generator from current time

    def markBinMap(point: tuple, mark_black: bool = True):
        BIN_IM[point[0], point[1]] = 255 * (int)(not mark_black)

    def markBGRMap(point: tuple, bgr_color: list):
        BGR_IM[point[0], point[1]] = bgr_color

    BGR_YELLOW = [0, 255, 255]
    BGR_RED = [0, 0, 255]
    BGR_BLACK = [0, 0, 0]

    END_POINT = (H - 1, W - 1)
    START_POINT = (0, 0)
    # initialize different end_point and next_point
    # end_point = getAvailableRandomPoint()
    end_point = END_POINT
    # markBinMap(end_point)
    markBGRMap(START_POINT, BGR_RED)  # Yellow for endpoint
    markBGRMap(end_point, BGR_RED)  # Yellow for endpoint
    next_point = START_POINT  # init dummy next point
    point_parent_dict = dict()

    # return a path: a list of tuples from start_point to end_point

    # RRT process
    ITER_MAX = 5000
    i = 0
    point_parent_dict.update({START_POINT: None})
    # while i < ITER_MAX:
    while True:

        # if i == 0:
        #     # while next_point != end_point:
        #     #     next_point = getAvailableRandomPoint()
        #     # marked_points[next_point] = None  # None == no parent == starting point
        #     next_point = START_POINT
        #     point_parent_dict.update({next_point: None})

        i += 1

        # get the next point
        # next_point = getAvailableRandomPoint()
        next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
        while (next_point in point_parent_dict):
            next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
        closest_point, closest_d = rf.getClosestPoint(
            next_point, point_parent_dict)

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

        unblocked_path = rf.getUnblockedPath(
            rf.getPath(closest_point, next_point), BIN_IM)
        # edge case where the whole path except the first point, the closest part, is blocked
        if len(unblocked_path) <= 1:
            print('whole path is blocked except the first point')
            continue
        # re-specify the next_point based on the last point of the path
        next_point = unblocked_path[-1]

        point_parent_dict.update({next_point: closest_point})
        cv.line(BGR_IM, (next_point[1], next_point[0]),
                (closest_point[1], closest_point[0]), (0, 0, 0))

        # If the endpoint can be reached directly, then reach it
        path_to_endpoint = rf.getPath(next_point, end_point)
        if not rf.checkBlockedPath(path_to_endpoint, BIN_IM):
            point_parent_dict.update({end_point: next_point})
            rf.drawBackToPoint(point_parent_dict, end_point, START_POINT, tuple(BGR_RED), BGR_IM)
            break
        print('-----------------')
        cv.imshow('win', BGR_IM)
        k = cv.waitKey(10)
        if k == ord('q'):
            break

    if i == ITER_MAX:
        print('could not get path within %d counts', ITER_MAX)
    print('final line: ', i)
    cv.imshow('win', BGR_IM)
    key = cv.waitKey(0)


def main():
    run_rrt()


if __name__ == "__main__":
    main()
