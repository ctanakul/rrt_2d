import numpy as np
import cv2 as cv
import math
import random
import rrt_functions as rf
import argparse

BGR_YELLOW = [0, 255, 255]
BGR_RED = [0, 0, 255]
BGR_BLACK = [0, 0, 0]
BGR_BLUE = [255, 0, 0]


def run_rrt(im_dir, resize_tuple):

    BGR_IM = cv.imread(im_dir, cv.IMREAD_COLOR)  # BGR
    if resize_tuple != (-1, -1):
        BGR_IM = cv.resize(BGR_IM, dsize=resize_tuple)  # (width, height)
    BIN_IM = cv.cvtColor(BGR_IM, cv.COLOR_BGR2GRAY)
    # keep track of obstacles and previously drawn lines
    cv.threshold(BIN_IM, 125, 255, cv.THRESH_BINARY, BIN_IM)
    cv.namedWindow('win', cv.WINDOW_NORMAL)

    H, W = BIN_IM.shape
    print('Height - y: ', H, "Width - x: ", W)

    random.seed()  # random generator from current time

    END_POINT = (H - 1, W - 1)
    START_POINT = (0, 0)
    # initialize different end_point and next_point
    # end_point = getAvailableRandomPoint()
    end_point = END_POINT
    rf.markBGRMap(START_POINT, BGR_RED, BGR_IM)  # Red for startpoint
    rf.markBGRMap(end_point, BGR_YELLOW, BGR_IM)  # Yellow for endpoint

    next_point = START_POINT  # init dummy next point
    point_parent_dict = dict()

    # return a path: a list of tuples from start_point to end_point

    # RRT process
    i = 0
    p_num_final_path = -1
    point_parent_dict.update({START_POINT: None})
    while True:
        print('\n\nround : ', i)
        i += 1

        # random a NEW next point (not in point_parent_dict)
        next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
        while (next_point in point_parent_dict):
            next_point = (random.randint(0, H - 1), random.randint(0, W - 1))

        # find the previous closest point
        closest_point, closest_d = rf.getClosestPoint(
            next_point, point_parent_dict)
        
        # get the unblocked path form the previous closest point to a new point
        unblocked_path = rf.getUnblockedPath(
            rf.getPath(closest_point, next_point), BIN_IM)
        # edge case where the whole path except the first point, the closest part, is blocked
        if len(unblocked_path) <= 1:
            rf.markBGRMap(closest_point, BGR_RED, BGR_IM)  # Red for startpoint
            cv.circle(BGR_IM, next_point[::-1], 3, (255, 255, 0), 3)
            cv.circle(BGR_IM, closest_point[::-1], 3, (255, 0, 0), 3)
            print('whole path is blocked except the first point')
            continue

        # re-specify the next_point based on the last point of the path
        next_point = unblocked_path[-1]

        # save the new point in the point_parent_dict and draw a line
        point_parent_dict.update({next_point: closest_point})
        cv.line(BGR_IM, (next_point[1], next_point[0]),
                (closest_point[1], closest_point[0]), (0, 0, 0), lineType=8)

        # If the endpoint can be reached directly, then reach it
        path_to_endpoint = rf.getPath(next_point, end_point)
        if not rf.checkBlockedPath(path_to_endpoint, BIN_IM):
            point_parent_dict.update({end_point: next_point})
            p_num_final_path = rf.drawBackToPoint(point_parent_dict, end_point,
                               START_POINT, tuple(BGR_RED), BGR_IM, 3)
            break

        cv.imshow('win', BGR_IM)
        k = cv.waitKey(10)
        if k == ord('q'):
            break
    print('------------SUMMARY------------')
    print('total random points = ', i)
    print('total points in final path = ', p_num_final_path)
    cv.imshow('win', BGR_IM)
    key = cv.waitKey(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='An RRT path planning educational tool with algorithmic customization capability.')
    parser.add_argument(
        '-i', '--image', default='/home/ctlattez/python_repos/logo_pathplanning/northwestern-n.jpg',
        help='background image directory')
    parser.add_argument('-r', '--resize', nargs=2, type=int, default=[-1, -1])
    args = parser.parse_args()
    resize_tuple = tuple(args.resize)
    if resize_tuple[0] != -1 and (resize_tuple[0] < 0 or resize_tuple[1] < 0):
        raise RuntimeError('resize_tuple[0] must be positive integer')
    run_rrt(args.image, resize_tuple)
