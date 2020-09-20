import numpy as np
import cv2 as cv
import math
import random
import rrt_functions as rf
import argparse
import matplotlib.pyplot as plt
import py_helpers as pyh
import statistics as stat

BGR_YELLOW = [0, 255, 255]
BGR_RED = [0, 0, 255]
BGR_BLACK = [0, 0, 0]
BGR_BLUE = [255, 0, 0]


def run_rrt(im_dir: str, resize_tuple: tuple, run_num: int, show_img: bool, show_log: bool):

    BGR_IM = cv.imread(im_dir, cv.IMREAD_COLOR)  # BGR
    if resize_tuple != (-1, -1):
        BGR_IM = cv.resize(BGR_IM, dsize=resize_tuple)  # (width, height)
    BIN_IM = cv.cvtColor(BGR_IM, cv.COLOR_BGR2GRAY)
    # keep track of obstacles and previously drawn lines
    cv.threshold(BIN_IM, 125, 255, cv.THRESH_BINARY, BIN_IM)
    if show_img:
        cv.namedWindow('win', cv.WINDOW_NORMAL)

    H, W = BIN_IM.shape

    random.seed()  # random generator from current time

    END_POINT = (H - 1, W - 1)
    START_POINT = (0, 0)
    # initialize different end_point and next_point
    end_point = END_POINT
    rf.markBGRMap(START_POINT, BGR_RED, BGR_IM)  # Red for startpoint
    rf.markBGRMap(end_point, BGR_YELLOW, BGR_IM)  # Yellow for endpoint

    next_point = START_POINT  # init dummy next point
    point_parent_dict = dict()

    # for plotting
    iter_list = []

    # RRT process
    for run_count in range(run_num):
        point_parent_dict.clear()
        point_parent_dict.update({START_POINT: None})
        iter = 0
        p_num_final_path = -1
        bgr_im = BGR_IM.copy()
        while True:
            iter += 1

            # random a NEW next point (not in point_parent_dict)
            next_point = (random.randint(0, H - 1), random.randint(0, W - 1))
            while (next_point in point_parent_dict):
                next_point = (random.randint(0, H - 1),
                              random.randint(0, W - 1))

            # find the previous closest point
            closest_point, closest_d = rf.getClosestPoint(
                next_point, point_parent_dict)

            # get the unblocked path form the previous closest point to a new point
            unblocked_path = rf.getUnblockedPath(
                rf.getPath(closest_point, next_point), BIN_IM)
            # edge case where the whole path except the first point, the closest part, is blocked
            if len(unblocked_path) <= 1:
                # Red for startpoint
                rf.markBGRMap(closest_point, BGR_RED, bgr_im)
                cv.circle(bgr_im, next_point[::-1], 3, (255, 255, 0), 3)
                cv.circle(bgr_im, closest_point[::-1], 3, (255, 0, 0), 3)
                if show_log:
                    print('whole path is blocked except the first point')
                continue

            # re-specify the next_point based on the last point of the path
            next_point = unblocked_path[-1]

            # save the new point in the point_parent_dict and draw a line
            point_parent_dict.update({next_point: closest_point})
            cv.line(bgr_im, (next_point[1], next_point[0]),
                    (closest_point[1], closest_point[0]), (0, 0, 0), lineType=8)

            # If the endpoint can be reached directly, then reach it
            path_to_endpoint = rf.getPath(next_point, end_point)
            if not rf.checkBlockedPath(path_to_endpoint, BIN_IM):
                point_parent_dict.update({end_point: next_point})
                p_num_final_path = rf.drawBackToPoint(point_parent_dict, end_point,
                                                      START_POINT, tuple(BGR_RED), bgr_im, 3)
                break
            if show_img:
                cv.imshow('win', bgr_im)
                k = cv.waitKey(10)
                if k == ord('q'):
                    break
        if show_log:
            print('------------SUMMARY------------')
            print('run: ', run_count)
            print('total random points = ', iter)
            print('total points in final path = ', p_num_final_path)
        if show_img:
            cv.imshow('win', bgr_im)
            key = cv.waitKey(0)
        iter_list.append(iter)

    # Data summary
    print('Average iterations per run: ', round(stat.mean(iter_list)))
    print('SD: ', stat.pstdev(iter_list))
    print('Max: ', max(iter_list))
    print('Min: ', min(iter_list))
    plt.plot(range(1, len(iter_list) + 1, 1), iter_list, 'ro')
    plt.ylabel('Total iterations')
    plt.xlabel('Run')
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='An RRT path planning educational tool with algorithmic customization capability.')
    parser.add_argument(
        '-i', '--image', default='/home/ctlattez/python_repos/logo_pathplanning/northwestern-n.jpg',
        help='background image directory')
    parser.add_argument('-r', '--resize', nargs=2, type=int, default=[-1, -1])
    parser.add_argument('-n', '--run_num', type=int, default=1)
    parser.add_argument('--no-imshow', type=pyh.str2bool,
                        nargs='?', const=True, default=False)
    parser.add_argument('--show-log', type=pyh.str2bool,
                        nargs='?', const=True, default=False)
    args = parser.parse_args()
    resize_tuple = tuple(args.resize)
    if resize_tuple[0] != -1 and (resize_tuple[0] < 0 or resize_tuple[1] < 0):
        raise RuntimeError('resize_tuple[0] must be positive integer')
    run_rrt(args.image, resize_tuple, args.run_num, not args.no_imshow, args.show_log)
