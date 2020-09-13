import math


def getClosestPoint(point: tuple, points_list: list):
    min_d = float('inf')
    min_d_point = tuple()

    if len(points_list) == 0:
        raise RuntimeError('points_list is empty, can\'t be iterated.')

    for prev_point in points_list:
        d_y = point[0] - prev_point[0]
        d_x = point[1] - prev_point[1]
        d_norm = math.sqrt(d_y*d_y + d_x*d_x)
        if d_norm < min_d:
            min_d = d_norm
            min_d_point = prev_point
    return min_d_point, min_d


def testGetClosestPointList():
    points_list = [(1, 2), (4, 5), (9, 10)]
    min_d_point, min_d = getClosestPoint((3, 2), points_list) #2, sqrt(10), further
    if min_d_point == (1,2):
        print("min_d_point test pass")
    else:
        raise RuntimeError('min_d_point is not correct')

    if min_d == 2:
        print("min_d test pass")
    else:
        raise RuntimeError('min_d is not correct')

if __name__ == "__main__":
    testGetClosestPointList()
