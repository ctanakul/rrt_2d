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
    b = y1 - m * x1

    long_x = abs(m) < 1
    step = 1  # init
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


def testGetClosestPointList():
    print("-------testGetClosestPointList-------")
    points_list = [(1, 2), (4, 5), (9, 10)]
    min_d_point, min_d = getClosestPoint(
        (3, 2), points_list)  # 2, sqrt(10), further
    if min_d_point == (1, 2):
        print("min_d_point test pass")
    else:
        raise RuntimeError('min_d_point is not correct')

    if min_d == 2:
        print("min_d test pass")
    else:
        raise RuntimeError('min_d is not correct')
    print()


def testGetPath():
    print("-------testGetPath-------")
    st_p1 = (0, 0)
    e_p1 = (4, 3)
    path1 = getPath(st_p1, e_p1)

    if path1 == [(0, 0), (1, 1), (2, 2), (3, 2), (4, 3)]:
        print("path1 test pass")
    else:
        raise RuntimeError("path1 test false")

    st_p2 = (4, 8)
    e_p2 = (1, 1)
    path2 = getPath(st_p2, e_p2)
    print(path2)
    if path2 == [(4, 8), (4, 7), (3, 6), (3, 5), (2, 4), (2, 3), (1, 2), (1, 1)]:
        print("path2 test pass")
    else:
        raise RuntimeError("path2 test false")
        


if __name__ == "__main__":
    testGetClosestPointList()
    testGetPath()
