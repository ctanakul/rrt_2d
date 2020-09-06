import numpy as np
import cv2 as cv

img_dir = 'northwestern-n.jpg'

bgr_img = cv.imread(img_dir, cv.IMREAD_COLOR)  # BGR
bin_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2GRAY)
# keep track of obstacles and previously drawn lines
cv.threshold(bin_img, 125, 255, cv.THRESH_BINARY, bin_img)
cv.namedWindow('main win', cv.WINDOW_NORMAL)

# dict : {point: {parents}}
iter_max = 500
i = 0
H, W = img.shape

random.seed() # random generator from current time

def isObstacle(tuple: point) -> bool:
    return bin_img[point[0], point[1]] == 0


# give a tuple of a new point. It is not obstacle or added point by getting verified in bin_map. 
def getAvailableRandomPoint() -> tuple:

    while (1):  # should have error check inside or random point limit
        # random the point within the image
        y = random.randint(0, H)
        x = random.randint(0, W)

        # check the availability
        if not isObstacle((x, y)):
            break

# initialize different end_point and next_point
end_point = getAvailableRandomPoint()
next_point = end_point
while (next_point != end_point):
    next_point = getAvailableRandomPoint()


# RRT process
while i < iter_max or next_point != end_point:
    i += 1

    # get the next point
    next_point = getNextPoint()

    closest_point = getClosestPoint(next_point)

    drawLine(next_point, closest_point)

    savePoint(cur_pt=next_point, parent_p=closest_point)

    key = cv.waitKey(20) & 0xFF
    if key == 27:  # ESC
        break

cv.imshow('main win', img)
