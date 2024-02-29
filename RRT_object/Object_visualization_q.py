import numpy as np
import cv2 as cv
import shapely.geometry
import shapely.affinity
import math
from src.robot_arm import FK, glo2loc, loc2glo
import imageio


class RotatedRect:
    def __init__(self, cx, cy, w, h, angle):
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.angle = angle

    def get_contour(self):
        w = self.w
        h = self.h
        c = shapely.geometry.box(-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
        rc = shapely.affinity.rotate(c, 360 - self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())


# convert rectangle from [center,width,length,angle] to [x1,y1,x2,y2,angle]
def convert_rectangle(rectangle):
    center = (rectangle[0], rectangle[1])
    width = rectangle[2]
    height = rectangle[3]
    angle = rectangle[4]
    rect_pts = (
        center[0] - width // 2, center[1] - height // 2,
        center[0] + width // 2, center[1] + height // 2)
    return rect_pts


# ----------------------------------
# Pohybujuci sa objekt
def object_visualize(center, width, height, angle, obstacle):
    # Define the coordinates of the rectangle
    # center = (300, 300)
    # width = 200
    # height = 100
    # angle = 70
    # interpretacia pre prienik
    r1 = obstacle
    r2 = RotatedRect(center[0], center[1], width, height, angle)

    # prienik 2 objektov - kolizia
    intersection = r1.intersection(r2)
    intersection = np.array(intersection.exterior.coords)
    intersection = intersection.astype(int)

    # cv.rectangle(img, (center[0] - width // 2, center[1] - height // 2),(center[0] + width // 2, center[1] + height // 2), color, thickness)

    # Calculate the rotation matrix
    rotation_matrix = cv.getRotationMatrix2D(center, angle, 1)

    # Get the coordinates of the rectangle's corners
    rect_pts = np.array([
        [center[0] - width // 2, center[1] - height // 2],
        [center[0] + width // 2, center[1] - height // 2],
        [center[0] + width // 2, center[1] + height // 2],
        [center[0] - width // 2, center[1] + height // 2]
    ], dtype=np.float32)

    # Rotate the rectangle's corners
    rotated_pts = cv.transform(np.array([rect_pts]), rotation_matrix).squeeze()
    rotated_pts = rotated_pts.astype(int)

    return rotated_pts, intersection

def robot_visualization(pos_q):

    robot_start_point = (525, (230+85))

    # Define the length of the line
    l1 = 360
    l2 = 260

    # Define the angle of the line in degrees
    q1 = math.radians(pos_q[0])
    q2 = math.radians(pos_q[1])

    # Calculate the endpoint of the line
    l1_end_point = (int(robot_start_point[0] + l1 * np.cos(q1)),
                 int(robot_start_point[1] + l1 * np.sin(q1)))

    l2_end_point = (int(l1_end_point[0] + l2 * np.cos(q1+q2)),
                 int(l1_end_point[1] + l2 * np.sin(q1+q2)))
    # #Draw the line on the image
    # color = (255, 255, 255)  # black color
    # thickness = 2
    # image = np.ones((700, 1000, 3), dtype=np.uint8)
    # cv.line(image,robot_start_point, l1_end_point, color, thickness)
    # cv.line(image,l1_end_point, l2_end_point, color, thickness)
    #
    # cv.imshow("Line", image)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    arm_points = (robot_start_point,l1_end_point,l2_end_point)
    return (arm_points)

def path_sampling(path):
    path_xy = []
    path_q = []
    for j in range(len(path)-1):
        start = path[j]
        goal = path[j + 1]

        start_xy = FK(start)
        start_xy = loc2glo(start_xy)
        path_xy.append(start_xy)
        path_q.append(start)

        if abs(start[0]-goal[0]) > 10 or abs(start[1]-goal[1]) > 10:

            increment = 3  # number of position checked between start and goal
            range_q1 = abs(start[0] - goal[0])
            range_q2 = abs(start[1] - goal[1])
            d = math.sqrt(range_q1 ** 2 + range_q2 ** 2)  # prepona
            samples = int(d / increment)

            increment_x = range_q1 / samples
            increment_y = range_q2 / samples

            for i in range(1, samples):
                if start[0] < goal[0]:
                    q1 = start[0] + i * increment_x
                else:
                    q1 = start[0] - i * increment_x

                if start[1] < goal[1]:
                    q2 = start[1] + i * increment_y
                else:
                    q2 = start[1] - i * increment_y

                position = (q1,q2)
                path_q.append(position)

                position = FK(position)  # from Q_space to XY_space
                position = loc2glo(position)

                path_xy.append(position)

    path_xy.append(loc2glo(FK(path[-1])))
    path_q.append(path[-1])

    return path_q

robot_visualization((1,1.))

# Create a white image
img = np.full((300, 600, 3), 255, dtype=np.uint8)
# Draw a diagonal blue line with thickness of 5 px
# cv.line(img,(0,0),(20,30),(255,0,0),1)

# ----------------------------------
# prekazka
cv.rectangle(img, (250, 100), (350, 300), (200, 255, 0), thickness=-1)
# interpretacia pre prienik
r1 = RotatedRect(300, 200, 100, 200, 0)

# --------------------------------
# Draw the rotated rectangle
center = (550, 130)
width = 20
height = 100
angle = 0
[rotated_pts, intersection] = object_visualize(center, width, height, angle, r1)
# if len(intersection) != 0:
#     print("EMPTY")
#
# # create GIF
# frames = []

# for i in range (150):
#     if (i<50):
#         x=1
#     elif (i<100):
#         x=-2
#     else:
#         x=3

#     center = (center[0]-3,center[1]+x)
#     angle = angle + 3

#     [rotated_pts, intersection] = object_visualize(center, width, height, angle,r1)
#     # Draw a rectangle
#     cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)

#     # draw intersection between 2 objects
#     cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)

#     cv.imshow("image",img)
#     frames.append(img)
#     cv.waitKey()


#     #reset image
#     img = np.full((300, 600, 3), 255, dtype=np.uint8)
#     cv.rectangle(img, (250, 100), (350, 300), (200, 255, 0), thickness=-1)


# Save GIF
# with imageio.get_writer("moving-object.gif",mode="I") as writer:
#     for frame in frames:
#         print("Adding frame to GIF file")
#         rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#         writer.append_data(rgb_frame)
