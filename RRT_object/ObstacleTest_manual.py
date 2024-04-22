import cv2 as cv
import numpy as np
import math
from src.robot_arm import FK,loc2glo,glo2loc,IK
from Object_visualization_q import RotatedRect, object_visualize, convert_rectangle, path_sampling, robot_visualization

q1 = 90
q2 = 180
width = 100
height = 100
angle = 180

def update_position(x):
    global q1, q2
    q1 = cv.getTrackbarPos("q1", "control")
    q2 = cv.getTrackbarPos("q2", "control")
    redraw_image()

def update_object(x):
    global width, height, angle
    width = cv.getTrackbarPos("O-Width", "control")
    height = cv.getTrackbarPos("O-Height", "control")
    angle = cv.getTrackbarPos("O-Angle", "control")
    redraw_image()

# Function to redraw the image with the rectangle at the updated position
def redraw_image():
    global q1, q2, width, height, angle, image
    img = image.copy()
    # draw obstacle
    obstacle = (525, 115, 130, 230, 0)
    obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
    rect_points = convert_rectangle(obstacle)
    rect_color = (200, 255, 0)
    cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)

    pos_q = (q1,q2)
    pos_xy = FK(pos_q)
    pos_xy = loc2glo(pos_xy)
    center = (pos_xy[0], pos_xy[1])
    rotation = 360 - angle - pos_xy[2]
    [rotated_pts, intersection] = object_visualize(center, height, width, rotation, obstacle1)
    # draw intersection between 2 objects
    cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)
    print('Center:',int(center[1]),int(center[0]))
    # Draw a rectangle
    cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    #cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)

    # draw robot arm
    robot_joints = robot_visualization(pos_q)
    color = (0, 0, 0)  # black color
    thickness = 3
    cv.line(img, robot_joints[0], robot_joints[1], color, thickness)
    cv.line(img, robot_joints[1], robot_joints[2], color, thickness)
    if len(intersection) != 0:
        print("Collision !")

    # otocenie obrazka - zrkadlovo y
    img = cv.flip(img, 0)
    cv.imshow("Image", img)



# blank image
image = np.full((745, 1050, 3), 255, dtype=np.uint8)

# Create a window and display the image
cv.namedWindow("Image")
cv.namedWindow("control",cv.WINDOW_NORMAL)
cv.resizeWindow("control", 600,200)
cv.imshow("Image", image)

# Create trackbars for adjusting the position
cv.createTrackbar("q1", "control", 60, 180, update_position)
cv.createTrackbar("q2", "control", 240, 360, update_position)

# Create trackbars for adjusting the size of the obstacle
cv.createTrackbar("O-Angle", "control", angle, 360, update_object)
cv.createTrackbar("O-Width", "control", 100,700, update_object)
cv.createTrackbar("O-Height", "control", 100, 700, update_object)


# Initially draw the rectangle at the default position
redraw_image()

# Wait for key press and close the window
cv.waitKey(0)
cv.destroyAllWindows()