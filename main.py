#This script was pulled from this GitHub repo: https://github.com/GINNOV/Scout-Controller
#Make sure to check out the README before attempting to run

# ROS libraries
import rospy
from geometry_msgs.msg import Twist
from rospy.msg import AnyMsg

from modules.jpgTranscoder import JPGFrame

from markers import *
from kalman import *
from planner import *

# open CV
import cv2
from cv_bridge import CvBridge

# system libraries
import pygame
import numpy as np
import math
import time
import threading
import os
import sys
import argparse

# Constants
ROS_HOST_ADDRESS = "10.42.0.1:11311"
LOCALHOST_ADDRESS = "127.0.0.1"
WINDOW_X, WINDOW_Y = 1966 / 3 , 2138 / 3
FORWARD_SPEED = 0.2
MAX_FORWARD_SPEED = 1.0
MIN_FORWARD_SPEED = 0.1

# Global variables
bridge = CvBridge()
latest_frame = None
cmd_vel_pub = None
joystick_left_x, joystick_left_y = 0, 0
joystick_right_x, joystick_right_y = 0, 0
forward_speed = FORWARD_SPEED
args = None


def parse_arguments():
    global args
    parser = argparse.ArgumentParser(description="Scout Access Controller")
    parser.add_argument("--windowX", type=int, default=1080, help="window width")
    parser.add_argument("--windowY", type=int, default=720, help="window height")
    parser.add_argument(
        "--host", default=ROS_HOST_ADDRESS, help="ROS endpoint such as IP_ADDRESS:PORT"
    )
    parser.add_argument(
        "--localhost", default=LOCALHOST_ADDRESS, help="localhost address"
    )
    parser.add_argument(
        "--control",
        default="keyboard",
        choices=["keyboard", "joystick"],
        help="control scheme",
    )
    parser.add_argument("--verbose", action="store_true", help="verbose output")
    parser.add_argument(
        "--topics", action="store_true", help="print all available ROS topics"
    )

    args = parser.parse_args()
    print(f"Setting ROS MASTER to: http://{args.host}")
    os.environ["ROS_MASTER_URI"] = "http://" + args.host


def color_frame_callback(msg):
    global latest_frame
    try:
        # Parse the incoming message to extract the frame
        frame = JPGFrame.parse_frame(msg)
        # print(f"Received frame: type={frame.type}, length={len(frame.data)} bytes")

        # Process the frame data to obtain the image
        img = JPGFrame.process_frame(frame.data)
        if img is not None:
            # print(f"Successfully decoded image. Shape: {img.shape}")

            # Calculate scaling factor to fit the image to the window
            scale_x = args.windowX / img.shape[1]
            scale_y = args.windowY / img.shape[0]
            scale = min(scale_x, scale_y)

            # Calculate new dimensions
            new_width = int(img.shape[1] * scale)
            new_height = int(img.shape[0] * scale)

            # Resize the image
            resized_image = cv2.resize(
                img, (new_width, new_height), interpolation=cv2.INTER_AREA
            )


            # Convert the resized image to RGB
            latest_frame = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)


            # Create a pygame surface from the frame
            # latest_frame = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))

            if args.verbose:
                print(f"Original frame: {img.shape}, Resized to: {resized_image.shape}")
        else:
            print("Failed to decode image")

    except Exception as e:
        print(f"Error in callback: {str(e)}")


def init_ros():
    """Initialize ROS node and publishers/subscribers"""
    global cmd_vel_pub

    try:
        rospy.init_node("SPIONE_NODE", anonymous=True)
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # rospy.Subscriber("/CoreNode/grey_img", Image, grey_frame_callback)
        sub = rospy.Subscriber(
            "/CoreNode/jpg", AnyMsg, color_frame_callback, queue_size=10
        )
        print(f"Subscribed to topic: {sub.name}")
        print(f"Number of publishers: {sub.get_num_connections()}")
    except Exception as e:
        print(f"Error initializing ROS: {str(e)}")


def send_robot_command(linear_x, linear_y, angular_z):
    """Send movement command to the robot"""
    twist = Twist()
    twist.linear.x = linear_x * 0.2  # strafe left/right
    twist.linear.y = linear_y * forward_speed  # move forward/backward
    twist.angular.z = angular_z * 2.9  # rotate left/right
    cmd_vel_pub.publish(twist)


def handle_input():
    global joystick_left_x, joystick_left_y, joystick_right_x, joystick_right_y, forward_speed

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                save_screenshot()
            elif event.key == pygame.K_h:
                scout_go_home()
            elif event.key == pygame.K_9:
                turn_on_light(0)
            elif event.key == pygame.K_0:
                turn_on_light(1)

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        return False

    joystick_left_x = keys[pygame.K_d] - keys[pygame.K_a]
    joystick_left_y = (
        keys[pygame.K_w] - keys[pygame.K_s]
    )  # Inverted for correct direction
    joystick_right_x = keys[pygame.K_e] - keys[pygame.K_q]

    if keys[pygame.K_p] and forward_speed < MAX_FORWARD_SPEED:
        forward_speed += 0.1
    if keys[pygame.K_o] and forward_speed > MIN_FORWARD_SPEED:
        forward_speed -= 0.1

    # Ensure joystick values are within -1 to 1 range
    joystick_left_x = max(-1, min(1, joystick_left_x))
    joystick_left_y = max(-1, min(1, joystick_left_y))
    joystick_right_x = max(-1, min(1, joystick_right_x))

    send_robot_command(joystick_right_x, joystick_left_y, joystick_left_x)
    return True



def save_screenshot():
    """Save the current frame as a screenshot"""
    if latest_frame:
        filename = f"scout-{time.strftime('%Y-%m-%d-%H-%M-%S')}.jpg"
        pygame.image.save(latest_frame, filename)
        print(f"Screenshot saved: {filename}")


def scout_go_home():
    """Command the Scout to return to its home position"""
    # Implement the go home functionality
    print("Scout returning home")


def turn_on_light(light_id):
    """Turn on the specified light"""
    # Implement the light control functionality
    print(f"Turning on light {light_id}")

LEFT_MARGIN = 300

# Increase window size
WINDOW_X += LEFT_MARGIN
screen = pygame.display.set_mode((WINDOW_X, WINDOW_Y))

# Load and resize map to fit the remaining width
map_image = pygame.image.load('gc.png').convert_alpha()
map_image = pygame.transform.scale(map_image, (WINDOW_X - LEFT_MARGIN, WINDOW_Y))

robot_image = pygame.image.load('robot.jpg').convert_alpha()
robot_image = pygame.transform.scale(robot_image, (40, 40))

# Map transformation config
PIXELS_PER_METER = 220
MAP_ORIGIN_X = 185  
MAP_ORIGIN_Y = WINDOW_Y - 140

def real_to_pixel(x, y):
    """
    Convert coords from map space to pixel space on the pygame screen
    """
    px = MAP_ORIGIN_X + x * PIXELS_PER_METER
    py = MAP_ORIGIN_Y - y * PIXELS_PER_METER
    return int(px), int(py)

def draw_robot_on_map(screen, robot_img, x, y, theta):
    """
    Draw the robot icon on the screen given the position of the robot in map space
    I used ChatGPT to help with pygame syntax
    """
    theta += np.pi
    # Convert real-world coordinates to pixel coordinates
    pixel_x, pixel_y = real_to_pixel(x, y)
    theta_deg = math.degrees(theta)  # Convert theta to degrees for rotation

    # Draw the robot image
    rotated_robot = pygame.transform.rotate(robot_img, theta_deg + 90)
    rect = rotated_robot.get_rect(center=(pixel_x, pixel_y))
    screen.blit(rotated_robot, rect.topleft)

    # draw direction line
    # Draw the green arrow pointing in the robot's direction
    arrow_length = 30  # Length of the arrow
    arrow_tip_x = pixel_x + arrow_length * math.cos(theta)  # X-coordinate of arrow tip
    arrow_tip_y = pixel_y - arrow_length * math.sin(theta)  # Y-coordinate of arrow tip
    pygame.draw.line(screen, (255, 0, 0), (pixel_x, pixel_y), (arrow_tip_x, arrow_tip_y), 2)
    # Draw the arrowhead
    arrow_head_length = 10  # Length of the arrowhead
    arrow_head_angle = math.radians(30)  # Angle of the arrowhead
    arrow_head_x1 = arrow_tip_x - arrow_head_length * math.cos(theta - arrow_head_angle)
    arrow_head_y1 = arrow_tip_y + arrow_head_length * math.sin(theta - arrow_head_angle)
    arrow_head_x2 = arrow_tip_x - arrow_head_length * math.cos(theta  + arrow_head_angle)
    arrow_head_y2 = arrow_tip_y + arrow_head_length * math.sin(theta  + arrow_head_angle)
    pygame.draw.line(screen, (255, 0, 0), (arrow_tip_x, arrow_tip_y), (arrow_head_x1, arrow_head_y1), 2)
    pygame.draw.line(screen, (255, 0, 0), (arrow_tip_x, arrow_tip_y), (arrow_head_x2, arrow_head_y2), 2)

def draw_HUD(screen, font, robot_estimate):
    """
    Draw overlays of information on the map
    """
    hud_text = f"lX: {joystick_left_x:.2f} lY: {joystick_left_y:.2f} rX: {joystick_right_x:.2f} rY: {joystick_right_y:.2f}"
    speed_text = f"Speed: {forward_speed:.2f}"
    estimate_text = f"EST: (X: {robot_estimate.flatten()[0]:.2f} Y: {robot_estimate.flatten()[1]:.2f} θ: {robot_estimate.flatten()[2]:.2f})"
    text_surface = font.render(hud_text, True, (0, 0, 0))
    speed_surface = font.render(speed_text, True, (0, 0, 0))
    estimate_surface = font.render(estimate_text, True, (0, 0, 0))
    screen.blit(text_surface, (10, 10))
    screen.blit(speed_surface, (10, 50))
    screen.blit(estimate_surface, (WINDOW_X - 320, 175))

def draw_marker(screen, marker_pos, marker_id):
    """
    Draw the position of the marker on the map when detected
    I used ChatGPT to help with pygame syntax
    """
    # Convert marker position to pixel coordinates
    pixel_x, pixel_y = real_to_pixel(marker_pos[0], marker_pos[1])

    # Draw the marker as a square
    marker_size = 20
    # pygame.draw.rect(screen, (0, 0, 255), (pixel_x - marker_size // 2, pixel_y - marker_size // 2, marker_size, marker_size))

    # Draw a triangle (arrow) pointing in the direction of the marker
    arrow_length = 20  # Length of the arrow
    arrow_width = 20   # Width of the arrow

    # Calculate the tip of the arrow
    tip_x = pixel_x + arrow_length * math.cos(marker_pos[2])
    tip_y = pixel_y - arrow_length * math.sin(marker_pos[2])

    # Calculate the base corners of the arrow
    base_left_x = pixel_x - arrow_width * math.sin(marker_pos[2])
    base_left_y = pixel_y - arrow_width * math.cos(marker_pos[2])

    base_right_x = pixel_x + arrow_width * math.sin(marker_pos[2])
    base_right_y = pixel_y + arrow_width * math.cos(marker_pos[2])

    # Draw the arrow
    pygame.draw.polygon(screen, (0, 0, 255), [(tip_x, tip_y), (base_left_x, base_left_y), (base_right_x, base_right_y)])

    # Draw the marker ID text on top of the box and arrow
    font = pygame.font.Font(None, 30)
    text_surface = font.render(str(marker_id), True, (0, 0, 0))
    text_rect = text_surface.get_rect(center=(pixel_x, pixel_y - marker_size // 2))  # Position text above the box
    screen.blit(text_surface, text_rect.topleft)

def draw_current_target(screen, target):
    """
    Draw the current target waypoint on the map
    """
    if target is not None:
        target_name, target_pos = target
        pixel_x, pixel_y = real_to_pixel(target_pos[0], target_pos[1])
        pygame.draw.circle(screen, (0, 0, 255), (pixel_x, pixel_y), 20)
    else:
        print("")

def robot_to_world_pos(marker_pos, marker_position, marker_rotation):
    """
    Find the position of the robot w.r.t the world frame given the position and rotation of the marker w.r.t
    the camera. 
    Used kinematics lecture slides 
    """
    robot_pos = np.zeros(3)

    R = marker_rotation
    tvec = marker_position

    # Marker pose in world
    xm, ym, theta_m = marker_pos
    
    # Tranform components to robot frame to marker
    tx = tvec[0]
    ty = tvec[1]
    tz = tvec[2]
    
    # In robot frame x is forward, y is left. In marker frame z is forward, x is right
    x_robot_m = tz
    y_robot_m = -tx

    # Convert to world frame
    x_robot_w = xm + np.cos(theta_m) * x_robot_m - np.sin(theta_m) * y_robot_m
    y_robot_w = ym + np.sin(theta_m) * x_robot_m + np.cos(theta_m) * y_robot_m


    # Get euler angles from rotation matrix (roll, pitch, yaw)
    # we only need y_rot which is yaw of the robot (y facing up in marker frame)
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x_rot = np.arctan2(R[2, 1], R[2, 2])
        y_rot = np.arctan2(-R[2, 0], sy)
        z_rot = np.arctan2(R[1, 0], R[0, 0])
    else:
        x_rot = np.arctan2(-R[1, 2], R[1, 1])
        y_rot = np.arctan2(-R[2, 0], sy)
        z_rot = 0
    # print(f"roll: {x_rot}, pitch: {y_rot}, yaw: {z_rot}")
    
    theta_robot_w = (theta_m) + y_rot # calculate the rotation in world frame
    theta_robot_w = (theta_robot_w + np.pi) % (2 * np.pi) - np.pi # normalize theta
    robot_pos[0] = x_robot_w
    robot_pos[1] = y_robot_w
    robot_pos[2] = theta_robot_w
    return robot_pos

# Threading for control loop
# This code allows for velocity commands to be sent asyncronously to the motors 
# This way the motor commands are not blocked by the main loop 
command_lock = threading.Lock()
shared_v = 0.0
shared_w = 0.0

def control_loop():
    """
    Constantly send velocity commands determined by the planner
    """
    global shared_v, shared_w, running
    while True:
        with command_lock:
            v = shared_v
            w = shared_w
        send_robot_command(0, v, w)
        print(v,w)
        time.sleep(0.001)

def main_loop():
    global shared_v, shared_w
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.Font(None, 36)

    EKF = ExtendedKalmanFilter() # EKF filter implemented as described in localization lecture

    markerIds = None
    marker_position = None
    marker_rotation = None
    
    # Wypoints and path
    WAYPOINTS = {
        "F": (0.0, 1.5),
        "A": (1.0, 2.5),
        "B": (2.0, 1.5),
        "C": (2.0, 0.0),
        "D": (1.0, 0.0),
        "E": (0.0, 0.0)
    }
    PATH = ["E", "F", "A", "B", "C", "E"]
    # PATH = ["A", "B", "C", "E"]

    # Create a Planner 
    planner = Planner(WAYPOINTS, PATH)

    # Marker ID, marker_x, marker_y, marker_theta
    markers = [
        [0,0.0, 1.75, -np.pi / 2], # pointing down 
        [1,1, 2.75,  -np.pi / 2],
        [2,2.25, 2.0, (2 * np.pi / 3)],
        [3, 2.25, 1.5, np.pi / 2], # pointing up 
        [4, 2.25, -0.25, np.pi / 2],
        [5, 1.0, -0.25, 0],
        [6, -0.25, 0.0, 0],
        [7, -0.25, 0.0, 0],
        [8, 2.25, 1.0, np.pi / 2],
    ]

    threading.Thread(target=control_loop, daemon=True).start() # Start sending velocity commands  

    while running and not rospy.is_shutdown():
        running = handle_input()
        screen.fill((255, 255, 255))
        screen.blit(map_image, (0, 0))

        # path planner
        rsim_x, rsim_y, rsim_theta = EKF.state.flatten()[0], EKF.state.flatten()[1], EKF.state.flatten()[2]
        v, w = planner.update((rsim_x, rsim_y, rsim_theta))  # Update the planner with the current pose

        u_v = joystick_left_y / 50 + (-1 * v / 50)  # 50 and 5 are scaling, moorebot turns slower than it translates
        u_w = joystick_left_x / 5 + w / 5
        
        with command_lock: # send commands to control_loop() thread
            shared_v = v
            shared_w = w

        # kalman filter estimation

        # propagation step
        robot_estimate = EKF.predict((u_v, u_w)) 
        
        # measurement step
        if markerIds is not None and marker_position is not None and marker_rotation is not None: # if there is a measurement
            for i in range(len(markerIds)): # for each marker
                marker_id = markerIds[i][0]
                # print(camera_orientation)
                if marker_id in [0,1,2,3,4,5,6,7,8]: # if is a valid marker
                    marker_pos = markers[marker_id][1:] # get the marker position [x, y, theta]
                    draw_marker(screen, marker_pos, marker_id)

                    robot_pos_measurement = robot_to_world_pos(marker_pos, marker_position, marker_rotation) # get the robot position in world space from the marker position
                    # update the EKF with the measurement
                    EKF.update(robot_pos_measurement)
        
        # get new estimated position and draw robot 
        rsim_x, rsim_y, rsim_theta = EKF.state.flatten()[0], EKF.state.flatten()[1], EKF.state.flatten()[2]
        draw_robot_on_map(screen, robot_image, rsim_x, rsim_y, rsim_theta)

        # overlay of camera feed in the corner
        if latest_frame is not None:
            # detect markers
            markerIds, marker_position, marker_rotation = detect_markers(latest_frame)

            frame = pygame.surfarray.make_surface(latest_frame.swapaxes(0, 1))
            # put image in the corner
            small_frame = pygame.transform.scale(frame, (frame.get_width() // 4, frame.get_height() // 4))
            frame_x = WINDOW_X - frame.get_width() // 4 - 10
            frame_y = 10
            screen.blit(small_frame, (frame_x, frame_y)) # place in the corner

        # HUD info
        draw_HUD(screen, font, robot_estimate)

        target = planner.get_current_target()
        draw_current_target(screen, target)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()



if __name__ == "__main__":
    parse_arguments()
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((WINDOW_X, WINDOW_Y))
    pygame.display.set_caption("Moorebot Final 🏎️")

    try:
        init_ros()
        main_loop()
    except rospy.ROSInterruptException:
        pass
