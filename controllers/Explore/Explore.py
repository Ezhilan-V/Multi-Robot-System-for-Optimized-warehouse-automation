from controller import Robot, DistanceSensor, Motor, Node, Supervisor
import cv2
import numpy as np
import random
import matplotlib.pyplot as plt
import requests
import sys
import time
import random
import os
from numba import njit
import argparse
from PIL import Image
import pygame
from pygame.locals import KEYDOWN, K_q
from sklearn.preprocessing import MinMaxScaler

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28
BLOCK_LENGTH = 0.25 * 24500

# create the Robot instance.
robot = Robot()  # Assumes this controller is assigned to a Supervisor node
# initialize devices
ps = []
URL = "http://127.0.0.1:4500/"
DefName = robot.getName()
try:
    response = requests.get(url=URL + "getposition")
    response.raise_for_status()  # Raise an error for bad status codes
    all_robots = response.json()
    print(all_robots[0])
    for i in range(len(all_robots)):
        if all_robots[i]["name"] == DefName:
            robot_position = all_robots[i]["position"]
except requests.exceptions.RequestException as e:
    print(f"Error fetching initial positions: {e}")
    robot_position = [0.0, 0.0, 0.0]
print(robot_position)
psNames = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
camera = robot.getDevice("camera")
accelerometer = robot.getDevice("accelerometer")
camera.enable(4 * TIME_STEP)
accelerometer.enable(4 * TIME_STEP)
# print(dir(accelerometer))
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
name = str(random.random())


def update_info(name, radius, speed):
    PARAMS = {"name": name, "radius": radius, "speed": speed}
    r = requests.get(url=URL + "update", params=PARAMS)
    print(r.json())


def retrieve_info():
    r = requests.get(url=URL + "retrieve")
    print(r.json())


def find_object(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))
    ret, thresh = cv2.threshold(mask, 127, 255, 1)
    contours, h = cv2.findContours(thresh, 1, 2)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) > 15:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(image, center, radius, (255, 0, 0), 4)
            return (True, radius)

    return (False, 0)


def get_heading_and_distance(current_pos, target_pos):
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    heading = np.arctan2(dy, dx)  # Angle to target in radians
    distance = np.sqrt(dx**2 + dy**2)  # Euclidean distance to target
    return heading, distance


target_position = [float(robot_position[0]) + 0.2, float(robot_position[0]) + 0.2]
HEADING_THRESHOLD = 0.9
kp = 0.1


def set_motor_speeds(left_speed, right_speed):
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)


def moveForward():
    set_motor_speeds(MAX_SPEED, MAX_SPEED)
    robot.step(int(BLOCK_LENGTH / (MAX_SPEED / 2)))
    set_motor_speeds(0, 0)


def moveBackward():
    set_motor_speeds(-MAX_SPEED, -MAX_SPEED)
    robot.step(int(BLOCK_LENGTH / (MAX_SPEED / 2)))
    set_motor_speeds(0, 0)


TURN = 1.040088


def turnLeft():
    set_motor_speeds(-MAX_SPEED, MAX_SPEED)
    robot.step(int(1000 * np.pi * TURN / 2))
    set_motor_speeds(0, 0)  # Adjust the 0.082 value based on the ePuck's wheel distance


def turnRight():
    set_motor_speeds(MAX_SPEED, -MAX_SPEED)
    robot.step(int(1000 * np.pi * TURN / 2))
    set_motor_speeds(0, 0)

    # nav = requests.get(url=URL + "nav")
    # print(nav)

    image = camera.getImageArray()
    if image:
        my_img = np.array(image, dtype=np.uint8)
        cv2.imshow(name, my_img)
        message, radius = find_object(my_img)
        if message:
            update_info(name, radius, accelerometer.getValues())
        cv2.waitKey(1)
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    response = requests.get(url=URL + "getposition")
    robot_position = response.json()[DefName]
    # write actuators inputs
    current_position = [float(robot_position[0]), float(robot_position[1])]
    current_angle = float(robot_position[2])  # Assuming this is in radians
    # Get current position and angle
    current_position = [float(robot_position[0]), float(robot_position[1])]
    current_angle = float(robot_position[2])  # Assuming this is in radians
    # Calculate heading and distance to the target
    desired_heading, distance_to_target = get_heading_and_distance(
        current_position, target_position
    )
    # Normalize an angle to be within 0 to 2π
    print(DefName, distance_to_target)
    # Define constants for navigation
    TURN_SPEED = 0.5 * MAX_SPEED  # Speed for turning
    FORWARD_SPEED = 0.5 * MAX_SPEED  # Speed for moving forward
    TURN_TOLERANCE = 0.5  # Radians tolerance for turning
    DISTANCE_TOLERANCE = 0.1  # Meters tolerance for stopping

    # Helper functions
    def normalize_angle(angle):
        return angle % (2 * np.pi)

    def calculate_angle_difference(target_angle, current_angle):
        # Normalize angles
        target_angle = normalize_angle(target_angle)
        current_angle = normalize_angle(current_angle)

        # Calculate the shortest angle difference
        angle_diff = target_angle - current_angle
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        return angle_diff

    def set_motor_speeds(left_speed, right_speed):
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)

    # Normalize current angle and desired heading
    current_angle = normalize_angle(current_angle)
    desired_heading = normalize_angle(desired_heading)

    # Calculate heading difference
    heading_difference = calculate_angle_difference(desired_heading, current_angle)
    if abs(heading_difference) > TURN_TOLERANCE:
        # Determine turn direction
        turn_direction = np.sign(heading_difference)
        set_motor_speeds(-turn_direction * TURN_SPEED, turn_direction * TURN_SPEED)
    elif distance_to_target > DISTANCE_TOLERANCE:
        # Move straight towards the target
        set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)
    else:
        set_motor_speeds(0, 0)


def navigate(path):
    for i in path:
        if i == "Right":
            turnRight()
        elif i == "Forward":
            moveForward()
        elif i == "Backward":
            moveBackward()
        elif i == "Left":
            turnLeft()


while robot.step(TIME_STEP) != -1:
    # nav = requests.get(url=URL + "explore")
    # navObj = nav.json()
    # navigate(navObj["path"])
    moveForward()
    turnRight()
    turnRight()
    
    moveForward()
    
    moveForward()

