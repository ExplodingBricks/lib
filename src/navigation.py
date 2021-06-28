#!/usr/bin/env pybricks-micropython

# pylint: disable=import-error
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# pylint: enable=import-error
import time
import math


def turnWithGyroSensorRampingDown(
    currentRobot: DriveBase,
    targetTurnAngle: float,
    currentGyro: GyroSensor,
    currentRightMotor: Motor,
    currentLeftMotor: Motor,
    initialTurnSpeed: float,
):
    """Function to turn with the Gyro while ramping down

    Args:
        currentRobot (DriveBase): [description]
        targetTurnAngle (float): [description]
        currentGyro (GyroSensor): [description]
        currentRightMotor (Motor): [description]
        currentLeftMotor (Motor): [description]
        initialTurnSpeed (float): [description]
    """
    currentRobot.stop(Stop.BRAKE)
    startingAngle = currentGyro.angle()
    currentAngle = startingAngle
    currentTurnAngle = currentAngle - startingAngle

    output = "{currentTime}, {turnSpeed}, {angle}"

    start_time = time.time()
    while True:
        # target turn angle reached, get out of the loop
        if abs(currentTurnAngle - targetTurnAngle) < 0.5:
            break

        currentGyroAngle = currentGyro.angle()
        currentTurnAngle = currentGyroAngle - startingAngle

        # Speed ramping down according to cosine curve (from 0 degree to 90 degree, or maxSpeed (initialSpeed * cos(0)), to zero speed (initialSpeed * cos(90)))
        adjustedAngle = (abs(currentTurnAngle) / targetTurnAngle) * 90
        currentTurnSpeed = initialTurnSpeed * \
            math.cos(math.radians(adjustedAngle))

        elapsed_time = (time.time() - start_time) * 1000
        print(
            output.format(
                currentTime=elapsed_time,
                turnSpeed=currentTurnSpeed,
                angle=currentTurnAngle,
            )
        )

        # Spin turn
        if (
            currentTurnAngle <= targetTurnAngle
        ):  # turn closewise, and the outer wheel (which is the left wheel) needs to turn faster for tighter turn. Reason Unknown.
            currentLeftMotor.run(currentTurnSpeed)
            currentRightMotor.run(-currentTurnSpeed)
        # turn counter-clockwise, and the outer wheel (which is the right wheel) needs to turn slower for tighter turn. Reason unknown.
        else:
            currentRightMotor.run(currentTurnSpeed)
            currentLeftMotor.run(-currentTurnSpeed)

    currentRobot.stop(Stop.BRAKE)

    elapsed_time = time.time() - start_time

    print(
        "Robot Stopped. The current Degree of the turn=",
        currentTurnAngle,
        ", elapsed time=",
        elapsed_time,
        ", gyro=",
        currentGyro.angle(),
        ", starting gyro=",
        startingAngle,
    )


def turnWithPIDControlOfGyroSensor(
    currentRobot: DriveBase,
    targetTurnAngle: float,
    currentGyro: GyroSensor,
    currentRightMotor: Motor,
    currentLeftMotor: Motor,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        targetTurnAngle (float): [description]
        currentGyro (GyroSensor): [description]
        currentRightMotor (Motor): [description]
        currentLeftMotor (Motor): [description]
    """
    # Parameters for PID gyro turning (optimized values for 90 degree clockwise turn: Kp=10.1,Ki=0.003, Kd=1.1)
    KP_VALUE = 10.1
    KI_VALUE = 0.003
    KD_VALUE = 1.1
    integral = 0
    currentError = 0
    pastError = 0

    currentRobot.stop(Stop.BRAKE)
    startingAngle = currentGyro.angle()
    currentAngle = startingAngle
    currentTurnAngle = currentAngle - startingAngle

    output = "{currentTime}, {PIDVal}"

    start_time = time.time()
    while True:
        # target turn angle reached, get out of the loop
        if abs(currentTurnAngle - targetTurnAngle) < 0.5:
            break

        currentGyroAngle = currentGyro.angle()
        currentTurnAngle = currentGyroAngle - startingAngle
        pastError = currentError

        currentError = targetTurnAngle - currentTurnAngle
        integral += currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        elapsed_time = time.time() - start_time

        # Spin turn
        if (
            currentTurnAngle <= targetTurnAngle
        ):  # turn closewise, and the outer wheel (which is the left wheel) needs to turn faster for tighter turn. Reason Unknown.
            currentLeftMotor.run(PIDValue)
            currentRightMotor.run(-PIDValue / 2)
        # turn counter-clockwise, and the outer wheel (which is the right wheel) needs to turn slower for tighter turn. Reason unknown.
        else:
            currentRightMotor.run(-PIDValue / 2)
            currentLeftMotor.run(PIDValue)

    currentRobot.stop(Stop.BRAKE)

    elapsed_time = time.time() - start_time

    print(
        "Robot Stopped. The current Degree of the turn=",
        currentTurnAngle,
        ", elapsed time=",
        elapsed_time,
        ", gyro=",
        currentGyro.angle(),
        ", starting gyro=",
        startingAngle,
    )


def simpleTurnWithGyro(
    currentRobot: DriveBase,
    currentGyro: GyroSensor,
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentGyro (GyroSensor): [description]
        turnAngle (int): [description]
        turnTime (int): [description]
        turnRadius (float): [description]
        isClockwise (Boolean): [description]
    """
    currentRobot.stop(Stop.BRAKE)

    if isClockwise:
        steering = turnAngle * (1000 / turnTime)
    else:
        steering = -(turnAngle * (1000 / turnTime))
    speed = 2 * math.pi * (turnRadius) * (turnAngle / 360) * (1000 / turnTime)

    gyroAngleBeforeTurn = currentGyro.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    currentRobot.drive(speed, steering)
    while True:
        currentGyroAngle = currentGyro.angle()
        if isClockwise and currentGyro.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyro.angle() <= gyroAngleAfterTurn:
            break

    currentRobot.stop(Stop.BRAKE)
    print("turn_with_gyro_sensor_guidance: gyro angle=", currentGyro.angle())

def variedSpeedTurnWithGyro(
    currentRobot: DriveBase,
    currentGyro: GyroSensor,
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentGyro (GyroSensor): [description]
        turnAngle (int): [description]
        turnTime (int): [description]
        turnRadius (float): [description]
        isClockwise (Boolean): [description]
    """
    currentRobot.stop(Stop.BRAKE)

    if isClockwise:
        steering = turnAngle * (1000 / turnTime)
    else:
        steering = -(turnAngle * (1000 / turnTime))
    speed = 2 * math.pi * (turnRadius) * (turnAngle / 360) * (1000 / turnTime)

    gyroAngleBeforeTurn = currentGyro.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    currentRobot.drive(speed, steering)
    while True:
        currentGyroAngle = currentGyro.angle()

        if isClockwise and currentGyro.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyro.angle() <= gyroAngleAfterTurn:
            break

    currentRobot.stop(Stop.BRAKE)
    print("turn_with_gyro_sensor_guidance: gyro angle=", currentGyro.angle())


def trueTurn(
    currentRobot: DriveBase,
    currentGyro: GyroSensor,
    targetAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentGyro (GyroSensor): [description]
        targetAngle (int): [description]
        turnTime (int): [description]
        turnRadius (float): [description]
        isClockwise (Boolean): [description]
    """
    halfTarget = 0.5 * targetAngle
    startingGyroAngle = currentGyro.angle()
    currentRobot.stop(Stop.BRAKE)
    moveBool = True

    if isClockwise:
        steering = targetAngle * (1000 / turnTime)
    else:
        steering = -(targetAngle * (1000 / turnTime))
    speed = 2 * math.pi * (turnRadius) * (targetAngle / 360) * (1000 / turnTime)

    gyroAngleBeforeTurn = currentGyro.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + targetAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - targetAngle
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    currentRobot.drive(speed, steering)

    while True:
        currentGyroAngle = currentGyro.angle()
        angleTurnSoFar = currentGyroAngle - startingGyroAngle

        if angleTurnSoFar >= targetAngle:
            break

        if angleTurnSoFar >= halfTarget and moveBool:
            moveBool = False
            steering = 0.5 * steering
            print("Angle turn so far = ", angleTurnSoFar, " steering = ", steering)
            currentRobot.drive(speed, steering)

    currentRobot.stop(Stop.BRAKE)
