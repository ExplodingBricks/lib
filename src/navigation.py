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
from myblocks import PIDLineFollower
import time, math

# pylint: enable=import-error


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
        currentTurnSpeed = initialTurnSpeed * math.cos(math.radians(adjustedAngle))

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
        else:  # turn counter-clockwise, and the outer wheel (which is the right wheel) needs to turn slower for tighter turn. Reason unknown.
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
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        elapsed_time = time.time() - start_time

        # Spin turn
        if (
            currentTurnAngle <= targetTurnAngle
        ):  # turn closewise, and the outer wheel (which is the left wheel) needs to turn faster for tighter turn. Reason Unknown.
            currentLeftMotor.run(PIDValue)
            currentRightMotor.run(-PIDValue / 2)
        else:  # turn counter-clockwise, and the outer wheel (which is the right wheel) needs to turn slower for tighter turn. Reason unknown.
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
