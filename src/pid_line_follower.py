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

import math




def followLinePIDForCertainMotorRotatingDegree(
    currentRobot: DriveBase,
    currentLeftMotor: Motor,
    lineFollowingColorSensor: ColorSensor,
    followLeftEdge: Boolean,
    targetLineFollowingLRI: int,
    currentDriveSpeed: int,
    targetMotorAngle: int,
    kpValue: float,
    kiValue: float,
    kdValue: float,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentLeftMotor (Motor): [description]
        lineFollowingColorSensor (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        targetLineFollowingLRI (int): [description]
        currentDriveSpeed (int): [description]
        targetMotorAngle (int): [description]
        kpValue (float): [description]
        kiValue (float): [description]
        kdValue (float): [description]
    """
    currentCheckPointColorSensor = lineFollowingColorSensor
    ignoreCheckPoint = True

    followLinePIDSimplified(
        currentRobot,
        currentLeftMotor,
        lineFollowingColorSensor,
        currentCheckPointColorSensor,
        followLeftEdge,
        currentDriveSpeed,
        targetLineFollowingLRI,
        targetLineFollowingLRI,
        kpValue,
        kiValue,
        kdValue,
        targetMotorAngle,
        ignoreCheckPoint,
    )


def followLinePIDSimplified(
    currentRobot: DriveBase,
    currentLeftMotor: Motor,
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensoR: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetLRI: int,
    targetCheckPointLRI: int,
    currentKpValue: float,
    currentKiValue: float,
    currentKdValue: float,
    targetMotorAngle: int,
    ignoreCheckPoint: Boolean,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentLeftMotor (Motor): [description]
        lineFollowingColorSensor (ColorSensor): [description]
        checkpointColorSensoR (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        currentDriveSpeed (int): [description]
        targetLRI (int): [description]
        targetCheckPointLRI (int): [description]
        currentKpValue (float): [description]
        currentKiValue (float): [description]
        currentKdValue (float): [description]
        targetMotorAngle (int): [description]
        ignoreCheckPoint (Boolean): [description]
    """
    # For detecting the black bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = targetCheckPointLRI
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = targetLRI
    # Parameters for PID line flollowing.
    KP_VALUE = currentKpValue
    KI_VALUE = currentKiValue
    KD_VALUE = currentKdValue
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True

    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    currentRobot.stop(Stop.BRAKE)
    currentLeftMotor.reset_angle(0)
    currentRobot.drive(currentDriveSpeed, 0)

    #
    while True:
        currentLeftMotorAngle = currentLeftMotor.angle()
        if currentLeftMotorAngle >= targetMotorAngle:
            break
        # If the checkpoint color sensor hits a black bar,
        # which means we reached the destination, then we stop the Robot

        if (not ignoreCheckPoint) and (
            checkpointColorSensoR.reflection() <= TARGET_CHECKPOINT_LIGHT_REFLECTION
        ):
            print(
                "PIDLineFollower.follow_Line_PID_simplified: target reached. current left motor angle=",
                currentLeftMotorAngle,
            )
            break

        # (=^._.^=) FAT CAT
        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        # Set the drive base speed and turn rate.
        currentRobot.drive(currentDriveSpeed, PIDValue)

    currentRobot.stop(Stop.BRAKE)


def follow_Line_PID(
    currentRobot: DriveBase,
    currentLeftMotor: Motor,
    currentGyro: GyroSensor,
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensoR: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetMotorAngle: int,
    targetLRI: int,
    targetCheckPointLRI: int,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentLeftMotor (Motor): [description]
        currentGyro (GyroSensor): [description]
        lineFollowingColorSensor (ColorSensor): [description]
        checkpointColorSensoR (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        currentDriveSpeed (int): [description]
        targetMotorAngle (int): [description]
        targetLRI (int): [description]
        targetCheckPointLRI (int): [description]
    """
    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    currentRobot.drive(currentDriveSpeed, 0)

    # For detecting the black bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = targetCheckPointLRI
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = targetLRI
    # Parameters for PID line flollowing.
    KP_VALUE = 1.1
    KI_VALUE = 0.001
    KD_VALUE = 10
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True

    #
    while True:
        currentLeftMotorAngle = currentLeftMotor.angle()
        # If the robot travels beyond the target motor angle and hits a black bar,
        # which means we reached the destination, then we stop the Robot
        if (
            checkpointColorSensoR.reflection() < TARGET_CHECKPOINT_LIGHT_REFLECTION
        ) and (currentLeftMotorAngle > targetMotorAngle):
            print("current left motor angle is ", currentLeftMotorAngle)
            break

        # When the robot has travelled far enough near the target motor angle,
        # we slow down so the robot will brake right on target

        if currentLeftMotorAngle > (targetMotorAngle - 100) and (
            isNotCloseToTargetMotorAngle
        ):
            currentDriveSpeed = currentDriveSpeed / 2
            isNotCloseToTargetMotorAngle = False
            print("Felix the right motor's angle is ", currentLeftMotorAngle)

        # (=^._.^=) FAT CAT
        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        # Set the drive base speed and turn rate.
        currentRobot.drive(currentDriveSpeed, PIDValue)

    currentRobot.stop(Stop.BRAKE)


def followLinePIDForPathDOS(
    currentRobot: DriveBase,
    currentLeftMotor: Motor,
    currentGyro: GyroSensor,
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensoR: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetMotorAngle: int,
    targetMotorAngleAfterSharpTurn: int,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentLeftMotor (Motor): [description]
        currentGyro (GyroSensor): [description]
        lineFollowingColorSensor (ColorSensor): [description]
        checkpointColorSensoR (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        currentDriveSpeed (int): [description]
        targetMotorAngle (int): [description]
        targetMotorAngleAfterSharpTurn (int): [description]
    """
    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    currentRobot.drive(currentDriveSpeed, 0)

    # For detecting the black bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = 15
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = 50
    # Parameters for PID line flollowing.
    KP_VALUE = 1.1
    KI_VALUE = 0.001
    KD_VALUE = 10
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True
    isBeforeSharpTurn = True

    #
    while True:
        currentLeftMotorAngle = currentLeftMotor.angle()
        if (
            checkpointColorSensoR.reflection() < TARGET_CHECKPOINT_LIGHT_REFLECTION
        ) and (currentLeftMotorAngle > targetMotorAngle):
            print(
                "HEY! This is the current angle for the LEFT MoTEr!!!",
                currentLeftMotor.angle(),
                "Yo! We have the color sensor Light Reflection!",
                checkpointColorSensoR.reflection(),
            )
            break

        # When the robot is close to the target we slow down so the robot will brake right on target
        if currentLeftMotorAngle > (targetMotorAngle - 100) and (
            isNotCloseToTargetMotorAngle
        ):
            currentDriveSpeed = currentDriveSpeed / 2
            isNotCloseToTargetMotorAngle = False
        if (currentLeftMotorAngle > targetMotorAngleAfterSharpTurn) and (
            isBeforeSharpTurn
        ):
            currentDriveSpeed = currentDriveSpeed * 3
            isBeforeSharpTurn = False
        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        print(
            "currentError= ",
            currentError,
            " proportionalGain= ",
            currentError * KP_VALUE,
            "integral=",
            integral,
            "derivative=",
            derivative,
            "PIDValue=",
            PIDValue,
            "CurentDriveSped=",
            currentDriveSpeed,
        )
        # Set the drive base speed and turn rate.
        currentRobot.drive(currentDriveSpeed, PIDValue)

    currentRobot.stop(Stop.BRAKE)


def drive_until_certain_gyro_angle(
    currentRobot: DriveBase,
    currentGyro: GyroSensor,
    driveSpeed: int,
    driveSteering: int,
    turnAngle: float,
    isClockwise: Boolean,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentGyro (GyroSensor): [description]
        driveSpeed (int): [description]
        driveSteering (int): [description]
        turnAngle (float): [description]
        isClockwise (Boolean): [description]
    """
    currentRobot.stop(Stop.BRAKE)

    gyroAngleBeforeTurn = currentGyro.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print(
        "drive_until_certain_gyro_angle: speed=", driveSpeed, "steering=", driveSteering
    )
    currentRobot.drive(driveSpeed, driveSteering)
    while True:
        currentGyroAngle = currentGyro.angle()
        if isClockwise and currentGyro.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyro.angle() <= gyroAngleAfterTurn:
            break

    currentRobot.stop(Stop.BRAKE)
    print("drive_until_certain_gyro_angle: gyro angle=", currentGyro.angle())


def turn_with_gyro_sensor_guidance_and_color_sensor(
    currentRobot: DriveBase,
    currentGyro: GyroSensor,
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
    targetColorSensor: ColorSensor,
    targetLRI: float,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentGyro (GyroSensor): [description]
        turnAngle (int): [description]
        turnTime (int): [description]
        turnRadius (float): [description]
        isClockwise (Boolean): [description]
        targetColorSensor (ColorSensor): [description]
        targetLRI (float): [description]
    """
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
    print(
        "speed=",
        speed,
        "steering=",
        steering,
        "rightColorLRI=",
        targetColorSensor.reflection(),
    )
    currentRobot.drive(speed, steering)
    while True:
        currentGyroAngle = currentGyro.angle()
        if targetColorSensor.reflection() <= targetLRI:
            break
        if isClockwise and currentGyro.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyro.angle() <= gyroAngleAfterTurn:
            break
    currentRobot.stop(Stop.BRAKE)


def driveForCertainMotorAngle(
    currentRobot: DriveBase,
    currentMotor: Motor,
    robotSpeed: float,
    robotSteering: float,
    targetMotorAngle: float,
):
    """[summary]

    Args:
        currentRobot (DriveBase): [description]
        currentMotor (Motor): [description]
        robotSpeed (float): [description]
        robotSteering (float): [description]
        targetMotorAngle (float): [description]
    """
    currentMotor.reset_angle(0)
    currentRobot.drive(robotSpeed, robotSteering)

    while True:
        if (
            robotSpeed < 0
        ) and currentMotor.angle() <= targetMotorAngle:  # robot drives backwards and it reaches the target angle
            break
        elif (robotSpeed > 0) and (
            currentMotor.angle() >= targetMotorAngle
        ):  # robot drives forwards and it reaches the target angle
            break

    currentRobot.stop(Stop.BRAKE)
