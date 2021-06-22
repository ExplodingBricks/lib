#!/usr/bin/env pybricks-micropython
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
from pybricks.media.ev3dev import SoundFile, ImageFile, Font


def get_light_reflection(currentColorSensor: ColorSensor) -> int:
    return currentColorSensor.reflection()


# Calibrates the selected COLOR SENSOR by getting LRI readings from both White and Black areas
# and store it in the CSV Data file
def calibrate_color_SENSOR(
    currentEV3: EV3Brick,
    currentColorSensor: ColorSensor,
    leftOrRight: str,
    calFileName: str,
):
    # Gives text prompt to put the selected COLOR SENSOR over a White AREA.
    big_font = Font(size=20, bold=True)
    currentEV3.screen.clear()
    currentEV3.screen.set_font(big_font)
    currentEV3.screen.draw_text(0, 30, "Place " + leftOrRight + " color")
    currentEV3.screen.draw_text(0, 50, "sensor over WHITE")
    currentEV3.screen.draw_text(0, 70, "Press any button..")
    # Waits till any buttons on the brick are pressed
    while True:
        if currentEV3.buttons.pressed():
            currentEV3.light.on(Color.RED)
            break
        wait(500)
    wait(2000)
    # Once the button is pressed, it takes the current LRI reading and stores it as the
    # LRI reading for the white area
    whiteReflection = currentColorSensor.reflection()
    currentEV3.light.off()

    # Gives a text prompt to put the selected COLOR SENSOR over a Black AREA
    currentEV3.screen.clear()
    currentEV3.screen.set_font(big_font)
    currentEV3.screen.draw_text(0, 30, "Place " + leftOrRight + " color")
    currentEV3.screen.draw_text(0, 50, "sensor over BLACK")
    currentEV3.screen.draw_text(0, 70, "Press any button..")

    wait(1500)
    # Waits until a button is pressed on the brick
    while True:
        if currentEV3.buttons.pressed():
            print("butoon pressed whilse ove black area.")
            currentEV3.light.on(Color.RED)
            break
        wait(500)

    wait(2000)

    # Once the button on the brick is pressed, it stores the current LRI reading as the
    # LRI reading for a black area
    blackReflection = currentColorSensor.reflection()

    # Opens the Calibration CSV data file in append mode, and the file will be
    # automatically closed once the writing is complete
    with open(calFileName, "a") as colorSensorCalibrationDataFile:

        # Adds the new LRI readings to the Calibration CSV data file
        colorSensorCalibrationDataFile.write(
            leftOrRight + "," + str(whiteReflection) + "," + str(blackReflection) + "\n"
        )

    # Tells you on the Ev3 brick "CLABRATION SUCCESSFULL!!!!"
    currentEV3.screen.clear()
    currentEV3.screen.draw_text(
        0, 40, "Calibration of " + leftOrRight + " color sensor successful"
    )
    currentEV3.light.off()


# Takes the LRI readings from both COLOR SENSORS, and stores them in a CSV file.☺ ☹
def Calibrate_color_sensor_for_Robot(
    currentEV3: EV3Brick,
    currentLeftColorSensor: ColorSensor,
    currentRightColorSensor: ColorSensor,
    fileName: str,
):

    # Erases previous content and makes the file blank
    with open(fileName, "w") as currCalFileName:
        pass

    # Takes the LRI readings from over the WHITE and BLACK areas by the Right COLOR SENSOR
    calibrate_color_SENSOR(currentEV3, currentRightColorSensor, "right", fileName)
    # Take a step back and wwwaaaaiiiitttt (:
    wait(1500)

    # Takes the LRI readings from over the WHITE and BLACK areas by the Left COLOR SENSOR
    calibrate_color_SENSOR(currentEV3, currentLeftColorSensor, "left", fileName)
    currentEV3.screen.draw_text(0, 40, "Crabs were successful")


# You will be attacked by a beast who has the body of a wolf, the tail of a lion...
# ...and the face of Donald Duck


def get_all_color_calibration_data(calibrationDataFileName: str):

    calibrationData = []

    # Opens Calibration Data File in READ-ONLY mode
    with open(calibrationDataFileName, "r") as calDatafile:
        calibrationDataList = calDatafile.readlines()
        # For each line in the data file...
        for row in calibrationDataList[0:]:
            # ...it extracts the values from the CSV data
            colorSensor, whiteLRI, blackLRI = row.strip().split(",")

            calData_dict = (colorSensor, whiteLRI, blackLRI)

            # Creates a tuple from the csv data and adds it to the data list
            calibrationData.append(calData_dict)

    return calibrationData


# Returns the reading of LRI by the left color sensor in the White Area
def get_white_LRI_of_left_ColorSensor(calData: []):
    whiteLRI = 90
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if "left" in loweredCurrentColorSensorPosition:
            whiteLRI = int(currentCalTuple[1])
            break

    return whiteLRI


# Returns the reading of LRI by the left color sensor in the Black Area
def get_black_LRI_of_left_ColorSensor(calData: []):
    blackLRI = 90
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if "left" in loweredCurrentColorSensorPosition:
            blackLRI = int(currentCalTuple[2])
            break

    return blackLRI


# Returns the reading of LRI by the right color sensor in the White Area
def get_white_LRI_of_right_ColorSensor(calData: []):
    whiteLRI = 90
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if "right" in loweredCurrentColorSensorPosition:
            whiteLRI = int(currentCalTuple[1])
            break

    return whiteLRI


# Returns the reading of LRI by the right color sensor in the Balcke Area
def get_black_LRI_of_right_ColorSensor(calData: []):
    blackLRI = 90
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if "right" in loweredCurrentColorSensorPosition:
            blackLRI = int(currentCalTuple[2])
            break

    return blackLRI


def funny_hellos(currentEV3: EV3Brick, currentColorSensor: ColorSensor):
    # It takes some time for fonts to load from file, so it is best to only
    # load them once at the beginning of the program like this:
    tiny_font = Font(size=6)
    big_font = Font(size=24, bold=True)
    chinese_font = Font(size=24, lang="zh-cn")

    currentEV3.screen.clear()

    # Say hello
    currentEV3.screen.print("Hello!")

    # Say tiny hello
    currentEV3.screen.set_font(tiny_font)
    currentEV3.screen.print("hello")

    # Say big hello
    currentEV3.screen.set_font(big_font)
    currentEV3.screen.print("HELLO")

    # Say Chinese hello
    currentEV3.screen.set_font(chinese_font)
    currentEV3.screen.print("好你")

    # Wait some time to look at the screen
    wait(5000)
