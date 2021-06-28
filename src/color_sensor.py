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
from pybricks.media.ev3dev import SoundFile, ImageFile, Font

# pylint: enable=import-error


def getLightReflection(currentColorSensor: ColorSensor) -> int:
    """Helper function for current reflection

    Args:
        currentColorSensor (ColorSensor): The current color sensor in use

    Returns:
        int: The current reflection
    """
    return currentColorSensor.reflection()


def calibrateColorSensor(
    currentEV3: EV3Brick,
    currentColorSensor: ColorSensor,
    leftOrRight: str,
    fileName: str,
):
    """Calibrates the selected COLOR SENSOR by getting LRI readings from both White and Black areas and stores it in the CSV Data file. See calibrateColorSensors for a full calibration.

    Args:
        currentEV3 (EV3Brick): The current EV3 brick to be used
        currentColorSensor (ColorSensor): The current color sensor to be used
        leftOrRight (str): Whether to use the left or right color sensor for this calibration
        fileName (str): Filename for calibration save
    """
    # Gives text prompt to put the selected COLOR SENSOR over a White AREA.
    big_font = Font(size=20, bold=True)
    printInstructions(currentEV3, big_font, leftOrRight, "sensor over WHITE")

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

    printInstructions(currentEV3, big_font, leftOrRight, "sensor over BLACK")

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
    with open(fileName, "a") as colorSensorCalibrationDataFile:

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


def printInstructions(currentEV3, big_font, leftOrRight, arg3):
    currentEV3.screen.clear()
    currentEV3.screen.set_font(big_font)
    currentEV3.screen.draw_text(0, 30, "Place " + leftOrRight + " color")
    currentEV3.screen.draw_text(0, 50, arg3)
    currentEV3.screen.draw_text(0, 70, "Press any button..")


def calibrateColorSensors(
    currentEV3: EV3Brick,
    currentLeftColorSensor: ColorSensor,
    currentRightColorSensor: ColorSensor,
    fileName: str,
):
    """Takes the LRI readings from both COLOR SENSORS, and stores them in a CSV file.

    Args:
        currentEV3 (EV3Brick): The current EV3 brick to be used
        currentLeftColorSensor (ColorSensor): The current left color sensor to be used
        currentRightColorSensor (ColorSensor): The current right color sensor to be used
        fileName (str): Filename for calibration save
    """
    # Erases previous content and makes the file blank
    with open(fileName, "w"):
        pass

    # Takes the LRI readings from over the WHITE and BLACK areas by the Right COLOR SENSOR
    calibrateColorSensor(currentEV3, currentRightColorSensor, "right", fileName)
    wait(1500)

    # Takes the LRI readings from over the WHITE and BLACK areas by the Left COLOR SENSOR
    calibrateColorSensor(currentEV3, currentLeftColorSensor, "left", fileName)
    currentEV3.screen.draw_text(0, 40, "Crabs were successful")


def getAllColorCalibrationData(calibrationDataFileName: str):
    """Gets all color calibration data

    Args:
        calibrationDataFileName (str): Filename for calibration save

    Returns:
        [type]: Calibration data
    """
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


def getLRIOfColorSensor(calData: [], side: str, color: str):
    """Returns the reading of LRI

    Args:
        calData ([]): Calibration data to be read
        side (str): `right` or `left`
        color (str): `black` or `white`

    Returns:
        [int]: The stored LRI
    """
    LRI = 90
    if color == "black":
        pos = 2
    elif color == "white":
        pos = 1
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if side in loweredCurrentColorSensorPosition:
            LRI = int(currentCalTuple[pos])
            break

    return LRI
