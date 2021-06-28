<a name="src"></a>
# src

### Python library for various tasks in EV3 MicroPython

## Use
Import relavent files like this:
```python
from src import color_sensor as colorSensor
```
You may want to include this as a Git submodule, in which case you would use
```python
from lib.src import color_sensor as colorSensor
```
## API Docs

<a name="src.nice_print"></a>
# src.nice\_print

<a name="src.color_sensor"></a>
# src.color\_sensor

<a name="src.color_sensor.getLightReflection"></a>
#### getLightReflection

```python
getLightReflection(currentColorSensor: ColorSensor) -> int
```

Helper function for current reflection

**Arguments**:

- `currentColorSensor` _ColorSensor_ - The current color sensor in use
  

**Returns**:

- `int` - The current reflection

<a name="src.color_sensor.calibrateColorSensor"></a>
#### calibrateColorSensor

```python
calibrateColorSensor(currentEV3: EV3Brick, currentColorSensor: ColorSensor, leftOrRight: str, fileName: str)
```

Calibrates the selected COLOR SENSOR by getting LRI readings from both White and Black areas and stores it in the CSV Data file. See calibrateColorSensors for a full calibration.

**Arguments**:

- `currentEV3` _EV3Brick_ - The current EV3 brick to be used
- `currentColorSensor` _ColorSensor_ - The current color sensor to be used
- `leftOrRight` _str_ - Whether to use the left or right color sensor for this calibration
- `fileName` _str_ - Filename for calibration save

<a name="src.color_sensor.calibrateColorSensors"></a>
#### calibrateColorSensors

```python
calibrateColorSensors(currentEV3: EV3Brick, currentLeftColorSensor: ColorSensor, currentRightColorSensor: ColorSensor, fileName: str)
```

Takes the LRI readings from both COLOR SENSORS, and stores them in a CSV file.

**Arguments**:

- `currentEV3` _EV3Brick_ - The current EV3 brick to be used
- `currentLeftColorSensor` _ColorSensor_ - The current left color sensor to be used
- `currentRightColorSensor` _ColorSensor_ - The current right color sensor to be used
- `fileName` _str_ - Filename for calibration save

<a name="src.color_sensor.getAllColorCalibrationData"></a>
#### getAllColorCalibrationData

```python
getAllColorCalibrationData(calibrationDataFileName: str)
```

Gets all color calibration data

**Arguments**:

- `calibrationDataFileName` _str_ - Filename for calibration save
  

**Returns**:

- `[type]` - Calibration data

<a name="src.color_sensor.getLRIOfColorSensor"></a>
#### getLRIOfColorSensor

```python
getLRIOfColorSensor(calData: [], side: str, color: str)
```

Returns the reading of LRI

**Arguments**:

- `calData` _[]_ - Calibration data to be read
- `side` _str_ - `right` or `left`
- `color` _str_ - `black` or `white`
  

**Returns**:

- `[int]` - The stored LRI

<a name="src.navigation"></a>
# src.navigation

<a name="src.navigation.turnWithGyroSensorRampingDown"></a>
#### turnWithGyroSensorRampingDown

```python
turnWithGyroSensorRampingDown(currentRobot: DriveBase, targetTurnAngle: float, currentGyro: GyroSensor, currentRightMotor: Motor, currentLeftMotor: Motor, initialTurnSpeed: float)
```

Function to turn with the Gyro while ramping down

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `targetTurnAngle` _float_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `currentRightMotor` _Motor_ - [description]
- `currentLeftMotor` _Motor_ - [description]
- `initialTurnSpeed` _float_ - [description]

<a name="src.navigation.turnWithPIDControlOfGyroSensor"></a>
#### turnWithPIDControlOfGyroSensor

```python
turnWithPIDControlOfGyroSensor(currentRobot: DriveBase, targetTurnAngle: float, currentGyro: GyroSensor, currentRightMotor: Motor, currentLeftMotor: Motor)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `targetTurnAngle` _float_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `currentRightMotor` _Motor_ - [description]
- `currentLeftMotor` _Motor_ - [description]

<a name="src.navigation.simpleTurnWithGyro"></a>
#### simpleTurnWithGyro

```python
simpleTurnWithGyro(currentRobot: DriveBase, currentGyro: GyroSensor, turnAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `turnAngle` _int_ - [description]
- `turnTime` _int_ - [description]
- `turnRadius` _float_ - [description]
- `isClockwise` _Boolean_ - [description]

<a name="src.pid_line_follower"></a>
# src.pid\_line\_follower

<a name="src.pid_line_follower.followLinePIDForCertainMotorRotatingDegree"></a>
#### followLinePIDForCertainMotorRotatingDegree

```python
followLinePIDForCertainMotorRotatingDegree(currentRobot: DriveBase, currentLeftMotor: Motor, lineFollowingColorSensor: ColorSensor, followLeftEdge: Boolean, targetLineFollowingLRI: int, currentDriveSpeed: int, targetMotorAngle: int, kpValue: float, kiValue: float, kdValue: float)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentLeftMotor` _Motor_ - [description]
- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `targetLineFollowingLRI` _int_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetMotorAngle` _int_ - [description]
- `kpValue` _float_ - [description]
- `kiValue` _float_ - [description]
- `kdValue` _float_ - [description]

<a name="src.pid_line_follower.followLinePIDSimplified"></a>
#### followLinePIDSimplified

```python
followLinePIDSimplified(currentRobot: DriveBase, currentLeftMotor: Motor, lineFollowingColorSensor: ColorSensor, checkpointColorSensoR: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetLRI: int, targetCheckPointLRI: int, currentKpValue: float, currentKiValue: float, currentKdValue: float, targetMotorAngle: int, ignoreCheckPoint: Boolean)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentLeftMotor` _Motor_ - [description]
- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `checkpointColorSensoR` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetLRI` _int_ - [description]
- `targetCheckPointLRI` _int_ - [description]
- `currentKpValue` _float_ - [description]
- `currentKiValue` _float_ - [description]
- `currentKdValue` _float_ - [description]
- `targetMotorAngle` _int_ - [description]
- `ignoreCheckPoint` _Boolean_ - [description]

<a name="src.pid_line_follower.followLinePID"></a>
#### followLinePID

```python
followLinePID(currentRobot: DriveBase, currentLeftMotor: Motor, currentGyro: GyroSensor, lineFollowingColorSensor: ColorSensor, checkpointColorSensoR: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetMotorAngle: int, targetLRI: int, targetCheckPointLRI: int)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentLeftMotor` _Motor_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `checkpointColorSensoR` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetMotorAngle` _int_ - [description]
- `targetLRI` _int_ - [description]
- `targetCheckPointLRI` _int_ - [description]

<a name="src.pid_line_follower.followLinePIDForPathDOS"></a>
#### followLinePIDForPathDOS

```python
followLinePIDForPathDOS(currentRobot: DriveBase, currentLeftMotor: Motor, currentGyro: GyroSensor, lineFollowingColorSensor: ColorSensor, checkpointColorSensoR: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetMotorAngle: int, targetMotorAngleAfterSharpTurn: int)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentLeftMotor` _Motor_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `checkpointColorSensoR` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetMotorAngle` _int_ - [description]
- `targetMotorAngleAfterSharpTurn` _int_ - [description]

<a name="src.pid_line_follower.driveUntilCertainGyroAngle"></a>
#### driveUntilCertainGyroAngle

```python
driveUntilCertainGyroAngle(currentRobot: DriveBase, currentGyro: GyroSensor, driveSpeed: int, driveSteering: int, turnAngle: float, isClockwise: Boolean)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `driveSpeed` _int_ - [description]
- `driveSteering` _int_ - [description]
- `turnAngle` _float_ - [description]
- `isClockwise` _Boolean_ - [description]

<a name="src.pid_line_follower.turnWithGyroSensorGuidanceAndColorSensor"></a>
#### turnWithGyroSensorGuidanceAndColorSensor

```python
turnWithGyroSensorGuidanceAndColorSensor(currentRobot: DriveBase, currentGyro: GyroSensor, turnAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean, targetColorSensor: ColorSensor, targetLRI: float)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentGyro` _GyroSensor_ - [description]
- `turnAngle` _int_ - [description]
- `turnTime` _int_ - [description]
- `turnRadius` _float_ - [description]
- `isClockwise` _Boolean_ - [description]
- `targetColorSensor` _ColorSensor_ - [description]
- `targetLRI` _float_ - [description]

<a name="src.pid_line_follower.driveForCertainMotorAngle"></a>
#### driveForCertainMotorAngle

```python
driveForCertainMotorAngle(currentRobot: DriveBase, currentMotor: Motor, robotSpeed: float, robotSteering: float, targetMotorAngle: float)
```

[summary]

**Arguments**:

- `currentRobot` _DriveBase_ - [description]
- `currentMotor` _Motor_ - [description]
- `robotSpeed` _float_ - [description]
- `robotSteering` _float_ - [description]
- `targetMotorAngle` _float_ - [description]

