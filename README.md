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

<a name="src.color_sensor"></a>
# src.color\_sensor

<a name="src.color_sensor.get_light_reflection"></a>
#### get\_light\_reflection

```python
get_light_reflection(currentColorSensor: ColorSensor) -> int
```

Helper function for current reflection

**Arguments**:

- `currentColorSensor` _ColorSensor_ - [description]
  

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

<a name="src.color_sensor.getWhiteLRIOfLeftColorSensor"></a>
#### getWhiteLRIOfLeftColorSensor

```python
getWhiteLRIOfLeftColorSensor(calData: [])
```

Returns the reading of LRI by the left color sensor in the White Area

**Arguments**:

- `calData` _[type]_ - [description]
  

**Returns**:

  White LRI

<a name="src.color_sensor.getBlackLRIOfLeftColorSensor"></a>
#### getBlackLRIOfLeftColorSensor

```python
getBlackLRIOfLeftColorSensor(calData: [])
```

Returns the reading of LRI by the left color sensor in the Black Area

**Arguments**:

- `calData` _[type]_ - [description]
  

**Returns**:

  Black LRI

<a name="src.color_sensor.getWhiteLRIOfRightColorSensor"></a>
#### getWhiteLRIOfRightColorSensor

```python
getWhiteLRIOfRightColorSensor(calData: [])
```

Returns the reading of LRI by the right color sensor in the White Area

**Arguments**:

- `calData` _[type]_ - [description]
  

**Returns**:

  White LRI

<a name="src.color_sensor.getBlackLRIOfRightColorSensor"></a>
#### getBlackLRIOfRightColorSensor

```python
getBlackLRIOfRightColorSensor(calData: [])
```

Returns the reading of LRI by the right color sensor in the Black Area

**Arguments**:

- `calData` _[type]_ - [description]
  

**Returns**:

  Black LRI

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

