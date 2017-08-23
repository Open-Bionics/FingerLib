# FingerLib.h - Finger Control Library Reference


## Initialisation

### attach()

#### Description
Attach pins to a finger using position control and force control, and allow the direction to be inverted

#### Syntax
Finger.attach(dir0, dir1, posSns)

Finger.attach(dir0, dir1, posSns, inv)
						
Finger.attach(dir0, dir1, posSns, forceSns, inv)

#### Parameters
dir1, dir1: the motor direction pins

posSns: the analog motor position pin

inv: boolean. true will invert motor position and direction. false will not invert the motors

#### Returns
byte: returns the finger number


### detach()

#### Description
Deactivate the finger

#### Syntax
Finger.detach()

#### Parameters
none

#### Returns
none


### attached()

#### Description
Return true if the current finger is attached and initialised correctly 

#### Syntax
Finger.attached()

#### Parameters
none

##### Returns
boolean: true if finger is attached correctly. false if finger is not attached


### invertFingerDir()

#### Description
Set the motor to be inverted

#### Syntax
Finger.invertFingerDir()

#### Parameters
none

#### Returns
none


## Limits

### setPosLimits()

#### Description
Set the maximum and minimum position limits

#### Syntax
Finger.setPosLimits(min, max)

#### Parameters
min: the minimum position limit (default 50)

max: the maximum position limit (default 973)

#### Returns
none


### setSpeedLimits()

#### Description
Set the maximum and minimum speed limits

#### Syntax
Finger.setSpeedLimits(min, max)

#### Parameters
min: the minimum speed limit (default 50)

max: the maximum speed limit (default 973)

#### Returns
none


### setForceLimits()

#### Description
Set the maximum and minimum force limits. Note, this is only available on the Chestnut board.

#### Syntax
Finger.setForceLimits(min, max)

#### Parameters
min: the minimum force limit (default 50)

max: the maximum force limit (default 973)

#### Returns
none




## Position

### writePos()

#### Description
Write a target position to the finger

#### Syntax
Finger.writePos(value)

#### Parameters
value: the target position of the finger (0 - 1024)

#### Returns
none


### movePos()

#### Description
Write a change in position to the finger

#### Syntax
Finger.movePos(value)

#### Parameters
value: the change in position of the finger

#### Returns
none


### readPos()

#### Description
Read the current finger position

#### Syntax
Finger.readPos()

#### Parameters
none

#### Returns
int: the current position of the finger (0 - 1024)


### readPosError()

#### Description
Read the error between the current position and the target position

#### Syntax
Finger.readPosError()

#### Parameters
none

#### Returns
int: the error between the current and the target position of the finger (0 - 1024)


### readTargetPos()

#### Description
Read the target position

#### Syntax
Finger.readTargetPos()

#### Parameters
none

#### Returns
int: the target position (0 - 1024)


### reachedPos()

#### Description
Returns true if position reached. i.e. if the position error is less than either a default value (50) or a user specified value

#### Syntax
Finger.reachedPos()
Finger.reachedPos(posErr)

#### Parameters
posErr: set a custom position error threshold

#### Returns
boolean: true if the position error is less than either a default value (50) or a user specified value




## Direction

### writeDir()

#### Description
Write a target direction to the finger

#### Syntax
Finger.writeDir(value)

#### Parameters
value: the target direction of the finger (0 - 1, OPEN - CLOSE)

#### Returns
none


### readDir()

#### Description
Read the current finger direction

#### Syntax
Finger.readDir()

#### Parameters
none

#### Returns
boolean: the current direction of the finger (0 - 1, OPEN - CLOSE)


### open()

#### Description
Open the finger

#### Syntax
Finger.open()

#### Parameters
none

#### Returns
none


### close()

#### Description
Close the finger

#### Syntax
Finger.close()

#### Parameters
none

#### Returns
none


### open_close()

#### Description
Toggle the finger between open and closed

#### Syntax
Finger.open_close()
Finger.open_close(dir)

#### Parameters
dir: set the finger to a direction (0 - 1, OPEN - CLOSE)

#### Returns
none




## Speed

### writeSpeed()

#### Description
Write a target speed to the finger

#### Syntax
Finger.writeSpeed(value)

#### Parameters
value: the target speed of the finger (0 - 255)

#### Returns
none


### readSpeed()

#### Description
Read the current finger speed

#### Syntax
Finger.readSpeed()

#### Parameters
none

#### Returns
byte: the current speed of the finger (0 - 255)


### readTargetSpeed()

#### Description
Read the target finger speed

#### Syntax
Finger.readTargetSpeed()

#### Parameters
none

#### Returns
byte: the target speed of the finger (0 - 255)




## Force

### writeForce()

#### Description
Write a target force to the finger, in a particular direction

#### Syntax
Finger.writeForce(value, dir)

#### Parameters
value: the target force of the finger (0.0N - 60N)

dir: the direction to apply the force (0 - 1, OPEN - CLOSE)

#### Returns
none


### readForce()

#### Description
Read the current finger force

#### Syntax
Finger.readForce()

#### Parameters
none

#### Returns
float: the current force of the finger. If force sense is not enabled, return blank (-1)


### readCurrent()

#### Description
Read the finger force as an ADC value

#### Syntax
Finger.readCurrent()

#### Parameters
none

#### Returns
int: the current force of the finger, as an ADC value (0 - 1024)


### reachedForceLimit()

#### Description
Return true if the force limit is reached (force limit is set by setForceLimits())

#### Syntax
Finger.reachedForceLimit()

#### Parameters
none

#### Returns
boolean: true if the force limit is reached. false if the force limit has not been reached


### convertADCToForce()

#### Description
Convert an ADC value to a force value, using the predefined conversions constants in FingerLib.h

#### Syntax
Finger.convertADCToForce(val)

#### Parameters
none

#### Returns
float: the converted force value, in Newtons


### convertForceToADC()

#### Description
Convert a force value to an ADC, using the predefined conversions constants in FingerLib.h

#### Syntax
Finger.convertForceToADC(val)

#### Parameters
none

#### Returns
float: the converted ADC value (0 - 1024)




## Stop/Start

### stopMotor()

#### Description
Stop the motor and maintain the current position

#### Syntax
Finger.stopMotor()

#### Parameters
none

#### Returns
none


#### disableMotor

#### Description
Disable the motor by setting the speed to 0

#### Syntax
Finger.disableMotor()

#### Parameters
none

#### Returns
none


#### enableMotor

#### Description
Enable the motor

#### Syntax
Finger.enableMotor()

#### Parameters
none

#### Returns
none


#### motorEnable

#### Description
Set the motor to be enabled/disabled

#### Syntax
Finger.motorEnable(val)

#### Parameters
val: boolean. true will enable the motor. false will disable the motor

#### Returns
none


#### enableInterrupt

#### Description
Enable the motor control to be called via an interrupt (enabled by default)

#### Syntax
Finger.enableInterrupt()

#### Parameters
none

#### Returns
none


#### disableInterrupt

#### Description
Disable the motor control being called via an interrupt. The motor control function '_fingerControlCallback()' must be called manually in the main loop in order for the motors to move.

#### Syntax
Finger.disableInterrupt()

#### Parameters
none

#### Returns
none


#### enableForceSense

#### Description
Enable force sensing (only available on the Chestnut board)

#### Syntax
Finger.enableForceSense()

#### Parameters
none

#### Returns
none


#### disableForceSense

#### Description
Disable force sensing

#### Syntax
Finger.disableForceSense()

#### Parameters
none

#### Returns
none


## Print

### printPos()

#### Description
Print the current position of the finger in the format ‘Pos ####’. (optional ‘\n’ after the position)
value)

#### Syntax
Finger.printPos()
Finger.printPos(newLine)

#### Parameters
newLine: boolean. true will print a new line after the value has been printed. false will not print a new line 

#### Returns
none


### printPosError()

#### Description
Print the position error in the format 'Err ####’. (optional ‘\n’ after the position)
value)

#### Syntax
Finger.printPosError()
Finger.printPosError(newLine)

#### Parameters
newLine: boolean. true will print a new line after the value has been printed. false will not print a new line 

#### Returns
none


### printDir()

#### Description
Print the current direction in the format 'Dir ####’. (optional ‘\n’ after the position)
value)

#### Syntax
Finger.printDir()
Finger.printDir(newLine)

#### Parameters
newLine: boolean. true will print a new line after the value has been printed. false will not print a new line 

#### Returns
none


### printReached()

#### Description
Print whether the target position has been reached, in the format 'Reached #’. (optional ‘\n’ after the position)
value)

#### Syntax
Finger.printReached()
Finger.printReached(newLine)

#### Parameters
newLine: boolean. true will print a new line after the value has been printed. false will not print a new line 

#### Returns
none


### printSpeed()

#### Description
Print the current speed in the format 'Speed ####’. (optional ‘\n’ after the position)
value)

#### Syntax
Finger.printSpeed()
Finger.printSpeed(newLine)

#### Parameters
newLine: boolean. true will print a new line after the value has been printed. false will not print a new line 

#### Returns
none


### printDetails()

#### Description
Print the current finger number, position, direction, speed and whether the target position has been reached, followed by a new line character

#### Syntax
Finger.printDetails()

#### Parameters
none

#### Returns
none


### printConfig()

#### Description
Print the current finger number, assigned pins and the limits, followed by a new line character

#### Syntax
Finger.printConfig()

#### Parameters
none

#### Returns
none