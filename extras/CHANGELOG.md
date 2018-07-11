# Open Bionics - FingerLib.h


# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) 
and this project adheres to [Semantic Versioning](http://semver.org/).

## [2.1.0] - Jul 11th, 2018


### Added 
- Added CircleBuffer functionality

### Changed 
- Updated library keywords and properties
- Fixed compatibility with Uno, Mega & Chestnut boards
- Updated PID controller and added ramp functionality
- Non-blocking delay (NB_DELAY) and Non-blocking timer (NB_TIMER) have been updated to use common parent class


## [2.0.6] - Aug 23rd, 2017

### Changed 
- Changed formatting of library_reference.md

## [2.0.5] - Aug 14th, 2017

### Added 
- Added missing semicolon to avr_FingerTimer.cpp


## [2.0.4] - Aug 10th, 2017

### Added 
- The ability to prevent the motor control from being called via an interrupt (Servo.h compatible)

### Changed
- The hardware timers now check the validity of the function pointers before calling them
- Pins in library examples are now correct

## [2.0.3] - June 6th, 2017

### Added 
- Compatibility with the Arduino UNO (uses timer1)


## [2.0.2] - April 10th, 2017

### Added 
- All examples will now throw a compilation error if the boards have not been selected correctly

### Changed
- All examples now have the correct pin assignments for the Chestnut board variant
- Increased the minimum movement speed to prevent the motors heating up when in stall


## [2.0.1] - Feb 7th, 2017

### Added
- MotorTest.ino will now discard an EOL char if it is a newline (\n) or a carriage return (\r)
- Created changelog

### Changed
- Changed the number of fingers in all of the examples for the Almond PCB to 5, instead of 4
- Increased the minimum motor speed for the Almond PCB to reduce humming on the thumb motor

### Removed
- Removed all 'finger[].reachedPos()' references from HandDemo example as it was not performing reliably



## [2.0.0] - Jan 12th, 2017

### Added
- Commented to pretty much everything
- Optional PID controller (enabled by default)
- Optional force controller (only available on the Chestnut PCB)


### Changed
- Formatting & layout (cleaned up the files)
- Updated license
- Updated library reference

### Removed
- timer0 period modification on the Atmega2560, as it breaks delay() (however, this does mean that the motor now hums)


TODO: Complete CHANGELOG

## [1.0.0] - Jan 11th, 2016

### Added
- Created FingerLib, for Almond board
- Created README


