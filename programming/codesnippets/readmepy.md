# FRC Robot Code Documentation

## Overview
This repository contains the robot code for an FRC (FIRST Robotics Competition) robot using WPILib's command-based programming framework in Python. The code is structured to control a robot with the following capabilities:
- Tank/Arcade drive system
- Articulated arm mechanism
- Powered intake system
- Vision processing for alignment

## Project Structure
```
frc_robot/
├── robot.py
├── robotcontainer.py
├── constants.py
├── subsystems/
│   ├── drive_subsystem.py
│   ├── arm_subsystem.py
│   ├── intake_subsystem.py
│   └── vision_subsystem.py
├── commands/
│   ├── autonomous/
│   │   ├── auto_routine.py
│   │   └── auto_drive.py
│   └── teleop/
│       ├── move_arm.py
│       └── intake_command.py
└── utils/
    ├── pid_controller.py
    └── limelight.py
```

## Key Components

### 1. RobotContainer
The `RobotContainer` class serves as the main robot configuration class. It:
- Initializes all subsystems
- Sets up controller button bindings
- Configures default commands
- Provides access to autonomous commands

Key methods:
- `configureButtonBindings()`: Maps controller buttons to commands
- `getAutonomousCommand()`: Returns the selected autonomous routine

### 2. Subsystems

#### DriveSubsystem
Controls the robot's drive train using differential drive.
- Supports both arcade and tank drive
- Manages motor controllers and encoders
- Handles gyro integration for rotation tracking

Key features:
- Encoder-based distance tracking
- Gyro-based heading control
- Motor safety implementations

```python
# Example usage:
drive.arcadeDrive(forward_speed, rotation)
drive.resetEncoders()
current_heading = drive.getHeading()
```

#### ArmSubsystem
Manages the robot's articulated arm mechanism.
- PID-controlled position management
- Encoder feedback for accurate positioning
- Soft limits for safety

Key features:
- Position presets for common operations
- Calibration routines
- Current limiting for motor protection

```python
# Example usage:
arm.setPosition(ArmConstants.kScorePosition)
current_pos = arm.getPosition()
```

#### IntakeSubsystem
Controls the intake mechanism for game piece manipulation.
- Variable speed control
- Game piece detection
- Current monitoring for jamming protection

Key features:
- Automated intake sequences
- Sensor-based game piece detection
- Reverse functionality for ejection

```python
# Example usage:
intake.setSpeed(IntakeConstants.kIntakeSpeed)
has_game_piece = intake.isGamePieceDetected()
```

#### VisionSubsystem
Handles vision processing for target alignment.
- Limelight integration
- Target distance calculation
- Alignment assistance

Key features:
- Real-time target tracking
- Distance estimation
- Pipeline switching

```python
# Example usage:
if vision.hasValidTarget():
    distance = vision.getTargetDistance()
    angle = vision.getTargetAngle()
```

### 3. Commands

#### Autonomous Commands
Pre-programmed sequences for autonomous operation:
- `AutonomousRoutine`: Complete autonomous sequence
- `AutoDrive`: Distance-based driving
- `AutoScore`: Automated scoring sequence

Example autonomous routine:
```python
auto_routine = commands2.SequentialCommandGroup(
    AutoDrive(drive, 2.0, 0.5),
    MoveArmToPosition(arm, ArmConstants.kScorePosition),
    IntakeCommand(intake, -IntakeConstants.kIntakeSpeed)
)
```

#### Teleop Commands
Interactive commands for driver control:
- `DefaultDrive`: Main driving command
- `MoveArmToPosition`: Arm positioning
- `IntakeCommand`: Intake control
- `AlignToTarget`: Vision-assisted alignment

### 4. Constants
Configuration values are organized in separate classes:
- `DriveConstants`: Drive train configuration
- `ArmConstants`: Arm mechanism settings
- `IntakeConstants`: Intake system parameters
- `VisionConstants`: Vision processing configuration

## Setup and Configuration

### Prerequisites
- Python 3.11+
- RobotPy 2024+
- NetworkTables
- OpenCV (for vision processing)

### Installation
1. Clone the repository
2. Install dependencies:
```bash
python -m pip install robotpy[all]
```
3. Deploy to robot:
```bash
python robot.py deploy
```

### Motor Controller Configuration
1. Drive Motors: CAN IDs 1-4
2. Arm Motor: CAN ID 5
3. Intake Motor: CAN ID 6

### Sensor Setup
1. Drive Encoders: DIO ports 0-3
2. Arm Encoder: DIO ports 4-5
3. Intake Sensor: DIO port 6
4. Gyro: SPI port

## Usage

### Driver Controls
- Left Stick: Forward/Backward drive
- Right Stick: Rotation
- A Button: Move arm to score position
- B Button: Run intake
- X Button: Run outtake
- Y Button: Move arm to stow position
- Right Trigger: Vision alignment

### Autonomous Selection
1. Use the SmartDashboard to select autonomous routine
2. Available routines:
   - Score and Drive
   - Two Game Piece Auto
   - Simple Drive

### Dashboard Integration
The code publishes the following data to SmartDashboard:
- Drive encoder values
- Arm position
- Game piece status
- Vision targeting data
- Battery voltage
- Motor currents

## Testing and Simulation

### Unit Tests
Run unit tests with:
```bash
python robot.py test
```

### Simulation
Run robot simulation with:
```bash
python robot.py sim
```

## Contributing
1. Fork the repository
2. Create a feature branch
3. Add your changes
4. Submit a pull request

## Troubleshooting

### Common Issues
1. Motor Not Responding
   - Check CAN ID configuration
   - Verify power connections
   - Check motor controller firmware

2. Sensor Errors
   - Verify wiring connections
   - Check sensor configuration
   - Confirm DIO port assignments

3. Vision Processing Issues
   - Check network connection
   - Verify Limelight configuration
   - Confirm pipeline settings

## Safety Features
- Motor current limiting
- Soft limits on arm movement
- Emergency stop implementation
- Watchdog monitoring
- Power monitoring
- Brownout protection

## Maintenance
- Regularly check encoder calibration
- Update motor controller firmware
- Maintain vision camera focus
- Monitor battery health
- Check CAN bus integrity

Remember to regularly backup your code and maintain a change log for all modifications.