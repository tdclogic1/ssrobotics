# FRC Robot Code Documentation

## Overview
This repository contains the robot code for an FRC (FIRST Robotics Competition) robot implemented in Java using WPILib's command-based programming framework. The code is structured to control a robot with the following capabilities:
- Tank/Arcade drive system
- Articulated arm mechanism
- Powered intake system
- Vision processing for alignment

## Project Structure
```
src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── subsystems/
│   ├── DriveSubsystem.java
│   ├── ArmSubsystem.java
│   ├── IntakeSubsystem.java
│   └── VisionSubsystem.java
├── commands/
│   ├── autonomous/
│   │   ├── AutonomousRoutine.java
│   │   └── AutoDrive.java
│   └── teleop/
│       ├── MoveArmCommand.java
│       └── IntakeCommand.java
└── utils/
    ├── PIDConstants.java
    └── LimelightHelpers.java
```

## Key Components

### 1. RobotContainer
The `RobotContainer` class serves as the main robot configuration class. It:
- Initializes all subsystems
- Sets up controller button bindings
- Configures default commands
- Provides access to autonomous commands

```java
public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureButtonBindings();
        
        m_drive.setDefaultCommand(
            new RunCommand(
                () -> m_drive.arcadeDrive(
                    -m_driverController.getLeftY(),
                    m_driverController.getRightX()
                ),
                m_drive
            )
        );
    }
}
```

### 2. Subsystems

#### DriveSubsystem
Controls the robot's drive train using differential drive.
```java
public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }
}
```

#### ArmSubsystem
Manages the robot's articulated arm mechanism using PID control.
```java
public class ArmSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ArmConstants.kMotorPort);
    private final ProfiledPIDController m_pidController;

    public ArmSubsystem() {
        m_pidController = new ProfiledPIDController(
            ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocity,
                ArmConstants.kMaxAcceleration
            )
        );
    }
}
```

#### IntakeSubsystem
Controls the intake mechanism for game piece manipulation.
```java
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(
        IntakeConstants.kMotorPort,
        MotorType.kBrushless
    );
    private final DigitalInput m_sensor = new DigitalInput(IntakeConstants.kSensorPort);

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }
}
```

### 3. Commands

#### Autonomous Commands
Example autonomous routine:
```java
public class AutonomousRoutine extends SequentialCommandGroup {
    public AutonomousRoutine(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
        addCommands(
            new ParallelCommandGroup(
                new AutoDrive(drive, 2.0, 0.5),
                new MoveArmCommand(arm, ArmConstants.kScorePosition)
            ),
            new IntakeCommand(intake, IntakeConstants.kEjectSpeed).withTimeout(1.0)
        );
    }
}
```

### 4. Constants
Configuration values organized in separate classes:
```java
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
        
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class ArmConstants {
        public static final int kMotorPort = 5;
        public static final double kP = 0.1;
        public static final double kMaxVelocity = 1.0;
        public static final double kMaxAcceleration = 0.5;
    }
}
```

## Setup and Configuration

### Prerequisites
- Java Development Kit (JDK) 17 or newer
- Visual Studio Code with WPILib Extension
- FRC Game Tools
- REV Hardware Client (for SPARK MAX)
- Phoenix Tuner (for TalonFX)

### Installation
1. Clone the repository
2. Open in VS Code with WPILib
3. Build the project:
```bash
./gradlew build
```
4. Deploy to robot:
```bash
./gradlew deploy
```

### Motor Controller Setup
1. Configure CAN IDs using Phoenix Tuner/REV Client
2. Update firmware if necessary
3. Configure current limits and ramp rates

## Usage

### Driver Controls
Using XBox Controller:
- Left Stick Y-axis: Forward/Backward drive
- Right Stick X-axis: Rotation
- A Button: Move arm to score position
- B Button: Run intake
- X Button: Run outtake
- Y Button: Move arm to stow position
- Right Trigger: Vision alignment

### Development Tips
1. Use Command Scheduler debug tool in Shuffleboard
2. Implement unit tests using WPILib's testing framework
3. Use Git for version control
4. Follow WPILib's command-based programming best practices

### Building and Testing
```bash
# Build project
./gradlew build

# Run tests
./gradlew test

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

## Simulation
WPILib includes a simulation GUI:
1. Run simulation: `./gradlew simulateJava`
2. Use simulated joysticks
3. Monitor robot state
4. Test autonomous routines

## Troubleshooting

### Common Issues
1. CAN Bus Issues
   - Check Phoenix Tuner/REV Client for device presence
   - Verify CAN wire termination
   - Check device firmware versions

2. Build Problems
   - Clean project: `./gradlew clean`
   - Update WPILib: Use WPILib VS Code menu
   - Check vendordeps versions

3. Code Deploy Issues
   - Verify robot connection
   - Check team number in `build.gradle`
   - Ensure correct radio configuration

## Safety Features
- Implemented safety limits
```java
// Motor current limiting example
public void configureMotors() {
    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        true, 30, 35, 0.1));
}
```

## Vendor Dependencies
Required vendor libraries (add via WPILib VS Code):
- Phoenix (for TalonFX)
- REV (for SPARK MAX)
- NavX (for gyro)
- PhotonVision (for vision processing)

## Contributing
1. Fork repository
2. Create feature branch
3. Follow Java style guide
4. Submit pull request

## Advanced Features

### Path Planning
Using WPILib's trajectory tools:
```java
public Command createAutoPath() {
    return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(2.0, 0.7),
        new SimpleMotorFeedforward(ks, kv, ka),
        kinematics,
        m_drive::getWheelSpeeds,
        leftController,
        rightController,
        m_drive::tankDriveVolts,
        m_drive
    );
}
```

Remember to regularly:
- Back up code
- Update documentation
- Test all systems
- Maintain change log