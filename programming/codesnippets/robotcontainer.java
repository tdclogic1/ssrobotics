{
    "RobotContainer": {
        "description": "Main robot container class",
        "code": """
import commands2
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.arm_subsystem import ArmSubsystem
from subsystems.intake_subsystem import IntakeSubsystem

class RobotContainer:
    def __init__(self):
        # Create subsystems
        self.drive = DriveSubsystem()
        self.arm = ArmSubsystem()
        self.intake = IntakeSubsystem()
        
        # Configure button bindings
        self.configureButtonBindings()
        
        # Set default commands
        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.arcadeDrive(
                    -self.controller.getLeftY(),
                    self.controller.getRightX()
                ),
                [self.drive]
            )
        )
    
    def configureButtonBindings(self):
        # Bind controller buttons to commands
        self.aButton.whenPressed(
            MoveArmToPosition(self.arm, ArmConstants.kScorePosition)
        )
        self.bButton.whenPressed(
            IntakeCommand(self.intake, IntakeConstants.kIntakeSpeed)
        )
    
    def getAutonomousCommand(self):
        return AutonomousRoutine(self.drive, self.arm, self.intake)
"""
    },
    
    "DriveSubsystem": {
        "description": "Controls the robot's drive train",
        "code": """
import commands2
import wpilib
from wpilib.drive import DifferentialDrive

class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        
        # Initialize motors
        self.leftMotors = wpilib.MotorControllerGroup(
            wpilib.PWMSparkMax(DriveConstants.kLeftMotor1Port),
            wpilib.PWMSparkMax(DriveConstants.kLeftMotor2Port)
        )
        
        self.rightMotors = wpilib.MotorControllerGroup(
            wpilib.PWMSparkMax(DriveConstants.kRightMotor1Port),
            wpilib.PWMSparkMax(DriveConstants.kRightMotor2Port)
        )
        
        self.drive = DifferentialDrive(self.leftMotors, self.rightMotors)
        
        # Initialize encoders
        self.leftEncoder = wpilib.Encoder(
            DriveConstants.kLeftEncoderPorts[0],
            DriveConstants.kLeftEncoderPorts[1]
        )
        self.rightEncoder = wpilib.Encoder(
            DriveConstants.kRightEncoderPorts[0],
            DriveConstants.kRightEncoderPorts[1]
        )
        
        # Initialize gyro
        self.gyro = wpilib.ADXRS450_Gyro()
        
    def arcadeDrive(self, xSpeed: float, rotation: float):
        self.drive.arcadeDrive(xSpeed, rotation)
        
    def resetEncoders(self):
        self.leftEncoder.reset()
        self.rightEncoder.reset()
        
    def getAverageEncoderDistance(self):
        return (
            self.leftEncoder.getDistance() + 
            self.rightEncoder.getDistance()
        ) / 2.0
        
    def zeroHeading(self):
        self.gyro.reset()
"""
    },
    
    "ArmSubsystem": {
        "description": "Controls the robot's arm mechanism",
        "code": """
import commands2
import wpilib
import wpimath.controller

class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        
        # Initialize motor and encoder
        self.motor = wpilib.PWMSparkMax(ArmConstants.kMotorPort)
        self.encoder = wpilib.Encoder(
            ArmConstants.kEncoderPorts[0],
            ArmConstants.kEncoderPorts[1]
        )
        
        # Initialize PID controller
        self.controller = wpimath.controller.PIDController(
            ArmConstants.kP, ArmConstants.kI, ArmConstants.kD
        )
        self.controller.setTolerance(ArmConstants.kPositionTolerance)
        
    def setPosition(self, angle: float):
        self.controller.setSetpoint(angle)
        
    def getPosition(self) -> float:
        return self.encoder.getDistance()
        
    def periodic(self):
        # Calculate PID output and apply to motor
        output = self.controller.calculate(
            self.getPosition()
        )
        self.motor.set(output)
"""
    },
    
    "IntakeSubsystem": {
        "description": "Controls the intake mechanism",
        "code": """
import commands2
import wpilib

class IntakeSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        
        self.motor = wpilib.PWMSparkMax(IntakeConstants.kMotorPort)
        self.sensor = wpilib.DigitalInput(IntakeConstants.kSensorPort)
        
    def setSpeed(self, speed: float):
        self.motor.set(speed)
        
    def stop(self):
        self.motor.set(0)
        
    def isGamePieceDetected(self) -> bool:
        return self.sensor.get()
"""
    },
    
    "VisionSubsystem": {
        "description": "Handles vision processing",
        "code": """
import commands2
from networktables import NetworkTables

class VisionSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        
        self.table = NetworkTables.getTable("limelight")
        
    def getTargetDistance(self) -> float:
        targetOffsetAngle = self.table.getNumber("ty", 0.0)
        
        # Calculate distance using known target height
        angleToTarget = VisionConstants.kCameraAngle + targetOffsetAngle
        return (
            (VisionConstants.kTargetHeight - VisionConstants.kCameraHeight) /
            math.tan(math.radians(angleToTarget))
        )
        
    def getTargetAngle(self) -> float:
        return self.table.getNumber("tx", 0.0)
        
    def hasValidTarget(self) -> bool:
        return self.table.getNumber("tv", 0) == 1
"""
    },
    
    "Commands": {
        "AutonomousRoutine": {
            "description": "Sample autonomous routine",
            "code": """
import commands2

class AutonomousRoutine(commands2.SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, 
                 arm: ArmSubsystem, 
                 intake: IntakeSubsystem):
        super().__init__(
            # Drive forward while lowering arm
            commands2.ParallelCommandGroup(
                AutoDrive(drive, 2.0, 0.5),
                MoveArmToPosition(arm, ArmConstants.kIntakePosition)
            ),
            # Intake game piece
            IntakeCommand(intake, IntakeConstants.kIntakeSpeed).withTimeout(1.0),
            # Raise arm and drive to scoring position
            commands2.ParallelCommandGroup(
                AutoDrive(drive, -2.0, 0.5),
                MoveArmToPosition(arm, ArmConstants.kScorePosition)
            ),
            # Score
            IntakeCommand(intake, -IntakeConstants.kIntakeSpeed).withTimeout(1.0)
        )
"""
        },
        "MoveArmToPosition": {
            "description": "Command to move arm to position",
            "code": """
import commands2

class MoveArmToPosition(commands2.CommandBase):
    def __init__(self, arm: ArmSubsystem, targetPosition: float):
        super().__init__()
        self.arm = arm
        self.targetPosition = targetPosition
        self.addRequirements([self.arm])
        
    def initialize(self):
        self.arm.setPosition(self.targetPosition)
        
    def isFinished(self):
        return self.arm.isAtPosition()
        
    def end(self, interrupted: bool):
        if interrupted:
            self.arm.stop()
"""
        }
    },
    
    "Constants": {
        "description": "Robot constants",
        "code": """
class DriveConstants:
    # Motor ports
    kLeftMotor1Port = 0
    kLeftMotor2Port = 1
    kRightMotor1Port = 2
    kRightMotor2Port = 3
    
    # Encoder ports
    kLeftEncoderPorts = (0, 1)
    kRightEncoderPorts = (2, 3)
    
    # PID constants
    kP = 1.0
    kI = 0.0
    kD = 0.0
    
    # Physical constants
    kWheelDiameter = 6  # inches
    kEncoderResolution = 4096  # ticks per revolution

class ArmConstants:
    kMotorPort = 4
    kEncoderPorts = (4, 5)
    
    # PID constants
    kP = 0.1
    kI = 0.0
    kD = 0.0
    kPositionTolerance = 2.0  # degrees
    
    # Arm positions
    kStowPosition = 0.0
    kIntakePosition = 45.0
    kScorePosition = 90.0

class IntakeConstants:
    kMotorPort = 5
    kSensorPort = 6
    
    # Motor speeds
    kIntakeSpeed = 0.5
    kEjectSpeed = -0.5

class VisionConstants:
    kCameraHeight = 24  # inches
    kCameraAngle = 45  # degrees
    kTargetHeight = 104  # inches
"""
    }
}