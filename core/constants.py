import wpilib
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID, 
  Tolerance,  
  Range,
  Value,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  DriftCorrectionConstants, 
  TargetAlignmentConstants,
  PoseSensorConstants,
  PoseSensorConfig,
  RelativePositionControlModuleConstants,
  RelativePositionControlModuleConfig,
  FollowerModuleConstants,
  FollowerModuleConfig,
  BinarySensorConfig
)
from core.classes import (
  Target, 
  TargetType, 
  TargetAlignmentLocation, 
  TargetPosition, 
  TargetPositionType, 
  ElevatorPosition
)

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-filtered.json')
_pathPlannerRobotConfig = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    CHASSIS_LENGTH: units.meters = units.inchesToMeters(36.0)
    CHASSIS_WIDTH: units.meters = units.inchesToMeters(36.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(26.0)
    TRACK_WIDTH: units.meters = units.inchesToMeters(26.0)

    TRANSLATION_SPEED_MAX: units.meters_per_second = 5.74
    ROTATION_SPEED_MAX: units.degrees_per_second = 720.0

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.33

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 22,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 14,
      drivingMotorFreeSpeed = 6784,
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0)
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    PATHPLANNER_ROBOT_CONFIG = _pathPlannerRobotConfig
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    DRIFT_CORRECTION_CONSTANTS = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    TARGET_ALIGNMENT_CONSTANTS = TargetAlignmentConstants(
      translationPID = PID(4.0, 0, 0),
      translationMaxVelocity = 1.4,
      translationMaxAcceleration = 1.0,
      translationTolerance = Tolerance(0.05, 0.1),
      rotationPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 360.0,
      rotationMaxAcceleration = 180.0,
      rotationTolerance = Tolerance(0.5, 1.0),
      rotationHeadingModeOffset = 0,
      rotationTranslationModeOffset = 180.0
    )

  class Elevator:
    LOWER_STAGE_CONFIG = RelativePositionControlModuleConfig("Elevator/LowerStage", 10, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 120,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.9, 1.0),
      motorMotionMaxVelocity = 7000.0,
      motorMotionMaxAcceleration = 14000.0,
      motorMotionAllowedClosedLoopError = 0.5,
      motorSoftLimitForward = 29.0,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.2,
      distancePerRotation = 0.5
    ))
    LOWER_STAGE_FOLLOWER_CONFIG = FollowerModuleConfig("Elevator/LowerStageFollower", 20, 10, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 120
    ))

    UPPER_STAGE_CONFIG = RelativePositionControlModuleConfig("Elevator/UpperStage", 11, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.9, 1.0),
      motorMotionMaxVelocity = 6500.0,
      motorMotionMaxAcceleration = 13000.0,
      motorMotionAllowedClosedLoopError = 0.25,
      motorSoftLimitForward = 28.75,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.1,
      distancePerRotation = 1.0
    ))

    LOWER_STAGE_SOFT_LIMIT_BUFFER: units.inches = 1.5
    LOWER_STAGE_REEF_CORAL_L4_POSITION: units.inches = 15.0
    CAGE_DEEP_CLIMB_UP_SPEED = -1.0
    CAGE_DEEP_CLIMB_DOWN_SPEED = 0.3
    INPUT_LIMIT: units.percent = 0.5

  class Arm:
    ARM_CONFIG = RelativePositionControlModuleConfig("Arm", 12, True, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-1.0, 0.3),
      motorMotionMaxVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionAllowedClosedLoopError = 0.25,
      motorSoftLimitForward = 35.0,
      motorSoftLimitReverse = 1.0,
      motorResetSpeed = 0.4,
      distancePerRotation = 1.0
    ))

    INPUT_LIMIT: units.percent = 1.0

  class Wrist:
    WRIST_CONFIG = RelativePositionControlModuleConfig("Wrist", 13, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 30,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.8, 1.0),
      motorMotionMaxVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionAllowedClosedLoopError = 0.25,
      motorSoftLimitForward = 75.0,
      motorSoftLimitReverse = 0.0,
      motorResetSpeed = 0.2,
      distancePerRotation = 1.0
    ))
    
    INPUT_LIMIT: units.percent = 0.6

  class Hand:
    MOTOR_CAN_ID: int = 14
    MOTOR_CURRENT_LIMIT: int = 40
    MOTOR_INTAKE_CORAL_SPEED: units.percent = 0.4
    MOTOR_INTAKE_CORAL_FROM_GROUND_SPEED: units.percent = 0.8
    MOTOR_SCORE_CORAL_SPEED: units.percent = 0.8
    SCORE_CORAL_TIMEOUT: units.seconds = 0.5
    MOTOR_INTAKE_ALGAE_SPEED: units.percent = 0.9
    MOTOR_SCORE_ALGAE_SPEED: units.percent = 1.0
    SCORE_ALGAE_TIMEOUT: units.seconds = 1.0

class Services:
  class Localization:
    VISION_MAX_TARGET_DISTANCE: units.meters = 4.0
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_GROUND_PLANE_DELTA: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Binary:
    class Hand:
      SENSOR_CONFIG = BinarySensorConfig("Hand", 7)

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = _aprilTagFieldLayout,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "FrontLeft",
        transform = Transform3d(Translation3d(-0.091468, 0.184259, 1.022391), Rotation3d(0.010458, -0.426496, 0.018353)), 
        stream = "http://10.28.81.7:1182/?action=stream", 
        constants = _poseSensorConstants
      ),
      PoseSensorConfig(
        name = "FrontRight",
        transform = Transform3d(Translation3d(-0.117923, -0.219624, 0.955892), Rotation3d(0.021838, 0.534359, 0.026724)), 
        stream = "http://10.28.81.6:1184/?action=stream", 
        constants = _poseSensorConstants
      ),
      PoseSensorConfig(
        name = "RearLeft",
        transform = Transform3d(Translation3d(-0.207278, 0.195631, 0.896791), Rotation3d(-0.072770, 0.329107, 2.877271)),
        stream = "http://10.28.81.7:1184/?action=stream", 
        constants = _poseSensorConstants
      ),
      PoseSensorConfig(
        name = "RearRight",
        transform = Transform3d(Translation3d(-0.205358, -0.172876, 0.895682), Rotation3d(0.106889, 0.326341, -2.855118)), 
        stream = "http://10.28.81.6:1182/?action=stream", 
        constants = _poseSensorConstants
      )
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1184/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "Scylla"

  class Commands:
    AUTO_TARGET_ALIGNMENT_TIMEOUT: units.seconds = 1.5

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, _aprilTagFieldLayout.getTagPose(1)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, _aprilTagFieldLayout.getTagPose(2)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(6).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(6)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(7).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(7)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(8).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(8)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(9).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(9)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(10).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(10)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(11).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, _aprilTagFieldLayout.getTagPose(12)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, _aprilTagFieldLayout.getTagPose(13)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(17).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(17)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(18).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(18)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(19).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(19)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(20).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(20)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(21).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(21)),
          utils.getTargetHash(_aprilTagFieldLayout.getTagPose(22).toPose2d()): Target(TargetType.Reef, _aprilTagFieldLayout.getTagPose(22))
        }
      }

      TARGET_ALIGNMENT_TRANSFORMS: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(27.0), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(19.5), units.inchesToMeters(-6.5), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(19.5), units.inchesToMeters(6.5), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))),
          TargetAlignmentLocation.LeftL4: Transform3d(units.inchesToMeters(22.5), units.inchesToMeters(-6.5), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))),
          TargetAlignmentLocation.RightL4: Transform3d(units.inchesToMeters(22.5), units.inchesToMeters(6.5), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))) 
        },
        TargetType.CoralStation: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(0.0), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(-24.0), 0, Rotation3d(Rotation2d.fromDegrees(-2.0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(24.0), 0, Rotation3d(Rotation2d.fromDegrees(-2.0)))
        }
      }                                                                                                                                           

      TARGET_POSITIONS: dict[TargetPositionType, TargetPosition] = {
        TargetPositionType.ReefCoralL4: TargetPosition(ElevatorPosition(28.5, 28.0), 15.0, 75.0),
        TargetPositionType.ReefCoralL3: TargetPosition(ElevatorPosition(3.0, 27.0), 12.5, 72.0),
        TargetPositionType.ReefCoralL2: TargetPosition(ElevatorPosition(Value.min, 9.5), 7.0, 72.0),
        TargetPositionType.ReefCoralL1: TargetPosition(ElevatorPosition(20.0, Value.min), 23.0, 75.0),
        TargetPositionType.ReefAlgaeL3: TargetPosition(ElevatorPosition(12.0, 28.0), 18.5, 33.0),
        TargetPositionType.ReefAlgaeL2: TargetPosition(ElevatorPosition(7.0, 20.0), 24.0, 39.0),
        TargetPositionType.IntakeCoralDown: TargetPosition(ElevatorPosition(Value.min, 2.5), 27.0, 36.5),
        TargetPositionType.IntakeCoralUp: TargetPosition(ElevatorPosition(Value.min, Value.min), 5.0, 60.0),
        TargetPositionType.IntakeAlgaeDown: TargetPosition(ElevatorPosition(Value.min, 6.0), 28.75, 36.0),
        TargetPositionType.CoralStation: TargetPosition(ElevatorPosition(Value.min, 10.25), Value.min, 32.5),
        TargetPositionType.Barge: TargetPosition(ElevatorPosition(Value.max, Value.max), Value.min, 44.0),
        TargetPositionType.Processor: TargetPosition(ElevatorPosition(Value.min, Value.min), Value.min, Value.min)
      }
