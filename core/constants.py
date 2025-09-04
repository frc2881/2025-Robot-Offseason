from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
import wpilib
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from photonlibpy.photonPoseEstimator import PoseStrategy
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import (
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
  PositionControlModuleConstants,
  PositionControlModuleConfig
)
from core.classes import (
  Target, 
  TargetType, 
  TargetAlignmentLocation, 
  TargetPosition, 
  TargetPositionType, 
  ElevatorPosition
)

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-filtered.json')
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kRobotLength: units.meters = units.inchesToMeters(36.0)
    kRobotWidth: units.meters = units.inchesToMeters(36.0)
    kTrackWidth: units.meters = units.inchesToMeters(26.0)
    kWheelBase: units.meters = units.inchesToMeters(26.0)

    kTranslationSpeedMax: units.meters_per_second = 5.74
    kRotationSpeedMax: units.degrees_per_second = 720.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

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

    kSwerveModuleConfigs: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants)
    )

    kDriveKinematics = SwerveDrive4Kinematics(*(c.translation for c in kSwerveModuleConfigs))

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      translationPID = PID(5.0, 0, 0),
      translationMaxVelocity = 1.0,
      translationMaxAcceleration = 0.5,
      translationTolerance = Tolerance(0.05, 0.1),
      rotationPID = PID(5.0, 0, 0), 
      rotationMaxVelocity = 180.0,
      rotationMaxAcceleration = 90.0,
      rotationTolerance = Tolerance(0.5, 1.0),
      rotationHeadingModeOffset = 0,
      rotationTranslationModeOffset = 180.0
    )

  class Elevator:
    _lowerStageModuleConstants = PositionControlModuleConstants(
      distancePerRotation = 0.5,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 120,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.9, 1.0),
      motorMotionMaxVelocity = 7000.0,
      motorMotionMaxAcceleration = 14000.0,
      motorMotionVelocityFF = 1.0 / 6784,
      motorMotionAllowedClosedLoopError = 0.5,
      motorSoftLimitForward = 29.0,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.2
    )

    kLowerStageConfig = PositionControlModuleConfig("Elevator/LowerStage", 10, None, False, _lowerStageModuleConstants)
    kLowerStageHelperConfig = PositionControlModuleConfig("Elevator/LowerStage2", 20, 10, True, _lowerStageModuleConstants)

    kUpperStageConfig = PositionControlModuleConfig("Elevator/UpperStage", 11, None, False, PositionControlModuleConstants(
      distancePerRotation = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.9, 1.0),
      motorMotionMaxVelocity = 6500.0,
      motorMotionMaxAcceleration = 13000.0,
      motorMotionVelocityFF = 1.0 / 6784,
      motorMotionAllowedClosedLoopError = 0.5,
      motorSoftLimitForward = 28.75,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.1
    ))

    kLowerStageSoftLimitBuffer: units.inches = 1.5
    kLowerStageReefCoralL4Position: units.inches = 15.0
    kCageDeepClimbUpSpeed = -1.0
    kCageDeepClimbDownSpeed = 0.3
    kInputLimit: units.percent = 0.5

  class Arm:
    kArmConfig = PositionControlModuleConfig("Arm", 12, None, True, PositionControlModuleConstants(
      distancePerRotation = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-0.8, 0.6),
      motorMotionMaxVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionVelocityFF = 1.0 / 6784,
      motorMotionAllowedClosedLoopError = 0.5,
      motorSoftLimitForward = 70.75,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.2
    ))

    kCoralStationSafePosition: units.inches = 30.0
    kInputLimit: units.percent = 0.6

  class Wrist:
    kWristConfig = PositionControlModuleConfig("Wrist", 13, None, False, PositionControlModuleConstants(
      distancePerRotation = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0),
      motorOutputRange = Range(-0.8, 0.6),
      motorMotionMaxVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionVelocityFF = 1.0 / 6784,
      motorMotionAllowedClosedLoopError = 0.5,
      motorSoftLimitForward = 1.0,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.2
    ))
    
    kInputLimit: units.percent = 0.6

  class Hand:
    kMotorCANId: int = 14
    kMotorCurrentLimit: int = 40
    kMotorIntakeSpeed: units.percent = 0.5
    kMotorHoldSpeed: units.percent = 0.05
    kMotorReleaseSpeed: units.percent = 1.0
    kReleaseTimeout: units.seconds = 0.5

class Services:
  class Localization:
    kVisionMaxTargetDistance: units.meters = 4.0
    kVisionMaxPoseAmbiguity: units.percent = 0.2
    kRobotPoseMaxGroundPlaneDelta: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1
  
  class Binary:
    class Hand:
      kChannel = 7

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "FrontLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-4.5972), units.inchesToMeters(8.125), units.inchesToMeters(39.3044)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-30.0), units.degreesToRadians(1.0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "FrontRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-4.1391), units.inchesToMeters(-8.125), units.inchesToMeters(38.3683)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(32.0), units.degreesToRadians(0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "RearLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-8.8236), units.inchesToMeters(7.2958), units.inchesToMeters(36.1419)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(20.0), units.degreesToRadians(166.0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "RearRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-8.8236), units.inchesToMeters(-7.2958), units.inchesToMeters(36.1515)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(20.0), units.degreesToRadians(-166.0))
        ), _poseSensorConstants
      )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "FrontRight": "http://10.28.81.6:1184/?action=stream",
      "FrontLeft": "http://10.28.81.7:1182/?action=stream",
      "RearRight": "http://10.28.81.6:1182/?action=stream",
      "RearLeft": "http://10.28.81.7:1186/?action=stream",
      "Driver": "http://10.28.81.6:1184/?action=stream",
      "Internal": "http://10.28.81.7:1184/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoTargetAlignmentTimeout: units.seconds = 1.5

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      kTargetAlignmentTransforms: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(36), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(22.0), units.inchesToMeters(-6.5), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(22.0), units.inchesToMeters(6.5), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.LeftL4: Transform3d(units.inchesToMeters(23.0), units.inchesToMeters(-6.5), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.RightL4: Transform3d(units.inchesToMeters(23.0), units.inchesToMeters(6.5), 0, Rotation3d(Rotation2d.fromDegrees(0))) 
        },
        TargetType.CoralStation: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(16.0), units.inchesToMeters(0.0), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(16.0), units.inchesToMeters(-24.0), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(16.0), units.inchesToMeters(24.0), 0, Rotation3d(Rotation2d.fromDegrees(0)))
        }
      }                                                                                                                                           

      kTargetPositions: dict[TargetPositionType, TargetPosition] = {
        TargetPositionType.ReefCoralL4: TargetPosition(ElevatorPosition(28.5, 28.0), 6.9, Value.min),
        TargetPositionType.ReefCoralL3: TargetPosition(ElevatorPosition(4.2, 27.5), 6.0, Value.min),
        TargetPositionType.ReefCoralL2: TargetPosition(ElevatorPosition(Value.min, 15.7), 4.25, Value.min),
        TargetPositionType.ReefCoralL1: TargetPosition(ElevatorPosition(Value.min, 20.0), 23.0, Value.min),
        TargetPositionType.ReefAlgaeL3: TargetPosition(ElevatorPosition(8.0, 28.0), 18.5, Value.min),
        TargetPositionType.ReefAlgaeL2: TargetPosition(ElevatorPosition(6.5, 19), 24.0, Value.min),
        TargetPositionType.IntakeReady: TargetPosition(ElevatorPosition(18.0, Value.max), 45.0, Value.min),
        TargetPositionType.CoralStation: TargetPosition(ElevatorPosition(Value.min, Value.min), Value.min, Value.min)
      }
