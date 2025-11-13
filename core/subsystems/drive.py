from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from ntcore import NetworkTableInstance
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import State, Position, MotorIdleMode, SpeedMode, DriveOrientation, TargetAlignmentMode
from lib.components.swerve_module import SwerveModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._modules = tuple(SwerveModule(c) for c in self._constants.kSwerveModuleConfigs)
    self._modulesStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Robot/Drive/Modules/States", SwerveModuleState).publish()

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._driftCorrectionState = State.Stopped
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetAlignmentState = State.Stopped
    self._targetAlignmentPose: Pose3d = None
    self._targetAlignmentTranslationXController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.translationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.translationMaxVelocity, self._constants.kTargetAlignmentConstants.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.translationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.translationMaxVelocity, self._constants.kTargetAlignmentConstants.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentRotationController = ProfiledPIDController(
      *self._constants.kTargetAlignmentConstants.rotationPID, 
      TrapezoidProfile.Constraints(self._constants.kTargetAlignmentConstants.rotationMaxVelocity, self._constants.kTargetAlignmentConstants.rotationMaxAcceleration)
    )
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedMode = SendableChooser()
    speedMode.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedMode.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedMode.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedMode)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientation = SendableChooser()
    orientation.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientation.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientation.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientation)

    self._driftCorrection: State = State.Enabled
    driftCorrection = SendableChooser()
    driftCorrection.setDefaultOption(State.Enabled.name, State.Enabled)
    driftCorrection.addOption(State.Disabled.name, State.Disabled)
    driftCorrection.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrection)

    idleMode = SendableChooser()
    idleMode.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleMode.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleMode.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleMode)

    self._lockPosition = Position.Unlocked

    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotLength", self._constants.kRobotLength)
    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotWidth", self._constants.kRobotWidth)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getInputX: Callable[[], units.percent], getInputY: Callable[[], units.percent], getInputRotation: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).onlyIf(
      lambda: self._lockPosition == Position.Unlocked
    ).withName("Drive:Drive")

  def _drive(self, inputX: units.percent, inputY: units.percent, inputRotation: units.percent) -> None:
    if self._driftCorrection == State.Enabled:
      isTranslating: bool = inputX != 0 or inputY != 0
      isRotating: bool = inputRotation != 0
      if isTranslating and not isRotating and not self._driftCorrectionState == State.Running:
        self._driftCorrectionState = State.Running
        self._driftCorrectionController.reset()
        self._driftCorrectionController.setSetpoint(self._getGyroHeading())
      elif isRotating or not isTranslating:
        self._driftCorrectionState = State.Stopped
      if self._driftCorrectionState == State.Running:
        inputRotation = self._driftCorrectionController.calculate(self._getGyroHeading())
        if self._driftCorrectionController.atSetpoint():
          inputRotation = 0

    if self._speedMode == SpeedMode.Demo:
      inputX = self._inputXFilter.calculate(inputX * self._constants.kInputLimitDemo)
      inputY = self._inputYFilter.calculate(inputY * self._constants.kInputLimitDemo)
      inputRotation = self._inputRotationFilter.calculate(inputRotation * self._constants.kInputLimitDemo)

    speedX: units.meters_per_second = inputX * self._constants.kTranslationSpeedMax
    speedY: units.meters_per_second = inputY * self._constants.kTranslationSpeedMax
    speedRotation: units.degrees_per_second = inputRotation * self._constants.kRotationSpeedMax
    
    self.setChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, units.degreesToRadians(speedRotation), Rotation2d.fromDegrees(self._getGyroHeading()))
      if self._orientation == DriveOrientation.Field else
      ChassisSpeeds(speedX, speedY, units.degreesToRadians(speedRotation))
    )

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.kDriveKinematics.toChassisSpeeds(self._getModuleStates())

  def getModulePositions(self) -> tuple[SwerveModulePosition, ...]:
    return tuple(m.getPosition() for m in self._modules)

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      self._constants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
          self._constants.kDriveKinematics.toChassisSpeeds(
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
              self._constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), 
              self._constants.kTranslationSpeedMax
            )
          ), 0.02
        )
      ), self._constants.kTranslationSpeedMax
    )
    for i, m in enumerate(self._modules):
      m.setTargetState(swerveModuleStates[i])

    if self._targetAlignmentState != State.Running:
      if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0:
        self._resetTargetAlignment()

  def _getModuleStates(self) -> tuple[SwerveModuleState, ...]:
    return tuple(m.getState() for m in self._modules)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for m in self._modules: m.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def lock(self) -> Command:
    return self.startEnd(
      lambda: self._setLockPosition(Position.Locked),
      lambda: self._setLockPosition(Position.Unlocked)
    ).withName("Drive:Lock")
  
  def _setLockPosition(self, position: Position) -> None:
    self._lockPosition = position
    if position == Position.Locked:
      for i, m in enumerate(self._modules): 
        m.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if i in { 0, 3 } else -45)))

  def alignToTarget(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d], targetAlignmentMode: TargetAlignmentMode) -> Command:
    return self.startRun(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(), targetAlignmentMode),
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).until(
      lambda: self._targetAlignmentState == State.Completed
    ).finallyDo(
      lambda end: self._endTargetAlignment()
    ).withName("Drive:AlignToTarget")
  
  def _initTargetAlignment(self, robotPose: Pose2d, targetAlignmentPose: Pose3d, targetAlignmentMode: TargetAlignmentMode) -> None:
    self._resetTargetAlignment()
    self._targetAlignmentPose = targetAlignmentPose
    self._targetAlignmentState = State.Running
    self._targetAlignmentTranslationXController.reset(0)
    self._targetAlignmentTranslationXController.setGoal(0)
    self._targetAlignmentTranslationYController.reset(0)
    self._targetAlignmentTranslationYController.setGoal(0)
    self._targetAlignmentRotationController.reset(robotPose.rotation().degrees())
    self._targetAlignmentRotationController.setGoal(
      targetAlignmentPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset
      if targetAlignmentMode == TargetAlignmentMode.Translation else
      utils.wrapAngle(utils.getTargetHeading(robotPose, targetAlignmentPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset)
    )

  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    speedTranslationX = 0
    speedTranslationY = 0
    speedRotation = 0
    if not self._targetAlignmentRotationController.atGoal():
      speedRotation = self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees())
    if targetAlignmentMode == TargetAlignmentMode.Translation:
      targetTranslation = self._targetAlignmentPose.toPose2d() - robotPose
      if not self._targetAlignmentTranslationXController.atGoal():
        speedTranslationX = -self._targetAlignmentTranslationXController.calculate(targetTranslation.X())
      if not self._targetAlignmentTranslationYController.atGoal():
        speedTranslationY = -self._targetAlignmentTranslationYController.calculate(targetTranslation.Y())
    self._setModuleStates(ChassisSpeeds(speedTranslationX, speedTranslationY, units.degreesToRadians(speedRotation)))
    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._targetAlignmentState = State.Completed

  def _endTargetAlignment(self) -> None:
    self._setModuleStates(ChassisSpeeds())
    if self._targetAlignmentState != State.Completed:
      self._targetAlignmentState = State.Stopped

  def isAlignedToTarget(self) -> bool:
    return self._targetAlignmentState == State.Completed
  
  def _resetTargetAlignment(self) -> None:
    self._targetAlignmentPose = None
    self._targetAlignmentState = State.Stopped

  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
    self._resetTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self.isAlignedToTarget())
    SmartDashboard.putString("Robot/Drive/TargetAlignmentState", self._targetAlignmentState.name)
    SmartDashboard.putString("Robot/Drive/LockPosition", self._lockPosition.name)
    self._modulesStatesPublisher.set(self._getModuleStates())
