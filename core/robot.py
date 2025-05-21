from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.xbox import Xbox
from lib.sensors.distance import DistanceSensor
from lib.sensors.binary import BinarySensor
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.elevator import Elevator
from core.subsystems.arm import Arm
from core.subsystems.wrist import Wrist
from core.subsystems.hand import Hand
from core.subsystems.intake import Intake
from core.services.localization import Localization
from core.services.lights import Lights
from core.classes import TargetAlignmentLocation, TargetPositionType, ElevatorStage
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initControllers()
    self._initCommands()
    self._initTriggers()
    self._initLights()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro_NAVX2(constants.Sensors.Gyro.NAVX2.kComType)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)
    SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))
    self.gripperSensor = BinarySensor("Gripper", constants.Sensors.Binary.Gripper.kChannel) 
    self.intakeSensor = BinarySensor("Intake", constants.Sensors.Binary.Intake.kChannel) 
    
  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    self.elevator = Elevator()
    self.arm = Arm()
    self.wrist = Wrist()
    self.hand = Hand(self.gripperSensor.hasTarget)
    self.intake = Intake(self.intakeSensor.hasTarget)
    
  def _initServices(self) -> None:
    self.localization = Localization(
      self.gyro.getRotation, 
      self.drive.getModulePositions, 
      self.poseSensors
    )

  def _initControllers(self) -> None:
    self.driver = Xbox(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operator = Xbox(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX))
    # self.driver.rightStick().whileTrue(cmd.none())
    self.driver.leftStick().whileTrue(self.drive.lock())
    self.driver.leftTrigger().whileTrue(self.game.intakeCoralFromGround())
    self.driver.rightTrigger().whileTrue(self.game.scoreCoral())
    self.driver.rightBumper().whileTrue(self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Right))
    self.driver.leftBumper().whileTrue(self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left))
    # self.driver.povUp().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povDown().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povLeft().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povRight().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.a().whileTrue(cmd.none())
    self.driver.b().whileTrue(self.intake.eject())
    self.driver.y().whileTrue(self.elevator.setSpeed(lambda: constants.Subsystems.Elevator.kCageDeepClimbDownSpeed, ElevatorStage.Lower))
    self.driver.x().whileTrue(self.elevator.setSpeed(lambda: constants.Subsystems.Elevator.kCageDeepClimbUpSpeed, ElevatorStage.Lower))
    self.driver.start().and_((
        self.driver.povLeft()
        .or_(self.driver.povUp())
        .or_(self.driver.povRight())
        .or_(self.driver.povDown())
      ).not_()
    ).whileTrue(self.intake.resetToZero())
    self.driver.back().onTrue(self.gyro.reset())

  def _setupOperator(self) -> None:
    self.elevator.setDefaultCommand(self.elevator.setSpeed(self.operator.getLeftY))
    self.arm.setDefaultCommand(self.arm.setSpeed(self.operator.getRightY))
    self.operator.leftTrigger().whileTrue(self.game.runGripper())
    self.operator.rightTrigger().whileTrue(self.game.scoreCoral())
    # self.operator.leftBumper().whileTrue(cmd.none())
    self.operator.rightBumper().whileTrue(self.game.liftCoralFromIntake())
    self.operator.povUp().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL4))
    self.operator.povRight().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL3))
    self.operator.povDown().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL2))
    # self.operator.povLeft().whileTrue(cmd.none())
    self.operator.a().whileTrue(self.game.intakeCoralFromStation())
    self.operator.b().whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefAlgaeL2))
    self.operator.y().whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefAlgaeL3))
    self.operator.x().and_((self.operator.povLeft()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.CageIntercept))
    self.operator.x().and_((self.operator.povLeft())).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.CageDeepClimb))
    self.operator.start().and_(self.operator.povDown()).whileTrue(self.elevator.resetLowerStageToZero())
    self.operator.start().and_(self.operator.povUp()).whileTrue(self.elevator.resetUpperStageToZero())
    self.operator.start().and_(self.operator.povRight()).whileTrue(self.wrist.togglePosition())
    self.operator.start().and_(self.operator.povLeft()).whileTrue(self.arm.resetToZero())
    self.operator.start().and_((
        self.operator.povLeft()
        .or_(self.operator.povUp())
        .or_(self.operator.povRight())
        .or_(self.operator.povDown())
      ).not_()
    ).whileTrue(self.elevator.setSpeed(self.operator.getLeftY, ElevatorStage.Upper))
    self.operator.back().whileTrue(self.elevator.setSpeed(self.operator.getLeftY, ElevatorStage.Lower))

  def _initLights(self) -> None:
    self.lights = Lights(
      self._hasAllZeroResets,
      self.localization.hasValidVisionTarget,
      self.game.isRobotReadyForScoring,
      self.game.isGripperHolding
    )

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()
    self.elevator.reset()
    self.arm.reset()
    self.wrist.reset()
    self.hand.reset()
    self.intake.reset()

  def _hasAllZeroResets(self) -> bool:
    return (
      self.elevator.hasZeroReset() and self.arm.hasZeroReset() 
      if not utils.isCompetitionMode() else 
      True
    )
      
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/HasAllZeroResets", self._hasAllZeroResets())
