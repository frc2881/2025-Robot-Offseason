from commands2 import cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.xbox import Xbox
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
    self.handSensor = BinarySensor("Hand", constants.Sensors.Binary.Hand.kChannel) 
    
  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    self.elevator = Elevator()
    self.arm = Arm()
    self.wrist = Wrist()
    self.hand = Hand(self.handSensor.hasTarget)
    
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
    self.driver.povUp().whileTrue(self.wrist.setSpeed(lambda: -0.2))
    self.driver.povDown().whileTrue(self.wrist.setSpeed(lambda: 0.2))
    # self.driver.povLeft().whileTrue(cmd.none())
    # self.driver.povRight().whileTrue(cmd.none())
    # self.driver.a().whileTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    # self.driver.y().whileTrue(cmd.none())
    # self.driver.x().whileTrue(cmd.none())
    # self.driver.start().whileTrue(cmd.none())
    self.driver.back().whileTrue(cmd.waitSeconds(0.5).andThen(self.gyro.reset()))

  def _setupOperator(self) -> None:
    self.elevator.setDefaultCommand(self.elevator.setSpeed(self.operator.getLeftY))
    self.arm.setDefaultCommand(self.arm.setSpeed(self.operator.getRightY))
    self.operator.leftTrigger().whileTrue(self.game.intakeAlgae())
    self.operator.rightTrigger().whileTrue(self.game.scoreAlgae())
    self.operator.leftBumper().whileTrue(self.hand.intakeCoral())
    self.operator.rightBumper().whileTrue(self.hand.scoreCoral())
    self.operator.povUp().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL4))
    self.operator.povRight().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL3))
    self.operator.povDown().and_((self.operator.start()).not_()).whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefCoralL2))
    self.operator.povLeft().whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.Barge))
    self.operator.a().whileTrue(self.game.intakeCoralFromStation())
    self.operator.b().whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefAlgaeL2))
    self.operator.y().whileTrue(self.game.setRobotToTargetPosition(TargetPositionType.ReefAlgaeL3))
    # self.operator.x().whileTrue(cmd.none())
    self.operator.start().and_(self.operator.povDown()).whileTrue(self.elevator.resetLowerStageToZero())
    self.operator.start().and_(self.operator.povUp()).whileTrue(self.elevator.resetUpperStageToZero())
    self.operator.start().and_(self.operator.povRight()).whileTrue(self.wrist.resetToZero())
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
      self.game.isHandHolding
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

  def _hasAllZeroResets(self) -> bool:
    return (
      self.elevator.hasZeroReset() and self.arm.hasZeroReset() 
      if not utils.isCompetitionMode() else 
      True
    )
      
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/HasAllZeroResets", self._hasAllZeroResets())
