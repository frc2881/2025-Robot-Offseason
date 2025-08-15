from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from rev import SparkFlex, SparkBaseConfig, SparkBase
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class Intake(Subsystem):
  def __init__(
      self,
      intakeSensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._intakeSensorHasTarget = intakeSensorHasTarget

    self._hasInitialZeroReset: bool = False

    self._intake = PositionControlModule(self._constants.kIntakeConfig)

    self._rollers = SparkFlex(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kRollerMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._rollers.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self.setDefaultCommand(self.hold())

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def hold(self) -> Command:
    return (
      self.startRun(
        lambda: self._intake.reset(),
        lambda: self._intake.setPosition(self._constants.kInPosition)
      )
      .until(lambda: self.isAtTargetPosition())
      .andThen(
        self.startEnd(
          lambda: self._intake.setSpeed(self._constants.kHoldSpeed),
          lambda: None
        )
      )
      .finallyDo(lambda end: self._intake.setSpeed(0))
      .withName("Intake:Hold")
    )

  def intake(self) -> Command:
    return self.runEnd(
      lambda: [
        self._intake.setPosition(self._constants.kOutPosition),
        self._rollers.set(self._constants.kRollerMotorIntakeSpeed)
      ],
      lambda: self._rollers.stopMotor()
    ).withName("Intake:Intake")
  
  def handoff(self) -> Command:
    return self.runEnd(
      lambda: [
        self._intake.setPosition(self._constants.kHandoffPosition),
        self._rollers.set(self._constants.kRollerMotorHandoffSpeed)
      ],
      lambda: self._rollers.stopMotor()
    ).withName("Intake:Handoff")
  
  def eject(self) -> Command:
    return self.runEnd(
      lambda: [
        self._intake.setPosition(self._constants.kEjectPosition),
        self._rollers.set(self._constants.kRollerMotorEjectSpeed if self.isAtTargetPosition() else 0)
      ],
      lambda: self._rollers.stopMotor()
    ).withName("Intake:Eject")
  
  def climb(self) -> Command:
    return self.runEnd(
      lambda: [
        self._intake.setPosition(self._constants.kOutPosition),
        self._rollers.set(self._constants.kRollerMotorClimbSpeed if self.isAtTargetPosition() else 0)
      ],
      lambda: self._rollers.stopMotor()
    ).withName("Intake:Climb")
  
  def setPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._intake.setPosition(position)
    ).withName("Intake:SetPosition")

  def getPosition(self) -> units.inches:
    return self._intake.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._intake.isAtTargetPosition()
  
  def runRollers(self) -> Command:
    return self.startEnd(
      lambda: self._rollers.set(self._constants.kRollerMotorIntakeSpeed),
      lambda: self._rollers.stopMotor()
    ).withName("Intake:RunRollers")

  def isIntakeEnabled(self) -> bool:
    return self._rollers.get() != 0
  
  def isIntakeHolding(self) -> bool:
    return self._intakeSensorHasTarget()
  
  def resetToZero(self) -> Command:
    return self._intake.resetToZero(self).withName("Intake:ResetToZero")

  def hasZeroReset(self) -> bool:
    return self._intake.hasZeroReset()
  
  def reset(self) -> None:
    self._intake.reset()
    self._rollers.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsAtTargetPosition", self.isAtTargetPosition())
    SmartDashboard.putBoolean("Robot/Intake/IsEnabled", self.isIntakeEnabled())
    SmartDashboard.putBoolean("Robot/Intake/IsHolding", self.isIntakeHolding())
    SmartDashboard.putNumber("Robot/Intake/Rollers/Current", self._rollers.getOutputCurrent())