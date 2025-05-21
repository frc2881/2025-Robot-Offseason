from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkMax, SparkBaseConfig, SparkBase
from lib import logger, utils
from lib.classes import Position
import core.constants as constants

class Wrist(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._targetPosition = Position.Unknown

    self._motor = SparkMax(self._constants.kMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkBaseConfig()
    self._sparkConfig.smartCurrentLimit(self._constants.kMotorCurrentLimit)
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    
  def periodic(self) -> None:
    self._updateTelemetry()
    
  def setPosition(self, position: Position) -> Command:
    return (
      self.startEnd(
        lambda: [
          self._setTargetPosition(Position.Unknown),
          self._motor.set(self._constants.kMotorUpSpeed if position == Position.Up else -self._constants.kMotorDownSpeed)
        ],
        lambda: None
      )
      .withTimeout(self._constants.kSetPositionTimeout)
      .andThen(self.runOnce(lambda: self._setTargetPosition(position)))
      .andThen(
        self.startEnd(
          lambda: self._motor.set(self._constants.kMotorHoldUpSpeed if position == Position.Up else -self._constants.kMotorHoldDownSpeed),
          lambda: None
        )
      )
      .finallyDo(lambda end: self._motor.stopMotor())
      .withName("Wrist:SetPosition")
    )

  def togglePosition(self) -> Command:
    return cmd.either(
      self.setPosition(Position.Up), 
      self.setPosition(Position.Down), 
      lambda: self._targetPosition != Position.Up
    ).withName("Wrist:TogglePosition")

  def refreshPosition(self) -> Command:
    self.setPosition(self._targetPosition)

  def getPosition(self) -> Position:
    return self._targetPosition
  
  def _setTargetPosition(self, position: Position) -> None:
    self._targetPosition = position

  def isAtTargetPosition(self) -> bool:
    return self._targetPosition != Position.Unknown

  def reset(self) -> None:
    self._motor.stopMotor()
    self._setTargetPosition(Position.Unknown)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Wrist/IsAtTargetPosition", self.isAtTargetPosition())
    SmartDashboard.putString("Robot/Wrist/Position", self._targetPosition.name)
    SmartDashboard.putNumber("Robot/Wrist/Current", self._motor.getOutputCurrent())
