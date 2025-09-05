from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkFlex, SparkBaseConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Hand(Subsystem):
  def __init__(
      self, 
      sensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._sensorHasTarget = sensorHasTarget
    
    self._motor = SparkFlex(self._constants.kMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
      
  def intake(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(self._constants.kMotorIntakeSpeed if not self.isHolding() else self._constants.kMotorHoldSpeed),
      lambda: self._motor.stopMotor()
    ).withName("Hand:Intake")
  
  def release(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(-self._constants.kMotorReleaseSpeed),
      lambda: self._motor.stopMotor()
    ).withTimeout(
      self._constants.kReleaseTimeout
    ).withName("Hand:Release")
  
  def scoreCoral(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kMotorReleaseSpeed),
      lambda: self._motor.stopMotor()
    ).withTimeout(
      self._constants.kScoreCoralTimeout
    ).withName("Hand:ScoreCoral")

  def isEnabled(self) -> bool:
    return self._motor.get() != 0
  
  def isHolding(self) -> bool:
    return self._sensorHasTarget()
  
  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/IsEnabled", self.isEnabled())
    SmartDashboard.putBoolean("Robot/Hand/IsHolding", self.isHolding())
    SmartDashboard.putNumber("Robot/Hand/Current", self._motor.getOutputCurrent())
 