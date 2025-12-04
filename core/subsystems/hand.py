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
    
    self._motor = SparkFlex(self._constants.MOTOR_CAN_ID, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.MOTOR_CURRENT_LIMIT)
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
      
  def intakeCoral(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(self._constants.MOTOR_INTAKE_CORAL_SPEED),
      lambda: self._motor.stopMotor()
    ).withName("Hand:IntakeCoral")
  
  def intakeCoralFromGround(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(self._constants.MOTOR_INTAKE_CORAL_FROM_GROUND_SPEED),
      lambda: self._motor.stopMotor()
    ).withName("Hand:IntakeCoralFromGround")

  def scoreCoral(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.MOTOR_SCORE_CORAL_SPEED),
      lambda: self._motor.stopMotor()
    ).beforeStarting(
      self.startEnd(
        lambda: self._motor.set(-0.2),
        lambda: self._motor.stopMotor()
      ).withTimeout(0.08)
    ).withTimeout(
      self._constants.SCORE_CORAL_TIMEOUT
    ).withName("Hand:ScoreCoral")
  
  def intakeAlgae(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(-self._constants.MOTOR_INTAKE_ALGAE_SPEED),
      lambda: self._motor.stopMotor()
    ).withName("Hand:IntakeAlgae")
  
  def scoreAlgae(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(self._constants.MOTOR_SCORE_ALGAE_SPEED),
      lambda: self._motor.stopMotor()
    ).withTimeout(
      self._constants.SCORE_ALGAE_TIMEOUT
    ).withName("Hand:ScoreAlgae")

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
 