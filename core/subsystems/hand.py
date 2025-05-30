from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkFlex, SparkBaseConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Hand(Subsystem):
  def __init__(
      self, 
      gripperSensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._gripperSensorHasTarget = gripperSensorHasTarget
    
    self._gripperMotor = SparkFlex(self._constants.kGripperMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kGripperMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._gripperMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
      
  def runGripper(self) -> Command:
    return self.runEnd(
      lambda: self._gripperMotor.set(self._constants.kGripperMotorIntakeSpeed if not self.isGripperHolding() else self._constants.kGripperMotorHoldSpeed),
      lambda: self._gripperMotor.stopMotor()
    ).withName("Hand:RunGripper")
  
  # TODO: test using gripper sensor as trigger for ending command vs. using a fixed timeout
  def releaseGripper(self) -> Command:
    return self.startEnd(
      lambda: self._gripperMotor.set(-self._constants.kGripperMotorReleaseSpeed),
      lambda: self._gripperMotor.stopMotor()
    ).withTimeout(
      self._constants.kGripperReleaseTimeout
    ).withName("Hand:ReleaseGripper")

  def isGripperEnabled(self) -> bool:
    return self._gripperMotor.get() != 0
  
  def isGripperHolding(self) -> bool:
    return self._gripperSensorHasTarget()
  
  def reset(self) -> None:
    self._gripperMotor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self.isGripperEnabled())
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsHolding", self.isGripperHolding())
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent())
 