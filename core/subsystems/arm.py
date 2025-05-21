from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class Arm(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Arm

    self._hasInitialZeroReset: bool = False

    self._arm = PositionControlModule(self._constants.kArmConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._arm.setSpeed(getInput() * self._constants.kInputLimit),
      lambda: self.reset()
    ).withName("Arm:SetSpeed")
  
  def setPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._arm.setPosition(position)
    ).withName("Arm:SetPosition")
  
  def getPosition(self) -> units.inches:
    return self._arm.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._arm.isAtTargetPosition()
  
  def isAtCoralStationSafePosition(self) -> bool:
    return self._arm.getPosition() <= self._constants.kCoralStationSafePosition
  
  def resetToZero(self) -> Command:
    return self._arm.resetToZero(self).withName("Arm:ResetToZero")

  def hasZeroReset(self) -> bool:
    return self._arm.hasZeroReset()

  def reset(self) -> None:
    self._arm.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Arm/IsAtTargetPosition", self.isAtTargetPosition())
