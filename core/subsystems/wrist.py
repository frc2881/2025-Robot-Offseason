from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Wrist(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._hasInitialZeroReset: bool = False

    self._wrist = RelativePositionControlModule(self._constants.kWristConfig)
    
  def periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._wrist.setSpeed(getInput() * self._constants.kInputLimit),
      lambda: self.reset()
    ).withName("Arm:SetSpeed")

  def setPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._wrist.setPosition(position)
    ).withName("Wrist:SetPosition")

  def getPosition(self) -> units.inches:
    return self._wrist.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._wrist.isAtTargetPosition()

  def resetToZero(self) -> Command:
    return self._wrist.resetToZero(self).withName("Wrist:ResetToZero")

  def hasZeroReset(self) -> bool:
    return self._wrist.hasZeroReset()

  def reset(self) -> None:
    self._wrist.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Wrist/IsAtTargetPosition", self.isAtTargetPosition())
