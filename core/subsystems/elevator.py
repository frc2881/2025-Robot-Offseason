from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.classes import MotorDirection
from lib.components.relative_position_control_module import RelativePositionControlModule
from core.classes import ElevatorPosition, ElevatorStage
import core.constants as constants

class Elevator(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroReset: bool = False

    self._lowerStage = RelativePositionControlModule(self._constants.kLowerStageConfig)
    self._lowerStageHelper = RelativePositionControlModule(self._constants.kLowerStageHelperConfig)
    self._upperStage = RelativePositionControlModule(self._constants.kUpperStageConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, getInput: Callable[[], units.percent], elevatorStage: ElevatorStage = ElevatorStage.Both) -> Command:
    return self.runEnd(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit, elevatorStage),
      lambda: self.reset()
    ).withName(f'Elevator:SetSpeed:{ elevatorStage.name }')
  
  def _setSpeed(self, speed: units.percent, elevatorStage: ElevatorStage) -> None:
    match elevatorStage:
      case ElevatorStage.Lower:
        self._lowerStage.setSpeed(speed)
        self._upperStage.setSpeed(0)
      case ElevatorStage.Upper:
        self._lowerStage.setSpeed(0)
        self._upperStage.setSpeed(speed)
      case ElevatorStage.Both:
        self._lowerStage.setSpeed(speed)
        self._upperStage.setSpeed(
          speed 
          if speed > 0 or self._lowerStage.isAtSoftLimit(MotorDirection.Reverse, self._constants.kLowerStageSoftLimitBuffer) 
          else 0
        )
  
  def setPosition(self, elevatorPosition: ElevatorPosition, isParallel: bool = True) -> Command:
    return self.run(
      lambda: self._lowerStage.setPosition(elevatorPosition.lowerStage),
    ).until(
      lambda: self._lowerStage.isAtTargetPosition()
    ).onlyIf(lambda: not isParallel).andThen(
      self.run(
        lambda: [
          self._lowerStage.setPosition(elevatorPosition.lowerStage),
          self._upperStage.setPosition(elevatorPosition.upperStage)
        ]
      )
    ).withName("Elevator:SetPosition")

  def getPosition(self) -> ElevatorPosition:
    return ElevatorPosition(self._lowerStage.getPosition(), self._upperStage.getPosition())

  def isAtTargetPosition(self) -> bool:
    return self._lowerStage.isAtTargetPosition() and self._upperStage.isAtTargetPosition()
  
  def isAtReefCoralL4Position(self) -> bool:
    return self.getPosition().lowerStage > self._constants.kLowerStageReefCoralL4Position

  def setUpperStageSoftLimitsEnabled(self, isEnabled: bool) -> None:
    self._upperStage.setSoftLimitsEnabled(isEnabled)

  def resetLowerStageToZero(self) -> Command:
    return self._lowerStage.resetToZero(self).withName("Elevator:ResetLowerStageToZero")

  def resetUpperStageToZero(self) -> Command:
    return self._upperStage.resetToZero(self).withName("Elevator:ResetUpperStageToZero")
  
  def hasZeroReset(self) -> bool:
    return self._lowerStage.hasZeroReset() and self._upperStage.hasZeroReset()
  
  def reset(self) -> None:
    self._lowerStage.reset()
    self._upperStage.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Elevator/IsAtTargetPosition", self.isAtTargetPosition())
