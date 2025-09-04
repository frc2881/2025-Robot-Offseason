from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetPositionType
import core.constants as constants

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTarget(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._robot.drive.alignToTarget(
        self._robot.localization.getRobotPose, 
        lambda: self._robot.localization.getTargetPose(targetAlignmentLocation, self._robot.elevator.isAtReefCoralL4Position()),
        targetAlignmentMode)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')
    )
  
  def isRobotAlignedToTarget(self) -> bool:
    return self._robot.drive.isAlignedToTarget()

  def setRobotToTargetPosition(self, targetPositionType: TargetPositionType) -> Command:
    return (
      cmd.select({
        TargetPositionType.ReefCoralL4: self._setRobotToTargetPosition(TargetPositionType.ReefCoralL4),
        TargetPositionType.ReefCoralL3: self._setRobotToTargetPosition(TargetPositionType.ReefCoralL3),
        TargetPositionType.ReefCoralL2: self._setRobotToTargetPosition(TargetPositionType.ReefCoralL2),
        TargetPositionType.ReefCoralL1: self._setRobotToTargetPosition(TargetPositionType.ReefCoralL1),
        TargetPositionType.ReefAlgaeL3: self._setRobotToTargetPosition(TargetPositionType.ReefAlgaeL3),
        TargetPositionType.ReefAlgaeL2: self._setRobotToTargetPosition(TargetPositionType.ReefAlgaeL2),
        TargetPositionType.CoralStation: self._setRobotToTargetPositionCoralStation()
      }, lambda: targetPositionType)
      .withName(f'Game:AlignRobotToTargetPosition:{ targetPositionType.name }')
    )
  
  def isRobotAtTargetPosition(self) -> bool:
    return (
      self._robot.elevator.isAtTargetPosition() and 
      self._robot.arm.isAtTargetPosition() and 
      self._robot.wrist.isAtTargetPosition()
    )

  def _setRobotToTargetPosition(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm),
      self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist),
      self._robot.hand.intake(),
      cmd.waitUntil(lambda: self.isRobotAtTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    )

  def _setRobotToTargetPositionCoralStation(self):
    return cmd.parallel(
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(
        self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].elevator, isParallel = False)
      ),
      self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].arm),
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(
        self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].wrist)
      )
    )

  def intakeCoralFromStation(self) -> Command:
    return (
      cmd.parallel(
        self._setRobotToTargetPositionCoralStation(),
        self._robot.hand.intake(),
        cmd.waitUntil(lambda: self.isHandHolding()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
      )
      .withName("Game:IntakeCoralFromStation")
    )

  def intakeCoralFromGround(self) -> Command:
    return (
      cmd.sequence(
        cmd.parallel(
          self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].elevator),
          self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].arm),
          self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].wrist),
        ).until(lambda: self.isHandHolding())
      )
      .onlyIf(lambda: not self.isHandHolding())
      .withName("Game:IntakeCoralFromGround")
    )

  def scoreCoral(self) -> Command:
    return (
      self._robot.hand.release()
      .andThen(self.rumbleControllers(ControllerRumbleMode.Both))
      .withName("Game:ScoreCoral")
    )

  def intake(self) -> Command:
    return (
      self._robot.hand.intake()
      .withName("Game:Intake")
    )
  
  def isHandHolding(self) -> bool:
    return self._robot.hand.isHolding()

  def isRobotReadyForScoring(self) -> bool:
    return (
      self.isRobotAlignedToTarget() and
      self.isRobotAtTargetPosition() and
      self.isHandHolding()
    )
  
  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.Short
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
