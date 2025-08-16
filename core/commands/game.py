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
        TargetPositionType.CoralStation: self._setRobotToTargetPositionCoralStation(),
        TargetPositionType.CageDeepClimb: self._setRobotToTargetPositionCageDeepClimb()
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
      self._robot.hand.runGripper(),
      self._robot.intake.hold(),
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
  
  def _setRobotToTargetPositionCageDeepClimb(self) -> Command:
    return cmd.sequence(
      cmd.waitSeconds(0.5),
      cmd.runOnce(lambda: self._robot.elevator.setUpperStageSoftLimitsEnabled(False)),
      cmd.parallel(
        self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].elevator, isParallel = False),
        cmd.waitUntil(lambda: self._robot.elevator.isAtTargetPosition()).andThen(
          self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].arm)
        ),
        self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].wrist),
        self._robot.intake.climb()
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAtTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    ).finallyDo(
      lambda end: self._robot.elevator.setUpperStageSoftLimitsEnabled(True)
    )

  def intakeCoralFromStation(self) -> Command:
    return (
      cmd.parallel(
        self._setRobotToTargetPositionCoralStation(),
        self._robot.hand.runGripper(),
        cmd.waitUntil(lambda: self.isGripperHolding()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
      )
      .withName("Game:IntakeCoralFromStation")
    )

  def intakeCoralFromGround(self) -> Command:
    return (
      cmd.sequence(
        cmd.parallel(
          self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].elevator),
          self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].wrist).withTimeout(2.0),
          self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].arm),
          self._robot.intake.intake()
        ).until(lambda: self.isIntakeHolding()),
        self.liftCoralFromIntake()
      )
      .onlyIf(lambda: not self.isGripperHolding())
      .withName("Game:IntakeCoralFromGround")
    )

  def liftCoralFromIntake(self) -> Command:
    return (
      cmd.sequence(
        cmd.parallel(
          self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].elevator),
          self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].arm),
          self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].wrist),
          cmd.sequence(
            self._robot.intake.setPosition(constants.Subsystems.Intake.kHandoffPosition).until(
              lambda: self.isRobotAtTargetPosition() and self._robot.intake.isAtTargetPosition()
            ),
            cmd.parallel(
              self._robot.hand.runGripper(),
              self._robot.intake.handoff()
            )
          ),
        ).until(lambda: self.isGripperHolding()),
        cmd.parallel(
          self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].elevator, isParallel = False),
          self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].arm),
          self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].wrist),
          self._robot.hand.runGripper(),
          self.rumbleControllers(ControllerRumbleMode.Both)
        )
        .onlyIf(lambda: self.isGripperHolding())
      )
      .withName("Game:LiftCoralFromIntake")
    )

  def scoreCoral(self) -> Command:
    return (
      self._robot.hand.releaseGripper()
      .andThen(self.rumbleControllers(ControllerRumbleMode.Both))
      .withName("Game:ScoreCoral")
    )

  def runGripper(self) -> Command:
    return (
      self._robot.hand.runGripper()
      .alongWith(self._robot.wrist.refreshPosition())
      .withName("Game:RunGripper")
    )
  
  def isGripperHolding(self) -> bool:
    return self._robot.hand.isGripperHolding()
  
  def isIntakeHolding(self) -> bool:
    return self._robot.intakeSensor.hasTarget()
  
  def isRobotReadyForScoring(self) -> bool:
    return (
      self.isRobotAlignedToTarget() and
      self.isRobotAtTargetPosition() and
      self.isGripperHolding()
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
