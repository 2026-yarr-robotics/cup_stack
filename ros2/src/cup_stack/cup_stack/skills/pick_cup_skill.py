"""Skill: pick one cup from an explicit XYZ position.

Standalone — no import of the existing task/runtime/config code.
Pick coordinates are fixed at construction time and may be overridden
per-call via a :class:`PickSpec`.
"""

from cup_stack.skills.base import PickSpec, RobotIO, Skill
from cup_stack.skills.config import DOWN_ORI, SkillStackConfig


class PickCupSkill(Skill):
    """Pick a cup from an explicit XYZ position.

    Only the pick motion is performed: approach at safe_z → open
    gripper → descend to the target Z → grip → lift back to safe_z.
    No place step is included.

    Coordinates given at construction are used by default; passing a
    :class:`PickSpec` to :meth:`execute` overrides all four fields
    (x, y, z, ori) for that single call.
    """

    def __init__(
        self,
        robot: RobotIO,
        x: float,
        y: float,
        z: float,
        config: SkillStackConfig | None = None,
        ori: dict[str, float] | None = None,
    ) -> None:
        self.robot = robot
        self.config = config or SkillStackConfig()
        self.logger = robot.logger
        self.x = x
        self.y = y
        self.z = z
        self.ori = ori or DOWN_ORI
        self.name = f"pick({x:.3f},{y:.3f},{z:.3f})"

    def describe(self) -> str:
        """Return a one-line human summary for plan logging."""

        return f"pick cup at ({self.x:.3f},{self.y:.3f},{self.z:.3f})"

    def execute(self, pick: PickSpec | None = None) -> bool:
        """Pick the cup.

        If ``pick`` is supplied its x/y/z/ori override the values set
        at construction time for this single call.
        """

        x = pick.x if pick is not None else self.x
        y = pick.y if pick is not None else self.y
        z = pick.z if pick is not None else self.z
        ori = (
            pick.ori if pick is not None and pick.ori is not None
            else self.ori
        )

        cfg = self.config
        self.logger.info(
            f"SKILL pick  ({x:.3f},{y:.3f}) z={z:.3f}"
        )

        self.logger.info("  [1] approach @ PICK_SAFE_Z")
        if not self.robot.try_move_to_pose(
            x, y, cfg.pick_safe_z, cfg.safe_z_min, ori=ori,
        ):
            return self._fail(1)

        self.logger.info("  [2] gripper OPEN")
        self.robot.try_open_gripper(cfg.open_sleep_sec)

        self.logger.info(f"  [3] descend -> z={z:.3f}")
        if not self.robot.try_move_to_pose(
            x, y, z, cfg.safe_z_min, ori=ori, lin=True,
        ):
            return self._fail(3)

        self.logger.info("  [4] GRIP")
        if not self.robot.try_grip_cup(cfg.grip_sleep_sec):
            return self._fail(4)

        self.logger.info("  [5] lift -> PICK_SAFE_Z")
        if not self.robot.try_move_to_pose(
            x, y, cfg.pick_safe_z, cfg.safe_z_min, ori=ori, lin=True,
        ):
            return self._fail(5)

        return True

    def _fail(self, step: int) -> bool:
        self.logger.error(f"=== SKILL pick STEP {step} failed ===")
        return False
