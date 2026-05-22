"""Skill: pick one cup and place it at an externally provided XYZ.

Unlike :class:`cup_stack.skills.place_cup_skill.PlaceCupSkill`, this
skill does not derive its destination from a ``PyramidSlot`` /
``center_xy`` / ``spread_axis``: the place pose is supplied directly,
so the caller (server) owns slot geometry and yaw.  Motion choreography
mirrors ``PlaceCupSkill`` so behaviour stays identical.
"""

from dataclasses import dataclass

from cup_stack.skills.base import PickSpec, RobotIO, Skill
from cup_stack.skills.config import DOWN_ORI, SkillStackConfig
from cup_stack.skills.geometry import make_twist_orientation


@dataclass(frozen=True)
class PlaceSpec:
    """Absolute place pose (base_link, m). ``name`` is a log/debug tag."""

    x: float
    y: float
    z: float
    name: str = ""


class PlaceCupAtSkill(Skill):
    """Pick → travel → place a single cup at an absolute XYZ.

    The pick pose (and its ``ori``) come from ``PickSpec``; the place
    pose comes from ``PlaceSpec`` passed at construction.  No
    dependency on the pyramid slot table or yaw axis — the server
    decides those externally and hands over the resolved place XYZ.
    """

    def __init__(
        self,
        robot: RobotIO,
        place: PlaceSpec,
        config: SkillStackConfig | None = None,
    ) -> None:
        self.robot = robot
        self.place = place
        self.config = config or SkillStackConfig()
        self.logger = robot.logger
        self.name = place.name or "place_cup_at"

    def describe(self) -> str:
        return (
            f"{self.name}: place "
            f"({self.place.x:.3f},{self.place.y:.3f},{self.place.z:.3f})"
        )

    def execute(self, pick: PickSpec | None = None) -> bool:
        """Run the full pick → travel → place cycle for one cup.

        Cycle mirrors :meth:`PlaceCupSkill.execute`: open, descend, grip,
        retreat to safe Z, traverse to place XY, descend with half-twist
        for cup settling, release, retreat.
        """
        if pick is None:
            self.logger.error(f"SKILL {self.name}: a PickSpec is required")
            return False

        cfg = self.config
        r = self.robot
        log = self.logger
        log.info(self.describe())

        pick_ori = pick.ori or DOWN_ORI
        half_twist = make_twist_orientation(cfg.place_twist_deg / 2.0)

        # 1) Approach pick at safe Z, then descend.
        if not r.try_move_to_pose(
            pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min, ori=pick_ori
        ):
            return False
        r.gripper_open()
        r.sleep(cfg.open_sleep_sec)
        if not r.try_move_to_pose(
            pick.x, pick.y, pick.z, cfg.safe_z_min, ori=pick_ori
        ):
            return False

        # 2) Grip and retreat.
        r.gripper_close()
        r.sleep(cfg.grip_sleep_sec)
        if not r.try_move_to_pose(
            pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min, ori=pick_ori
        ):
            return False

        # 3) Travel to place XY at safe Z (gripper-down).
        if not r.try_move_to_pose(
            self.place.x, self.place.y, cfg.pick_safe_z, cfg.safe_z_min,
            ori=DOWN_ORI,
        ):
            return False

        # 4) Descend with half-twist for cup-on-cup settling, release.
        if not r.try_move_to_pose(
            self.place.x, self.place.y, self.place.z, cfg.safe_z_min,
            ori=half_twist,
        ):
            return False
        r.gripper_open()
        r.sleep(cfg.release_sleep_sec)

        # 5) Retreat to safe Z.
        if not r.try_move_to_pose(
            self.place.x, self.place.y, cfg.pick_safe_z, cfg.safe_z_min,
            ori=DOWN_ORI,
        ):
            return False
        return True
