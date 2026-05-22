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

        Mirrors :meth:`PlaceCupSkill.execute` step-for-step so motion
        choreography (orientations, linear/joint moves, sleep timings)
        stays identical — only the place pose source differs.
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
        full_twist = make_twist_orientation(cfg.place_twist_deg)

        # ── Pick ─────────────────────────────────────────────────────
        log.info("  [1] pick XY move @ PICK_SAFE_Z")
        if not r.try_move_to_pose(
            pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min, ori=pick_ori,
        ):
            return False
        log.info("  [2] gripper OPEN")
        r.try_open_gripper(cfg.open_sleep_sec)
        log.info(f"  [3] pick descend -> z={pick.z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, pick.z, cfg.safe_z_min, ori=pick_ori, lin=True,
        ):
            return False
        log.info("  [4] GRIP")
        if not r.try_grip_cup(cfg.grip_sleep_sec):
            return False
        log.info("  [5] lift -> PICK_SAFE_Z")
        if not r.try_move_to_pose(
            pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min,
            ori=pick_ori, lin=True,
        ):
            return False

        # ── Travel ───────────────────────────────────────────────────
        log.info(
            f"  [6] target XY move ({self.place.x:.3f},{self.place.y:.3f}) "
            f"@ z={cfg.pick_safe_z:.3f}"
        )
        if not r.try_move_to_pose(
            self.place.x, self.place.y, cfg.pick_safe_z, cfg.safe_z_min,
        ):
            return False

        # ── Place ────────────────────────────────────────────────────
        approach_z = cfg.pick_safe_z
        if approach_z > self.place.z:
            mid_z = self.place.z + (approach_z - self.place.z) / 2.0
        else:
            mid_z = self.place.z + 0.02

        log.info(f"  [7a] place mid -> z={mid_z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, mid_z, cfg.safe_z_min,
            ori=half_twist,
        ):
            return False
        log.info(f"  [7b] place final -> z={self.place.z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, self.place.z, cfg.safe_z_min,
            ori=full_twist,
        ):
            return False
        log.info("  [8] RELEASE")
        r.try_release_cup(cfg.release_sleep_sec)

        lift_z = max(cfg.pick_safe_z, self.place.z + 0.02)
        log.info(f"  [9] lift -> z={lift_z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, lift_z, cfg.safe_z_min,
            ori=full_twist,
        ):
            return False
        return True
