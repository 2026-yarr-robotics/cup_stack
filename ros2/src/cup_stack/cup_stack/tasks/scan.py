"""Task that traces a rectangular scan path at the current home Z height."""

from cup_stack.config import DOWN_ORI
from cup_stack.runtime import CupStackRuntime

# 추정값 — 실제 로봇에서 IK 검증 후 수정 필요
# DOWN_ORI + safe_z(≈0.55) 기준 M0609 최대 도달 꼭짓점 (시계 방향)
DEFAULT_CORNERS: list[tuple[float, float]] = [
    (0.70,  0.15),   # 1: 正X 正Y (좌전방)
    (0.70, -0.35),   # 2: 正X 負Y (우전방)
    (0.35, -0.35),   # 3: 負X 負Y (우후방)
    (0.35,  0.15),   # 4: 負X 正Y (좌후방)
]


class ScanTask:
    """Scan a rectangle at the home Z while looking straight down.

    Sequence: start → corner1 → corner2 → corner3 → corner4 → start
    All moves use LIN (falls back to PTP when strict=False).
    """

    def __init__(
        self,
        runtime: CupStackRuntime,
        corners: list[tuple[float, float]] | None = None,
        safe_z_min: float = 0.25,
    ) -> None:
        self.runtime = runtime
        self.corners = corners or DEFAULT_CORNERS
        self.safe_z_min = safe_z_min
        self.logger = runtime.logger

    def try_execute(self) -> bool:
        """Execute the rectangular scan and return to the starting pose."""

        self.logger.info("=== Scan task start ===")

        transform = self.runtime.current_ee_matrix()
        home_x = float(transform[0, 3])
        home_y = float(transform[1, 3])
        home_z = float(transform[2, 3])
        self.logger.info(
            f"Start EE: x={home_x:.3f}  y={home_y:.3f}  z={home_z:.3f}"
        )
        self.logger.info(
            f"Corners (z={home_z:.3f} fixed): {self.corners}"
        )

        for i, (cx, cy) in enumerate(self.corners, 1):
            self.logger.info(
                f"[{i}] → corner {i}: ({cx:.3f}, {cy:.3f}, {home_z:.3f})"
            )
            if not self.runtime.try_move_to_pose(
                cx, cy, home_z,
                self.safe_z_min,
                ori=DOWN_ORI,
                lin=True,
                strict=False,
            ):
                self.logger.error(f"Corner {i} 이동 실패 — 중단")
                return False

        self.logger.info(
            f"[5] → start: ({home_x:.3f}, {home_y:.3f}, {home_z:.3f})"
        )
        if not self.runtime.try_move_to_pose(
            home_x, home_y, home_z,
            self.safe_z_min,
            ori=DOWN_ORI,
            lin=True,
            strict=False,
        ):
            self.logger.error("시작 위치 복귀 실패")
            return False

        self.logger.info("=== Scan task complete ===")
        return True
