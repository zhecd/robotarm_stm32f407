#!/usr/bin/env python3
"""Fail fast when new code violates the RobotArm layering rules."""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ROBOTARM_ROOT = ROOT / "RobotArm"
RULES = {
    ROBOTARM_ROOT / "Algorithm": ("stm32f4xx", "HAL_", "CMSIS", "FreeRTOS", "Platform/", "BSP/", "Device/", "Service/", "App/"),
    ROBOTARM_ROOT / "Service": ("HAL_", "GPIO_TypeDef", "TIM_HandleTypeDef", "UART_HandleTypeDef", "__disable_irq"),
    ROBOTARM_ROOT / "App": ("stm32f4xx", "HAL_", "GPIO_TypeDef", "TIM_HandleTypeDef",
                    "UART_HandleTypeDef", "bsp/", "device/", "driver/", "internal/"),
}

FILE_RULES = {
    ROBOTARM_ROOT / "Service" / "command" / "command_service.c": ("ctrl_planner.h",),
    ROBOTARM_ROOT / "Service" / "planning" / "internal" / "ctrl_planner.c": ("ctrl_motion_engine.h",),
    ROBOTARM_ROOT / "Service" / "safety" / "safety_service.c": ("motion_service.h",),
}


def strip_comments(source: str) -> str:
    """Keep dependency checks focused on C tokens instead of documentation."""
    without_block_comments = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//.*$", "", without_block_comments, flags=re.MULTILINE)


def main() -> int:
    violations: list[str] = []
    for directory, forbidden in RULES.items():
        if not directory.exists():
            continue
        for path in directory.rglob("*.[ch]"):
            # RobotArm/App/adapters is the explicit boundary that translates CubeMX,
            # BSP and Device details into application-facing operations.
            if directory == ROBOTARM_ROOT / "App" and "adapters" in path.relative_to(directory).parts:
                continue
            text = strip_comments(path.read_text(encoding="utf-8", errors="ignore"))
            for token in forbidden:
                if token in text:
                    violations.append(f"{path.relative_to(ROOT)}: forbidden dependency `{token}`")
    for path, forbidden in FILE_RULES.items():
        if not path.exists():
            continue
        text = strip_comments(path.read_text(encoding="utf-8", errors="ignore"))
        for token in forbidden:
            if token in text:
                violations.append(f"{path.relative_to(ROOT)}: forbidden dependency `{token}`")
    if violations:
        print("Architecture dependency check failed:")
        print("\n".join(violations))
        return 1
    print("Architecture dependency check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
