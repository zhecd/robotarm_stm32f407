#!/usr/bin/env python3
"""Fail fast when new code violates the RobotArm layering rules."""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
RULES = {
    ROOT / "Domain": ("stm32f4xx", "HAL_", "CMSIS", "FreeRTOS", "Platform/", "BSP/", "Device/", "Service/", "App/"),
    ROOT / "Service": ("HAL_", "GPIO_TypeDef", "TIM_HandleTypeDef", "UART_HandleTypeDef", "__disable_irq"),
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
