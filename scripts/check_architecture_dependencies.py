#!/usr/bin/env python3
"""检查模块依赖；静态通过不代表硬件行为或时序已经验收。"""
from pathlib import Path
from architecture_rules import check_tree


def main() -> int:
    violations = check_tree(Path(__file__).resolve().parents[1])
    if violations:
        print("Architecture dependency check failed:")
        print("\n".join(violations))
        return 1
    print("Architecture dependency check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
