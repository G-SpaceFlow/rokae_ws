#!/usr/bin/env python3
"""Run the installed JSON MoveAbsJ program with one short command.

Before running, edit config/dual_moveabsj_program.json in the source package,
replace every null target with a measured joint angle in radians, and change
replace_before_use to false.

Run after building and sourcing the workspace:

  ros2 run rokae_motion dual_arm_program

The action server must already be running. EXECUTE=True still requires the
operator to type EXECUTE before any action goal is sent.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .ros_dual_moveabsj_client import run_program


# Set ROKAE_MOVEABSJ_PROGRAM to use a JSON file outside the installed package.
PROGRAM_ENVIRONMENT_VARIABLE = "ROKAE_MOVEABSJ_PROGRAM"

# Choose "left", "right" or "both".
ARM = "left"

# Use None for the entire sequence, or enter one JSON step name.
STEP = None

# True enables execution only after an additional EXECUTE confirmation.
# False validates and displays the program without sending a robot command.
EXECUTE = True


def resolve_program_file() -> Path:
    override = os.environ.get(PROGRAM_ENVIRONMENT_VARIABLE)
    if override:
        return Path(override).expanduser().resolve()

    return (
        Path(get_package_share_directory("rokae_motion"))
        / "config"
        / "dual_moveabsj_program.json"
    )


def main() -> int:
    return run_program(
        program=resolve_program_file(),
        arm=ARM,
        selected_step=STEP,
        execute=EXECUTE,
    )


if __name__ == "__main__":
    raise SystemExit(main())
