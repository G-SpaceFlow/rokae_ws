#!/usr/bin/env python3
"""Fixed behavior-tree workflow, similar to the reference control.py."""

import argparse
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Sequence


# Allow both `python3 control.py` and the installed `bt_control` command.
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rokae_motion.bt_executor import (  # noqa: E402
    ActionSpec,
    BehaviorTree,
    BehaviorTreeError,
    BehaviorTreeExecutor,
    TreeSelection,
    select_actions,
)
from rokae_motion.bt_runner_config import (  # noqa: E402
    add_common_runner_args,
    build_config_from_args,
)


# Behavior-tree YAML used by this fixed workflow. Change it here when needed.
BEHAVIOR_TREE_PATH = (
    PACKAGE_ROOT / "behavior_trees" / "examples" / "展会动作.yaml"
)


# Fixed process: groups run from top to bottom; steps run in written order.
WORKFLOW = [
    {
        "name": "展会调试流程",
        "repeat": 1,
        "steps": [
            {
                "task_id": "task_001",
                "location_id": "location_001",
                "behavior_id": "behavior_001",
                # "action_id": "action_004",
            },
        ],
    },
]


def parse_arguments(
    arguments: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the fixed WORKFLOW in control.py"
    )
    add_common_runner_args(parser, execute_by_default=True)
    return parser.parse_args(arguments)


def build_action_plan(
    tree: BehaviorTree,
    workflow: Sequence[Dict[str, Any]] = WORKFLOW,
) -> List[ActionSpec]:
    """Expand workflow groups and repeats into one ordered action list."""
    actions: List[ActionSpec] = []
    for group in workflow:
        name = str(group.get("name", "workflow"))
        repeat = int(group.get("repeat", 1))
        steps = group.get("steps", [])
        if repeat < 1 or not steps:
            raise BehaviorTreeError(
                f"workflow group {name!r} needs repeat >= 1 and steps"
            )

        print(
            f"Workflow group: {name}, "
            f"repeat={repeat}, steps={len(steps)}"
        )
        for cycle_index in range(repeat):
            for step_index, step in enumerate(steps):
                if not step.get("task_id"):
                    raise BehaviorTreeError(
                        f"{name} step {step_index + 1} needs task_id"
                    )
                selection = TreeSelection(
                    task_id=step.get("task_id"),
                    location_id=step.get("location_id"),
                    behavior_id=step.get("behavior_id"),
                    action_id=step.get("action_id"),
                )
                selected = select_actions(tree, selection)
                actions.extend(selected)
                print(
                    f"  cycle {cycle_index + 1}/{repeat}, "
                    f"step {step_index + 1}/{len(steps)} "
                    f"-> {len(selected)} action(s)"
                )
    return actions


def main(arguments: Optional[Sequence[str]] = None) -> int:
    options = parse_arguments(arguments)
    executor = BehaviorTreeExecutor(
        build_config_from_args(
            options, initialize_before_execute=True
        )
    )
    try:
        tree = executor.load_tree(BEHAVIOR_TREE_PATH)
        actions = build_action_plan(tree)
        return executor.execute_actions(tree, actions)
    except (BehaviorTreeError, TypeError, ValueError) as exc:
        print(f"Workflow error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
