#!/usr/bin/env python3
"""Command-line entry point for Rokae YAML behavior trees."""

import argparse
from pathlib import Path
import sys
from typing import Optional, Sequence

from ament_index_python.packages import get_package_share_directory

from .bt_executor import (
    BehaviorTreeError,
    BehaviorTreeExecutor,
    TreeSelection,
)
from .bt_runner_config import (
    add_common_runner_args,
    build_config_from_args,
)


def default_tree_path() -> Path:
    return (
        Path(get_package_share_directory("rokae_motion"))
        / "behavior_trees"
        / "examples"
        / "展会动作.yaml"
    )


def parse_arguments(
    arguments: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate or execute a task/location/behavior/action YAML tree"
        )
    )
    parser.add_argument(
        "tree",
        type=Path,
        nargs="?",
        default=None,
        help="YAML tree; defaults to the installed 展会动作.yaml",
    )
    parser.add_argument("--task-id")
    parser.add_argument("--location-id")
    parser.add_argument("--behavior-id")
    parser.add_argument("--action-id")
    add_common_runner_args(parser)
    return parser.parse_args(arguments)


def main(arguments: Optional[Sequence[str]] = None) -> int:
    options = parse_arguments(arguments)
    config = build_config_from_args(options)
    executor = BehaviorTreeExecutor(config)
    selection = TreeSelection(
        task_id=options.task_id,
        location_id=options.location_id,
        behavior_id=options.behavior_id,
        action_id=options.action_id,
    )
    try:
        tree = executor.load_tree(options.tree or default_tree_path())
        return executor.execute_selected(tree, selection)
    except BehaviorTreeError as exc:
        print(f"Behavior tree error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
