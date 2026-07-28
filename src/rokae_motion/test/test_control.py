"""Unit tests for fixed behavior-tree workflow planning."""

from pathlib import Path

import pytest

from rokae_motion.bt_executor import BehaviorTreeError, load_behavior_tree
from rokae_motion.control import build_action_plan


TREE = """\
version: 1
execution_locked: true
tasks:
  - id: task_1
    locations:
      - id: station_1
        behaviors:
          - id: behavior_1
            actions:
              - id: wait_1
                type: wait
                duration_s: 0.1
              - id: wait_2
                type: wait
                duration_s: 0.2
"""


def load_tree(tmp_path: Path):
    path = tmp_path / "tree.yaml"
    path.write_text(TREE, encoding="utf-8")
    return load_behavior_tree(path)


def test_workflow_preserves_step_and_repeat_order(
    tmp_path: Path,
) -> None:
    tree = load_tree(tmp_path)
    workflow = [
        {
            "name": "cycle",
            "repeat": 2,
            "steps": [
                {
                    "task_id": "task_1",
                    "action_id": "wait_2",
                },
                {
                    "task_id": "task_1",
                    "action_id": "wait_1",
                },
            ],
        }
    ]

    actions = build_action_plan(tree, workflow)

    assert [action.action_id for action in actions] == [
        "wait_2",
        "wait_1",
        "wait_2",
        "wait_1",
    ]


@pytest.mark.parametrize(
    "workflow, message",
    [
        (
            [{"name": "bad", "repeat": 0, "steps": [{}]}],
            "repeat >= 1",
        ),
        (
            [{"name": "bad", "steps": [{"action_id": "wait_1"}]}],
            "needs task_id",
        ),
    ],
)
def test_invalid_workflow_is_rejected(
    tmp_path: Path, workflow, message: str
) -> None:
    with pytest.raises(BehaviorTreeError, match=message):
        build_action_plan(load_tree(tmp_path), workflow)
