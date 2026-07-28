"""Unit tests for behavior-tree validation and hierarchy selection."""

from pathlib import Path

import pytest

from rokae_motion.bt_executor import (
    BehaviorTreeError,
    TreeSelection,
    load_behavior_tree,
    select_actions,
)


TEMPLATE = """\
version: 1
execution_locked: true
tasks:
  - id: task_1
    locations:
      - id: station_1
        behaviors:
          - id: behavior_1
            actions:
              - id: disabled_move
                type: move_l
                enabled: false
                targets:
                  right:
                    pose: [null, null, null, null, null, null]
                    elbow: null
              - id: wait_1
                type: wait
                duration_s: 0.5
"""


def write_tree(tmp_path: Path, contents: str = TEMPLATE) -> Path:
    tree_path = tmp_path / "tree.yaml"
    tree_path.write_text(contents, encoding="utf-8")
    return tree_path


def test_disabled_template_placeholders_are_safe(tmp_path: Path) -> None:
    tree = load_behavior_tree(write_tree(tmp_path))

    assert tree.execution_locked
    assert len(tree.actions) == 2
    assert not tree.actions[0].enabled
    assert tree.actions[1].parameters["duration_s"] == 0.5


def test_action_id_selection(tmp_path: Path) -> None:
    tree = load_behavior_tree(write_tree(tmp_path))

    selected = select_actions(
        tree, TreeSelection(task_id="task_1", action_id="wait_1")
    )

    assert [action.action_id for action in selected] == ["wait_1"]


def test_duplicate_action_ids_preserve_yaml_order(tmp_path: Path) -> None:
    duplicate_actions = TEMPLATE.replace(
        """\
              - id: wait_1
                type: wait
                duration_s: 0.5
""",
        """\
              - id: repeated
                name: first
                type: wait
                duration_s: 0.1
              - id: repeated
                name: second
                type: wait
                duration_s: 0.2
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, duplicate_actions))
    selected = select_actions(
        tree, TreeSelection(task_id="task_1", action_id="repeated")
    )

    assert [action.action_name for action in selected] == [
        "first",
        "second",
    ]
    assert [
        action.parameters["duration_s"] for action in selected
    ] == [0.1, 0.2]


def test_enabled_move_l_rejects_placeholders(tmp_path: Path) -> None:
    invalid = TEMPLATE.replace("enabled: false", "enabled: true")

    with pytest.raises(BehaviorTreeError, match="must be a number"):
        load_behavior_tree(write_tree(tmp_path, invalid))


def test_relative_move_l_is_normalized(tmp_path: Path) -> None:
    relative = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: relative_1
                type: move_l_relative
                relative_offset:
                  left:
                    dx: 0.01
                    dz: 0.02
                    rx: -2.7
                    rz: -0.7
                  right: {dy: -0.01}
                speed_mm_s: 30.0
                zone_mm: 0.0
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, relative))
    action = select_actions(
        tree, TreeSelection(action_id="relative_1")
    )[0]

    assert action.parameters["offsets"] == {
        "left": [0.01, 0.0, 0.02],
        "right": [0.0, -0.01, 0.0],
    }
    assert action.parameters["orientations"] == {
        "left": [-2.7, None, -0.7],
    }


def test_relative_move_l_rejects_waist_rotation(tmp_path: Path) -> None:
    relative = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: relative_1
                type: move_l_relative
                relative_offset:
                  rotate_with_waist: true
                  dz: 0.02
                speed_mm_s: 30.0
              - id: wait_1
""",
    )

    with pytest.raises(BehaviorTreeError, match="waist"):
        load_behavior_tree(write_tree(tmp_path, relative))


def test_relative_move_l_accepts_sparse_translation_and_orientation(
    tmp_path: Path,
) -> None:
    relative = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: relative_1
                type: move_l_relative
                relative_offset:
                  left:
                    dx: 0.02
                    ry: 0.2
                speed_mm_s: 30.0
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, relative))
    action = select_actions(
        tree, TreeSelection(action_id="relative_1")
    )[0]

    assert action.parameters["offsets"] == {
        "left": [0.02, 0.0, 0.0],
    }
    assert action.parameters["orientations"] == {
        "left": [None, 0.2, None],
    }


def test_hand_q_matches_reference_dual_hand_order(tmp_path: Path) -> None:
    hand_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: hand_1
                type: hand
                q: [255, 254, 253, 252, 251, 250,
                    69, 70, 71, 72, 73, 74]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, hand_tree))
    action = select_actions(
        tree, TreeSelection(action_id="hand_1")
    )[0]

    assert action.parameters["requests"] == {
        "left": {
            "command": "motors",
            "values": [255, 254, 253, 252, 251, 250],
        },
        "right": {
            "command": "motors",
            "values": [69, 70, 71, 72, 73, 74],
        },
    }


def test_hand_allows_single_side_and_presets(tmp_path: Path) -> None:
    hand_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: hand_left
                type: hand
                command: motors
                targets:
                  left: [255, 160, 69, 70, 71, 72]
              - id: hand_open_right
                type: hand
                command: open
                hands: [right]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, hand_tree))
    left, right = select_actions(
        tree, TreeSelection(behavior_id="behavior_1")
    )[1:3]

    assert left.parameters["requests"] == {
        "left": {
            "command": "motors",
            "values": [255, 160, 69, 70, 71, 72],
        }
    }
    assert right.parameters["requests"] == {
        "right": {"command": "open", "values": [0] * 6}
    }


@pytest.mark.parametrize(
    "hand_yaml, message",
    [
        (
            """\
              - id: bad_hand
                type: hand
                q: [0, 1, 2]
""",
            "exactly 12",
        ),
        (
            """\
              - id: bad_hand
                type: hand
                command: motors
                targets:
                  left: [0, 1, 2, 3, 4, 256]
""",
            r"integer in \[0, 255\]",
        ),
    ],
)
def test_hand_rejects_invalid_protocol_values(
    tmp_path: Path, hand_yaml: str, message: str
) -> None:
    hand_tree = TEMPLATE.replace(
        "              - id: wait_1\n", hand_yaml + "              - id: wait_1\n"
    )

    with pytest.raises(BehaviorTreeError, match=message):
        load_behavior_tree(write_tree(tmp_path, hand_tree))


def test_complete_example_validates_all_visual_motion_modes() -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "behavior_trees"
        / "examples"
        / "全部动作示例.yaml"
    )

    tree = load_behavior_tree(example)
    modes = {
        action.parameters["motion_mode"]
        for action in tree.actions
        if action.action_type == "move_l_vision"
    }

    assert modes == {1, 2, 3, 4, 5, 6}


def test_visual_motion_rejects_waist_rotation(tmp_path: Path) -> None:
    visual_motion = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: visual_move
                type: move_l_vision
                motion_mode: 3
                base_key: target
                points: [center]
                relative_offset:
                  rotate_with_waist: true
                  left: {dx: 0.01}
                speed_mm_s: 20.0
              - id: wait_1
""",
    )

    with pytest.raises(BehaviorTreeError, match="waist"):
        load_behavior_tree(write_tree(tmp_path, visual_motion))
