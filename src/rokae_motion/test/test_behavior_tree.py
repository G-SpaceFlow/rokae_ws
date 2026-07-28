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


def test_location_navigation_is_normalized(tmp_path: Path) -> None:
    navigation = TEMPLATE.replace(
        "      - id: station_1\n",
        """\
      - id: station_1
        command: lm2
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, navigation))
    action = select_actions(
        tree, TreeSelection(action_id="nav_pose")
    )[0]

    assert action.location_navigation
    assert action.parameters == {
        "command": "LM2",
        "station": "LM2",
        "arrival_state": "ARRIVE_A",
        "connection_timeout_s": 5.0,
        "response_timeout_s": 180.0,
    }


def test_location_navigation_precedes_selected_behavior_action(
    tmp_path: Path,
) -> None:
    navigation = TEMPLATE.replace(
        "      - id: station_1\n",
        """\
      - id: station_1
        command: LM1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, navigation))
    selected = select_actions(
        tree, TreeSelection(action_id="wait_1")
    )

    assert [action.action_id for action in selected] == [
        "nav_pose",
        "wait_1",
    ]


def test_location_navigation_rejects_unknown_command(
    tmp_path: Path,
) -> None:
    navigation = TEMPLATE.replace(
        "      - id: station_1\n",
        """\
      - id: station_1
        command: somewhere
""",
    )

    with pytest.raises(
        BehaviorTreeError, match="unsupported navigation command"
    ):
        load_behavior_tree(write_tree(tmp_path, navigation))


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
        "              - id: wait_1\n",
        hand_yaml + "              - id: wait_1\n",
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


def test_aruco_vision_action_uses_one_tool_pose(
    tmp_path: Path,
) -> None:
    aruco_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: detect_tool
                type: vision
                source: aruco
                key: tool_pose
                trigger_value: 1
                points: [tool]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, aruco_tree))
    action = next(
        item for item in tree.actions
        if item.action_id == "detect_tool"
    )

    assert action.parameters == {
        "source": "aruco",
        "echo_topic": "/tool/pose",
        "pub_topic": "/aruco/enable",
        "key": "tool_pose",
        "trigger_value": 1,
        "labels": [],
        "point_names": ["tool"],
        "response_timeout_s": 10.0,
    }


def test_aruco_vision_rejects_xyz_as_point_names(
    tmp_path: Path,
) -> None:
    aruco_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: detect_tool
                type: vision
                source: aruco
                key: tool_pose
                trigger_value: 1
                points: [x, y, z]
              - id: wait_1
""",
    )

    with pytest.raises(
        BehaviorTreeError,
        match="points must contain exactly one point name",
    ):
        load_behavior_tree(write_tree(tmp_path, aruco_tree))


def test_aruco_vision_accepts_topics_from_tree(
    tmp_path: Path,
) -> None:
    aruco_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: detect_tool
                type: vision
                source: aruco
                echo_topic: /custom/tool_pose
                pub_topic: /custom/aruco_enable
                key: tool_pose
                trigger_value: 1
                points: [tool]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, aruco_tree))
    action = next(
        item for item in tree.actions
        if item.action_id == "detect_tool"
    )

    assert action.parameters["echo_topic"] == "/custom/tool_pose"
    assert action.parameters["pub_topic"] == "/custom/aruco_enable"


def test_box_vision_action_uses_configured_topics_and_two_points(
    tmp_path: Path,
) -> None:
    box_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: detect_box
                type: vision
                source: box_grab_points
                echo_topic: /box_grab_points
                pub_topic: /box/enable
                key: box_pose
                trigger_value: 1
                points: [left_center, right_center]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(write_tree(tmp_path, box_tree))
    action = next(
        item for item in tree.actions
        if item.action_id == "detect_box"
    )

    assert action.parameters == {
        "source": "box_grab_points",
        "echo_topic": "/box_grab_points",
        "pub_topic": "/box/enable",
        "key": "box_pose",
        "trigger_value": 1,
        "labels": [],
        "point_names": ["left_center", "right_center"],
        "response_timeout_s": 10.0,
    }


def test_small_box_vision_action_uses_one_center_point(
    tmp_path: Path,
) -> None:
    small_box_tree = TEMPLATE.replace(
        "              - id: wait_1\n",
        """\
              - id: detect_small_box
                type: vision
                source: small_box_target
                echo_topic: /small_box/target
                pub_topic: /small_box/enable
                key: small_box_pose
                trigger_value: 1
                points: [center]
              - id: wait_1
""",
    )

    tree = load_behavior_tree(
        write_tree(tmp_path, small_box_tree)
    )
    action = next(
        item for item in tree.actions
        if item.action_id == "detect_small_box"
    )

    assert action.parameters == {
        "source": "small_box_target",
        "echo_topic": "/small_box/target",
        "pub_topic": "/small_box/enable",
        "key": "small_box_pose",
        "trigger_value": 1,
        "labels": [],
        "point_names": ["center"],
        "response_timeout_s": 10.0,
    }


def test_exhibition_visual_move_uses_cached_box_points() -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "behavior_trees"
        / "examples"
        / "展会动作.yaml"
    )

    tree = load_behavior_tree(example)
    action = next(
        item for item in tree.actions
        if item.action_id == "move_to_box_grab_points"
    )

    assert action.action_type == "move_l_vision"
    assert action.parameters["source"] == {
        "key": "1-1-1-1",
        "points": ["left_center", "right_center"],
    }
    assert action.parameters["offsets"] == {
        "left": [0.0, 0.1, 0.01],
        "right": [0.0, -0.15, 0.01],
    }
    assert action.parameters["orientations"] == {
        "left": [2.606341, -0.752082, 0.461936],
        "right": [-2.581585, -0.830764, -0.407993],
    }
    assert action.parameters["speed_mm_s"] == 30.0


def test_exhibition_small_box_actions_share_cached_center() -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "behavior_trees"
        / "examples"
        / "展会动作.yaml"
    )

    tree = load_behavior_tree(example)
    detect = next(
        item for item in tree.actions
        if item.action_id == "detect_small_box"
    )
    move = next(
        item for item in tree.actions
        if item.action_id == "move_above_small_box"
    )

    assert detect.parameters["source"] == "small_box_target"
    assert detect.parameters["echo_topic"] == "/small_box/target"
    assert detect.parameters["pub_topic"] == "/small_box/enable"
    assert detect.parameters["key"] == "1-1-1-2"
    assert detect.parameters["point_names"] == ["center"]
    assert move.parameters["source"] == {
        "key": "1-1-1-2",
        "points": ["center"],
    }


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
