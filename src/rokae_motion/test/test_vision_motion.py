"""Unit tests for the six ported visual motion modes."""

import pytest

from rokae_motion.vision_motion import (
    CartesianTarget,
    resolve_vision_motion,
)


def target(x, y, z, orientation=(None, None, None)):
    return CartesianTarget((x, y, z), orientation)


def make_fetch(values):
    def fetch(source, mode):
        value = values[(source["key"], source["points"][0])]
        if isinstance(value, tuple):
            return value
        return value, value

    return fetch


def test_motion_mode_1_uses_independent_visual_targets() -> None:
    left = target(1.0, 2.0, 3.0, (0.1, 0.2, 0.3))
    right = target(4.0, 5.0, 6.0, (-0.1, -0.2, -0.3))
    parameters = {
        "motion_mode": 1,
        "source": {"key": "pair", "points": ["left", "right"]},
    }

    result = resolve_vision_motion(
        parameters,
        {},
        lambda source, mode: (left, right),
    )

    assert result.targets == {"left": left, "right": right}


def test_motion_mode_2_moves_current_midpoint_to_visual_point() -> None:
    parameters = {
        "motion_mode": 2,
        "source": {"key": "pair", "points": ["center"]},
        "midpoint_offset": [0.0, 0.0, 0.2],
        "orientations": {},
    }
    current = {
        "left": target(0.0, 1.0, 0.0),
        "right": target(0.0, -1.0, 0.0),
    }

    result = resolve_vision_motion(
        parameters,
        current,
        make_fetch({("pair", "center"): target(1.0, 0.0, 0.0)}),
    )

    assert result.targets["left"].position == pytest.approx(
        (1.0, 1.0, 0.2)
    )
    assert result.targets["right"].position == pytest.approx(
        (1.0, -1.0, 0.2)
    )


def test_motion_mode_3_moves_only_configured_arm() -> None:
    parameters = {
        "motion_mode": 3,
        "source": {"key": "single", "points": ["center"]},
        "offsets": {"left": [0.1, 0.2, 0.3]},
        "orientations": {"left": [None, None, 0.5]},
    }

    result = resolve_vision_motion(
        parameters,
        {},
        make_fetch(
            {("single", "center"): target(1.0, 2.0, 3.0)}
        ),
    )

    assert set(result.targets) == {"left"}
    assert result.targets["left"].position == pytest.approx(
        (1.1, 2.2, 3.3)
    )
    assert result.targets["left"].orientation_rpy == (
        None,
        None,
        0.5,
    )


def test_motion_mode_4_applies_shared_delta_and_axis_override() -> None:
    parameters = {
        "motion_mode": 4,
        "point_sources": {
            "target": {"key": "delta", "points": ["target"]},
            "current": {"key": "delta", "points": ["current"]},
        },
        "delta_overrides": {
            "left": [None, None, 0.0],
            "right": [None, None, 0.0],
        },
        "orientations": {},
        "skip_threshold": None,
        "skip_axes": [0, 1],
    }
    current = {
        "left": target(1.0, 1.0, 1.0),
        "right": target(2.0, 2.0, 2.0),
    }
    fetch = make_fetch(
        {
            ("delta", "target"): target(0.1, 0.2, 0.3),
            ("delta", "current"): target(0.0, 0.0, 0.0),
        }
    )

    result = resolve_vision_motion(parameters, current, fetch)

    assert result.targets["left"].position == pytest.approx(
        (1.1, 1.2, 1.0)
    )
    assert result.targets["right"].position == pytest.approx(
        (2.1, 2.2, 2.0)
    )


def test_motion_mode_5_uses_independent_arm_deltas() -> None:
    parameters = {
        "motion_mode": 5,
        "point_sources": {
            name: {"key": "mode5", "points": [name]}
            for name in (
                "left_current",
                "right_current",
                "left_target",
                "right_target",
            )
        },
        "delta_overrides": {
            "left": [None, None, None],
            "right": [None, None, None],
        },
        "orientations": {},
        "skip_threshold": None,
        "skip_axes": [0, 1],
    }
    current = {
        "left": target(1.0, 1.0, 1.0),
        "right": target(2.0, 2.0, 2.0),
    }
    fetch = make_fetch(
        {
            ("mode5", "left_current"): target(0.0, 0.0, 0.0),
            ("mode5", "left_target"): target(0.1, 0.2, 0.3),
            ("mode5", "right_current"): target(1.0, 1.0, 1.0),
            ("mode5", "right_target"): target(0.8, 1.1, 1.4),
        }
    )

    result = resolve_vision_motion(parameters, current, fetch)

    assert result.targets["left"].position == pytest.approx(
        (1.1, 1.2, 1.3)
    )
    assert result.targets["right"].position == pytest.approx(
        (1.8, 2.1, 2.4)
    )


def test_motion_mode_6_applies_scalar_to_selected_axis_and_hands() -> None:
    parameters = {
        "motion_mode": 6,
        "scalar_source": {
            "key": "scalar",
            "points": ["y_offset"],
        },
        "initial_point": 0.004,
        "axis_index": 1,
        "hands": ["left", "right"],
        "offsets": {
            "left": [0.01, 0.0, 0.0],
            "right": [0.0, 0.0, 0.0],
        },
        "orientations": {},
        "skip_threshold": 0.005,
    }
    current = {
        "left": target(1.0, 1.0, 1.0),
        "right": target(2.0, 2.0, 2.0),
    }
    fetch = make_fetch(
        {("scalar", "y_offset"): target(0.010, 0.0, 0.0)}
    )

    result = resolve_vision_motion(parameters, current, fetch)

    assert result.targets["left"].position == pytest.approx(
        (1.01, 1.006, 1.0)
    )
    assert result.targets["right"].position == pytest.approx(
        (2.0, 2.006, 2.0)
    )
