"""Unit tests for concrete behavior implementations."""

from types import SimpleNamespace

from rokae_motion.bt_actions import execute_behavior


class FakeExecutor:
    def __init__(self) -> None:
        self.calls = []

    def move_absj(self, action_name, targets) -> None:
        self.calls.append(("move_absj", action_name, targets))

    def move_l(self, parameters) -> None:
        self.calls.append(("move_l", parameters))

    def move_l_relative(self, parameters) -> None:
        self.calls.append(("move_l_relative", parameters))

    def wait(self, duration_s) -> None:
        self.calls.append(("wait", duration_s))


def test_move_absj_behavior() -> None:
    executor = FakeExecutor()
    action = SimpleNamespace(
        action_type="move_absj",
        action_name="home",
        parameters={"targets": {"left": [0.0] * 7}},
    )

    execute_behavior(executor, action)

    assert executor.calls == [
        ("move_absj", "home", {"left": [0.0] * 7})
    ]


def test_move_l_and_wait_behaviors() -> None:
    executor = FakeExecutor()
    move_l = SimpleNamespace(
        action_type="move_l",
        parameters={"targets": {"right": {"pose": [0.0] * 6}}},
    )
    wait = SimpleNamespace(
        action_type="wait",
        parameters={"duration_s": 0.5},
    )

    execute_behavior(executor, move_l)
    execute_behavior(executor, wait)

    assert executor.calls == [
        ("move_l", move_l.parameters),
        ("wait", 0.5),
    ]


def test_relative_move_l_behavior() -> None:
    executor = FakeExecutor()
    action = SimpleNamespace(
        action_type="move_l_relative",
        parameters={
            "offsets": {
                "left": [0.0, 0.0, 0.02],
                "right": [0.0, 0.0, 0.02],
            }
        },
    )

    execute_behavior(executor, action)

    assert executor.calls == [
        ("move_l_relative", action.parameters)
    ]
