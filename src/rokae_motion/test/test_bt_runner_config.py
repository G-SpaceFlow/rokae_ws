"""Unit tests for shared runner execution policy."""

from argparse import ArgumentParser
from pathlib import Path

import rokae_motion.bt_executor as bt_executor_module
from rokae_motion.bt_runner_config import (
    RunnerConfig,
    add_common_runner_args,
    build_config_from_args,
)
from rokae_motion.bt_executor import (
    ActionSpec,
    BehaviorTree,
    BehaviorTreeExecutor,
)


def test_runner_defaults_to_safe_dry_run() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser)

    config = build_config_from_args(parser.parse_args([]))

    assert not config.execute
    assert not config.initialize_before_execute
    assert not config.require_confirmation
    assert config.confirmation_phrase == "EXECUTE"
    assert not config.continue_on_action_fail
    assert not config.skip_navigation


def test_runner_builds_explicit_execution_policy() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser)

    config = build_config_from_args(
        parser.parse_args(
            ["--execute", "--continue-on-action-fail", "--skip-nav"]
        )
    )

    assert config.execute
    assert config.continue_on_action_fail
    assert config.skip_navigation


def test_control_defaults_to_execution_and_supports_dry_run() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser, execute_by_default=True)

    execute_config = build_config_from_args(
        parser.parse_args([]), initialize_before_execute=True
    )
    dry_run_config = build_config_from_args(
        parser.parse_args(["--dry-run"]),
        initialize_before_execute=True,
    )

    assert execute_config.execute
    assert execute_config.initialize_before_execute
    assert not dry_run_config.execute
    assert dry_run_config.initialize_before_execute


def test_disabled_confirmation_does_not_prompt(
    monkeypatch, capsys
) -> None:
    def fail_if_called(prompt):
        raise AssertionError(f"unexpected input prompt: {prompt}")

    monkeypatch.setattr("builtins.input", fail_if_called)
    executor = BehaviorTreeExecutor(
        RunnerConfig(execute=True, require_confirmation=False)
    )

    assert executor._confirm_execution(3)
    assert capsys.readouterr().out == ""


def test_control_initializes_before_first_action(monkeypatch) -> None:
    events = []

    class FakeLogger:
        def warning(self, message):
            pass

        def info(self, message):
            pass

        def error(self, message):
            pass

    class FakeNode:
        def get_logger(self):
            return FakeLogger()

        def initialize_robots(self):
            events.append("initialize")

        def cancel_active(self):
            events.append("cancel")

        def destroy_node(self):
            events.append("destroy")

    action = ActionSpec(
        task_id="task",
        task_name="task",
        location_id="location",
        location_name="location",
        behavior_id="behavior",
        behavior_name="behavior",
        action_id="action",
        action_name="wait",
        action_type="wait",
        enabled=True,
        optional=False,
        parameters={"duration_s": 0.0},
    )
    monkeypatch.setattr(
        bt_executor_module, "RokaeActionExecutor", FakeNode
    )
    monkeypatch.setattr(
        bt_executor_module,
        "execute_behavior",
        lambda node, selected: events.append("action"),
    )
    monkeypatch.setattr(bt_executor_module.rclpy, "init", lambda: None)
    monkeypatch.setattr(bt_executor_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        bt_executor_module.rclpy, "shutdown", lambda: None
    )

    executor = BehaviorTreeExecutor(
        RunnerConfig(execute=True, initialize_before_execute=True)
    )

    assert executor._run_actions([action]) == 0
    assert events == ["initialize", "action", "destroy"]


def test_control_dry_run_does_not_initialize(monkeypatch) -> None:
    action = ActionSpec(
        task_id="task",
        task_name="task",
        location_id="location",
        location_name="location",
        behavior_id="behavior",
        behavior_name="behavior",
        action_id="action",
        action_name="wait",
        action_type="wait",
        enabled=True,
        optional=False,
        parameters={"duration_s": 0.0},
    )
    tree = BehaviorTree(
        source=Path("tree.yaml"),
        execution_locked=False,
        actions=[action],
    )
    executor = BehaviorTreeExecutor(
        RunnerConfig(execute=False, initialize_before_execute=True)
    )
    monkeypatch.setattr(
        executor,
        "_run_actions",
        lambda actions: (_ for _ in ()).throw(
            AssertionError("dry-run must not initialize or execute")
        ),
    )

    assert executor.execute_actions(tree, [action]) == 0


def test_skip_nav_removes_navigation_before_execution(
    monkeypatch, capsys
) -> None:
    navigation = ActionSpec(
        task_id="task",
        task_name="task",
        location_id="location",
        location_name="location",
        behavior_id="behavior",
        behavior_name="behavior",
        action_id="navigate",
        action_name="navigate",
        action_type="navigate",
        enabled=True,
        optional=False,
        parameters={"command": "LM2"},
    )
    wait = ActionSpec(
        task_id="task",
        task_name="task",
        location_id="location",
        location_name="location",
        behavior_id="behavior",
        behavior_name="behavior",
        action_id="wait",
        action_name="wait",
        action_type="wait",
        enabled=True,
        optional=False,
        parameters={"duration_s": 0.0},
    )
    tree = BehaviorTree(
        source=Path("tree.yaml"),
        execution_locked=False,
        actions=[navigation, wait],
    )
    executor = BehaviorTreeExecutor(
        RunnerConfig(execute=True, skip_navigation=True)
    )
    executed = []
    monkeypatch.setattr(
        executor,
        "_run_actions",
        lambda actions: executed.extend(actions) or 0,
    )

    assert executor.execute_actions(tree, [navigation, wait]) == 0
    assert executed == [wait]
    assert "Navigation skipped by --skip-nav" in capsys.readouterr().out
