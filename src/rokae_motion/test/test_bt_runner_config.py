"""Unit tests for shared runner execution policy."""

from argparse import ArgumentParser

from rokae_motion.bt_runner_config import (
    RunnerConfig,
    add_common_runner_args,
    build_config_from_args,
)
from rokae_motion.bt_executor import BehaviorTreeExecutor


def test_runner_defaults_to_safe_dry_run() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser)

    config = build_config_from_args(parser.parse_args([]))

    assert not config.execute
    assert not config.require_confirmation
    assert config.confirmation_phrase == "EXECUTE"
    assert not config.continue_on_action_fail


def test_runner_builds_explicit_execution_policy() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser)

    config = build_config_from_args(
        parser.parse_args(["--execute", "--continue-on-action-fail"])
    )

    assert config.execute
    assert config.continue_on_action_fail


def test_control_defaults_to_execution_and_supports_dry_run() -> None:
    parser = ArgumentParser()
    add_common_runner_args(parser, execute_by_default=True)

    assert build_config_from_args(parser.parse_args([])).execute
    assert not build_config_from_args(
        parser.parse_args(["--dry-run"])
    ).execute


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
