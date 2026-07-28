"""Shared command-line execution and confirmation configuration."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RunnerConfig:
    """Runtime policy applied by the behavior-tree executor."""

    execute: bool = False
    initialize_before_execute: bool = False
    continue_on_action_fail: bool = False
    skip_navigation: bool = False
    require_confirmation: bool = False
    confirmation_phrase: str = "EXECUTE"


def add_common_runner_args(
    parser, *, execute_by_default: bool = False
) -> None:
    """Add execution policy options shared by behavior-tree entry points."""
    if execute_by_default:
        parser.add_argument(
            "--dry-run",
            action="store_true",
            help="preview the workflow without sending robot commands",
        )
    else:
        parser.add_argument(
            "--execute",
            action="store_true",
            help="send robot commands; default behavior is dry-run only",
        )
    parser.add_argument(
        "--continue-on-action-fail",
        action="store_true",
        help=(
            "continue after a required action fails; optional actions "
            "always continue"
        ),
    )
    parser.add_argument(
        "--skip-nav",
        action="store_true",
        help="skip enabled navigate actions without sending chassis commands",
    )


def build_config_from_args(
    arguments, *, initialize_before_execute: bool = False
) -> RunnerConfig:
    """Build an immutable execution policy from parsed CLI arguments."""
    if hasattr(arguments, "dry_run"):
        execute = not bool(arguments.dry_run)
    else:
        execute = bool(getattr(arguments, "execute", False))
    return RunnerConfig(
        execute=execute,
        initialize_before_execute=initialize_before_execute,
        continue_on_action_fail=bool(
            getattr(arguments, "continue_on_action_fail", False)
        ),
        skip_navigation=bool(
            getattr(arguments, "skip_nav", False)
        ),
    )
