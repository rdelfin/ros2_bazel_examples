from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _shutdown_on_nonzero_exit():
    # Global handler: only shut down if a process exits with non-zero status.
    def _maybe_shutdown(event, context):
        rc = getattr(event, "returncode", None)
        if rc is None or rc != 0:
            reason = f"Process '{getattr(event.action, 'name', 'unknown')}' exited with {rc}; shutting down."
            return [Shutdown(reason=reason)]
        return []

    return RegisterEventHandler(OnProcessExit(on_exit=_maybe_shutdown))


def generate_launch_description():
    return LaunchDescription([
        Node(
            name="talker_node",
            executable="nodes/talker",
        ),
        Node(
            name="listener_node",
            executable="nodes/listener",
        ),
        _shutdown_on_nonzero_exit(),
    ])
