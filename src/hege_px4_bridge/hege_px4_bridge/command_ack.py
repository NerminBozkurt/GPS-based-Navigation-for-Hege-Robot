"""Thread-safe matching and waiting for one PX4 VehicleCommandAck.

This module deliberately has no ROS imports so its race, timeout and result
handling can be unit-tested outside a ROS installation.
"""

from __future__ import annotations

import threading
import time


class CommandAckWaiter:
    """Track one command and wake its service callback on a final ACK."""

    def __init__(self, in_progress_result: int) -> None:
        self._in_progress_result = int(in_progress_result)
        self._condition = threading.Condition()
        self._command: int | None = None
        self._result: int | None = None
        self._done = False

    def begin(self, command: int) -> None:
        """Start waiting. Call this before publishing to avoid a fast-ACK race."""
        with self._condition:
            if self._command is not None:
                raise RuntimeError("another PX4 command acknowledgement is pending")
            self._command = int(command)
            self._result = None
            self._done = False

    def update(self, command: int, result: int) -> bool:
        """Accept a matching ACK; IN_PROGRESS is recorded but is not final."""
        with self._condition:
            if self._command != int(command):
                return False
            self._result = int(result)
            if self._result != self._in_progress_result:
                self._done = True
                self._condition.notify_all()
            return True

    def wait(self, timeout: float) -> int | None:
        """Return the final result, or None after a monotonic timeout."""
        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while not self._done:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self._clear_locked()
                    return None
                self._condition.wait(remaining)
            result = self._result
            self._clear_locked()
            return result

    def cancel(self) -> None:
        with self._condition:
            self._clear_locked()
            self._condition.notify_all()

    @property
    def pending_command(self) -> int | None:
        with self._condition:
            return self._command

    def _clear_locked(self) -> None:
        self._command = None
        self._result = None
        self._done = False
