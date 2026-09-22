"""Deterministic tests for the command ACK waiter."""

import threading
import time

from hege_px4_bridge.command_ack import CommandAckWaiter

IN_PROGRESS = 5
ACCEPTED = 0
DENIED = 2


def test_ack_already_received_before_wait():
    waiter = CommandAckWaiter(IN_PROGRESS)
    waiter.begin(400)
    assert waiter.update(400, ACCEPTED)
    assert waiter.wait(0.1) == ACCEPTED
    assert waiter.pending_command is None


def test_wrong_command_is_ignored():
    waiter = CommandAckWaiter(IN_PROGRESS)
    waiter.begin(400)
    assert not waiter.update(176, ACCEPTED)
    assert waiter.update(400, DENIED)
    assert waiter.wait(0.1) == DENIED


def test_in_progress_waits_for_final_result():
    waiter = CommandAckWaiter(IN_PROGRESS)
    waiter.begin(176)
    assert waiter.update(176, IN_PROGRESS)

    def finish():
        time.sleep(0.02)
        waiter.update(176, ACCEPTED)

    thread = threading.Thread(target=finish)
    thread.start()
    assert waiter.wait(0.5) == ACCEPTED
    thread.join()


def test_timeout_returns_none_and_clears_pending():
    waiter = CommandAckWaiter(IN_PROGRESS)
    waiter.begin(400)
    assert waiter.wait(0.01) is None
    assert waiter.pending_command is None


def test_second_pending_command_is_rejected():
    waiter = CommandAckWaiter(IN_PROGRESS)
    waiter.begin(400)
    try:
        waiter.begin(176)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a second pending command must be rejected")
