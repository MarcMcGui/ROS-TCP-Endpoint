#!/usr/bin/env python3

"""Minimal Unity-side emulator for exercising ROS2 action support."""

import json
import socket
import struct
import threading
import time
import uuid

from example_interfaces.action import Fibonacci
from rclpy.serialization import serialize_message, deserialize_message

from ros_tcp_endpoint.client import ClientThread


HOST = "127.0.0.1"
PORT = 10000
ACTION_NAME = "/unity_fibonacci"
ACTION_TYPE = "example_interfaces/Fibonacci"


def _recv_exact(sock, size):
    buffer = bytearray()
    while len(buffer) < size:
        chunk = sock.recv(size - len(buffer))
        if not chunk:
            raise ConnectionError("Socket closed while waiting for data")
        buffer.extend(chunk)
    return bytes(buffer)


def _serialize_command(command, payload):
    cmd_bytes = command.encode("utf-8")
    json_str = json.dumps(payload) + "\n"
    json_bytes = json_str.encode("utf-8")
    return (
        struct.pack("<I", len(cmd_bytes))
        + cmd_bytes
        + struct.pack("<I", len(json_bytes))
        + json_bytes
    )


def _read_loop(sock, runtime_state):
    try:
        while True:
            try:
                dest_len_bytes = sock.recv(4)
            except socket.timeout:
                continue
            if not dest_len_bytes:
                break
            dest_len = struct.unpack("<I", dest_len_bytes)[0]
            destination = _recv_exact(sock, dest_len).decode("utf-8")
            payload_len = struct.unpack("<I", _recv_exact(sock, 4))[0]
            payload = _recv_exact(sock, payload_len)
            if destination.startswith("__"):
                message = json.loads(payload.decode("utf-8"))
                print(f"[UNITY] Received {destination}: {message}")
                if destination == "__action_feedback":
                    runtime_state["pending_payload_type"] = "feedback"
                elif destination == "__action_result":
                    runtime_state["pending_payload_type"] = "result"
                    cancel_goal_id = runtime_state.get("cancel_goal_id")
                    if (
                        cancel_goal_id is not None
                        and message.get("goal_id") == cancel_goal_id
                    ):
                        runtime_state["cancel_event"].set()
                elif destination == "__action_goal_request":
                    runtime_state["pending_payload_type"] = "goal"
            else:
                pretty = _decode_ros_payload(
                    runtime_state.get("pending_payload_type"), payload
                )
                if pretty is not None:
                    print(f"[UNITY] Decoded {destination}: {pretty}")
                else:
                    print(f"[UNITY] Received {destination}: {payload[:80]!r}")
                runtime_state["pending_payload_type"] = None
    except ConnectionError:
        pass


def main():
    print("[UNITY] Connecting to ROS TCP Endpoint at %s:%s" % (HOST, PORT))
    runtime_state = {
        "cancel_goal_id": None,
        "cancel_event": threading.Event(),
        "pending_payload_type": None,
    }
    with socket.create_connection((HOST, PORT), timeout=5) as sock:
        sock.settimeout(1.0)
        reader = threading.Thread(
            target=_read_loop, args=(sock, runtime_state), daemon=True
        )
        reader.start()

        # Register ROS action client on the endpoint
        register_cmd = _serialize_command(
            "__ros_action", {"action_name": ACTION_NAME, "action_type": ACTION_TYPE}
        )
        sock.sendall(register_cmd)
        print("[UNITY] Sent action registration for", ACTION_NAME)
        time.sleep(1.0)

        # Send a Fibonacci goal
        goal_id = str(uuid.uuid4())
        goal_cmd = _serialize_command(
            "__action_goal", {"action_name": ACTION_NAME, "goal_id": goal_id}
        )
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 6
        goal_payload = ClientThread.serialize_message(ACTION_NAME, goal_msg)

        sock.sendall(goal_cmd)
        sock.sendall(goal_payload)
        print("[UNITY] Sent Fibonacci goal", goal_id)

        # Allow time for feedback/result loop to complete
        time.sleep(8.0)

        # Initiate a second goal then cancel it mid-flight
        cancel_goal_id = str(uuid.uuid4())
        cancel_goal_cmd = _serialize_command(
            "__action_goal", {"action_name": ACTION_NAME, "goal_id": cancel_goal_id}
        )
        cancel_goal_msg = Fibonacci.Goal()
        cancel_goal_msg.order = 20
        sock.sendall(cancel_goal_cmd)
        sock.sendall(ClientThread.serialize_message(ACTION_NAME, cancel_goal_msg))
        print("[UNITY] Sent long-running goal", cancel_goal_id)
        runtime_state["cancel_goal_id"] = cancel_goal_id
        runtime_state["cancel_event"].clear()

        time.sleep(3.0)
        cancel_cmd = _serialize_command(
            "__action_cancel", {"action_name": ACTION_NAME, "goal_id": cancel_goal_id}
        )
        sock.sendall(cancel_cmd)
        print("[UNITY] Requested cancel for", cancel_goal_id)

        runtime_state["cancel_event"].wait(timeout=20.0)


def _decode_ros_payload(payload_type, payload):
    if payload_type == "feedback":
        msg = deserialize_message(payload, Fibonacci.Feedback)
        return {"sequence": list(msg.sequence)}
    if payload_type == "result":
        msg = deserialize_message(payload, Fibonacci.Result)
        return {"sequence": list(msg.sequence)}
    if payload_type == "goal":
        msg = deserialize_message(payload, Fibonacci.Goal)
        return {"order": msg.order}
    return None


if __name__ == "__main__":
    main()
