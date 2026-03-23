#!/usr/bin/env python3

import pickle
import socket
import time
from typing import Any, Dict, List


HOST = "127.0.0.1"
PORT = 50000

LEG_ORDER = ("fl", "bl", "fr", "br")
LegPayload = Dict[str, List[float]]


def validate_leg_payload(payload: Any) -> LegPayload:
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict")

    normalized: LegPayload = {}
    for leg in LEG_ORDER:
        if leg not in payload:
            raise ValueError(f"missing leg '{leg}'")

        values = payload[leg]
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"payload['{leg}'] must be a list or tuple")

        if len(values) != 3:
            raise ValueError(f"payload['{leg}'] must have exactly 3 values")

        normalized[leg] = [float(v) for v in values]

    return normalized


def send_leg_angles(payload: Any, host: str = HOST, port: int = PORT) -> None:
    normalized = validate_leg_payload(payload)
    data = pickle.dumps(normalized, protocol=pickle.HIGHEST_PROTOCOL)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        sock.sendall(data)


def main() -> None:
    payload: LegPayload = {
        "fl": [60.0, -105.0, 6.0], # 1, 2, 3
        "bl": [60.0, -105.0, 6.0], # 4, 5, 6
        "fr": [60.0, -105.0, 6.0], # 7, 8, 9
        "br": [60.0, -105.0, 6.0], # 10, 11, 12
    }  

    send_leg_angles(payload)
    print("Sent grouped leg payload:")
    print(payload)


if __name__ == "__main__":
    main()
