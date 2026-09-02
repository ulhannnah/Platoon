"""Small serial helpers that keep ROS executor callbacks bounded."""
from typing import Tuple, Type


def try_serial_write(port, payload: bytes, serial_errors: Tuple[Type[BaseException], ...]) -> bool:
    """Write once; convert configured serial exceptions into a False result."""
    try:
        port.write(payload)
        return True
    except serial_errors:
        return False
