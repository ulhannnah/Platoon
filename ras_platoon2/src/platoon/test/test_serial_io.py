import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = ROOT / "platoon" if (ROOT / "platoon").is_dir() else ROOT
package = types.ModuleType("platoon")
package.__path__ = [str(MODULE_ROOT)]
sys.modules.setdefault("platoon", package)

from platoon.serial_io import try_serial_write


class SerialWriteTest(unittest.TestCase):
    def test_timeout_returns_false_instead_of_blocking_executor(self):
        class WriteTimeout(Exception):
            pass

        class Port:
            def write(self, payload):
                raise WriteTimeout("blocked ESP32 write")

        self.assertFalse(try_serial_write(Port(), b"status\n", (WriteTimeout,)))

    def test_successful_write_returns_true(self):
        class Port:
            def __init__(self):
                self.payload = None

            def write(self, payload):
                self.payload = payload

        port = Port()
        self.assertTrue(try_serial_write(port, b"status\n", (Exception,)))
        self.assertEqual(port.payload, b"status\n")


if __name__ == "__main__":
    unittest.main()
