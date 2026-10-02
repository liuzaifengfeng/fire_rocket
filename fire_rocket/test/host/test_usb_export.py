import importlib.util
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[2] / "tools" / "usb_monitor.py"
spec = importlib.util.spec_from_file_location("usb_monitor", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Port:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.sent = b""

    def write(self, data):
        self.sent += data

    def readline(self):
        return next(self.lines, b"")


class ExportTests(unittest.TestCase):
    def test_success(self):
        with tempfile.TemporaryDirectory() as root:
            target = Path(root) / "data.csv"
            port = Port([b"BEGIN CSV\r\n", b"x,y\r\n", b"1,nan\r\n", b"END\r\n"])
            module.save_export(port, "CSV", target)
            self.assertEqual(port.sent, b"EXPORT CSV\n")
            self.assertEqual(target.read_text(), "x,y\n1,nan\n")
            self.assertFalse(target.with_name("data.csv.partial").exists())

    def test_error_keeps_partial(self):
        with tempfile.TemporaryDirectory() as root:
            target = Path(root) / "data.csv"
            port = Port([b"BEGIN CSV\n", b"x,y\n", b"ERR TRUNCATED_LOG\n"])
            with self.assertRaises(RuntimeError):
                module.save_export(port, "CSV", target)
            self.assertFalse(target.exists())
            self.assertTrue(target.with_name("data.csv.partial").exists())

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            target = Path(root) / "data.csv"
            target.write_text("previous")
            port = Port([])
            with self.assertRaises(FileExistsError):
                module.save_export(port, "CSV", target)
            self.assertEqual(target.read_text(), "previous")
            self.assertEqual(port.sent, b"")


if __name__ == "__main__":
    unittest.main()
