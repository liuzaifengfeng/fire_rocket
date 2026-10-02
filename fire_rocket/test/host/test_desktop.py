from pathlib import Path
import queue
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from desktop.model import MonitorState, key_values
from desktop.transport import SerialWorker


class ModelTests(unittest.TestCase):
    def test_legacy_no_config(self):
        model=MonitorState(connected=True)
        model.ingest("STATUS mode=MONITOR_ONLY source=LIVE fresh=0 zeroed=0 recording=0 storage=NEED FORMAT",1)
        model.ingest("SENSOR config=0 fields=00 rate_code=0 reporting=0 samples=0 checksum_errors=0 framing_errors=0 data_errors=0 missing_config=0",1)
        model.ingest("HEIGHT INVALID",1)
        model.ingest("TILT INVALID",1)
        self.assertEqual(model.status["storage"],"NEED FORMAT")
        self.assertIn("配置",model.diagnosis(1)[0])
        self.assertIsNone(model.displayed("height",1))
        self.assertIsNone(model.displayed("tilt",1))

    def test_height_zero_does_not_gate_tilt(self):
        model=MonitorState(connected=True)
        model.ingest("STATUS fresh=1 zeroed=0",1)
        model.ingest("HEIGHT INVALID",1)
        model.ingest("TILT 12.5 deg",1)
        self.assertIsNone(model.displayed("height",1))
        self.assertEqual(model.displayed("tilt",1),12.5)
        model.ingest("RAW altitude_m=102.5 ax=NA pressure_pa=nan",1)
        self.assertEqual(model.displayed("altitude_m",1),102.5)
        self.assertIsNone(model.displayed("ax",1))
        self.assertIsNone(model.displayed("pressure_pa",1))

    def test_disconnect_and_stale_invalidate_all_cards(self):
        model=MonitorState(connected=True)
        model.ingest("STATUS fresh=1 zeroed=1",1)
        model.ingest("HEIGHT 1.2 m",1)
        model.ingest("TILT 4 deg",1)
        self.assertEqual(model.displayed("height",1),1.2)
        self.assertIsNone(model.displayed("height",5))
        self.assertIsNone(model.displayed("tilt",5))
        model.reset()
        self.assertIsNone(model.last_status)
        self.assertIsNone(model.displayed("tilt",1))

    def test_no_uart_rx_v2(self):
        model=MonitorState(connected=True)
        model.ingest("STATUS fresh=0 zeroed=0",1)
        model.ingest("SENSOR config=0",1)
        model.ingest("DIAG firmware=2 rx_bytes=0 height_reason=NO_RX tilt_reason=NO_RX",1)
        self.assertIn("未收到 AS201",model.diagnosis(1)[0])

    def test_received_but_bad_config(self):
        model=MonitorState(connected=True)
        model.ingest("STATUS fresh=0",1)
        model.ingest("SENSOR config=0",1)
        model.ingest("DIAG rx_bytes=128",1)
        self.assertIn("UART 有数据",model.diagnosis(1)[0])

    def test_config_readback_and_space_values(self):
        self.assertEqual(key_values("fresh=1 storage=TIME LIMIT source=LIVE")["storage"],"TIME LIMIT")
        model=MonitorState()
        model.ingest("CFG stale_ms=500 zero_span_m=0.500",1)
        self.assertEqual(model.config["stale_ms"],"500")


class FakePort:
    def __init__(self, chunks):
        self.chunks=iter(chunks)
    def readline(self):
        return next(self.chunks,b"")


class TransportTests(unittest.TestCase):
    def test_fragmented_line_is_not_parsed_early(self):
        worker=SerialWorker("TEST",queue.Queue())
        worker.port=FakePort([b"STAT",b"US fresh=",b"0\r\n"])
        self.assertEqual(worker.readline(),b"")
        self.assertEqual(worker.readline(),b"")
        self.assertEqual(worker.readline(),b"STATUS fresh=0\r\n")

    def test_disconnect_interrupts_export(self):
        worker=SerialWorker("TEST",queue.Queue())
        worker.stopping.set()
        with self.assertRaises(InterruptedError):
            worker.readline()


if __name__=="__main__":
    unittest.main()
