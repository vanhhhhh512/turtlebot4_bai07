"""Safety-relevant text rejection and rotated coordinate conversion."""

from pathlib import Path
import unittest
import numpy as np
import yaml
from bai07_turtlebot4.command_parser import CommandParser
from bai07_turtlebot4.map_segmentation import pixel_to_map, map_to_pixel, load_map

ROOT = Path(__file__).resolve().parents[1]


class TestCommands(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((ROOT / "config/regions.yaml").read_text())
        self.parser = CommandParser(self.cfg)

    def test_destinations_and_aliases(self):
        for text, expected in [
            ("Đi đến khu vực A", ("A",)),
            ("ĐI ĐẾN KHU VỰC B!", ("B",)),
            ("đi đến khu vực xê", ("C",)),
            ("Di chuyển từ khu vực A đến khu vực B", ("A", "B")),
            ("đi đến phía đông", ("C",)),
        ]:
            with self.subTest(text=text):
                self.assertEqual(self.parser.parse(text).regions, expected)

    def test_reject_whole_invalid_route(self):
        for text in [
            "Đi đến khu vực Z",
            "Di chuyển từ khu vực A đến khu vực Z",
            "Di chuyển từ khu vực Z đến khu vực B",
            "Đi đến",
            "xin chào",
            "đi đến A và B",
            "đừng đi đến A",
            "đi đến khu vực bàn",
            "đi đến khu vực ab",
        ]:
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.parser.parse(text)

    def test_alias_collision(self):
        self.cfg["regions"][1]["aliases"].append("Á")
        with self.assertRaises(ValueError):
            CommandParser(self.cfg)

    def test_rotated_roundtrip(self):
        for angle in [0, 0.7, -1.2]:
            meta = {"origin": [-10, 3, angle], "resolution": 0.05}
            for u, v in [(0, 0), (400, 400), (200, 200), (75, 125)]:
                self.assertEqual(
                    map_to_pixel(*pixel_to_map(u, v, meta, 401), meta, 401), (u, v)
                )

    def test_region_mask_and_goals(self):
        meta, _, grid = load_map(ROOT / "config/map_navigation.yaml")
        labels = np.load(ROOT / "maps/region_labels.npy")
        self.assertTrue(np.all(grid[labels > 0] == 0))
        self.assertEqual(int((grid == -1).sum()), 42778)
        for reg in self.cfg["regions"]:
            u, v = map_to_pixel(reg["goal"]["x"], reg["goal"]["y"], meta, 401)
            self.assertEqual(labels[v, u], reg["label"])
            self.assertGreaterEqual(reg["clearance_m"], 0.55)


if __name__ == "__main__":
    unittest.main()
