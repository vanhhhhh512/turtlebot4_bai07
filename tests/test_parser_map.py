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
            ("Đi đến khu 1", ("1",)),
            ("ĐI ĐẾN KHU VỰC HAI!", ("2",)),
            ("đi đến khu ba", ("3",)),
            ("Di chuyển từ khu 1 đến khu 2", ("1", "2")),
            ("Di chuyển từ khu một đến khu ba", ("1", "3")),
            ("Đi từ khu vực số hai đến khu số 1", ("2", "1")),
            ("đi đến 3", ("3",)),
        ]:
            with self.subTest(text=text):
                self.assertEqual(self.parser.parse(text).regions, expected)

    def test_numeric_and_spoken_endpoints(self):
        forms = [
            (region["id"], phrase)
            for region in self.cfg["regions"]
            for phrase in region["aliases"]
        ]
        for region_id, phrase in forms:
            with self.subTest(phrase=phrase):
                self.assertEqual(
                    self.parser.parse("Đi đến " + phrase).regions, (region_id,)
                )
        for src_id, src in forms:
            for dst_id, dst in forms:
                self.assertEqual(
                    self.parser.parse(f"Di chuyển từ {src} đến {dst}").regions,
                    (src_id, dst_id),
                )

    def test_numeric_config_contract(self):
        seeds = yaml.safe_load((ROOT / "config/region_seeds.yaml").read_text())
        self.assertEqual([r["id"] for r in self.cfg["regions"]], ["1", "2", "3"])
        for actual, seed in zip(self.cfg["regions"], seeds["regions"]):
            for key in ["id", "name", "aliases"]:
                self.assertEqual(actual[key], seed[key])
            self.assertEqual(actual["name"], "Khu " + actual["id"])

    def test_reject_whole_invalid_route(self):
        for text in [
            "Đi đến khu 4",
            "Đi đến khu bốn",
            "Đi đến khu 0",
            "Đi đến khu 12",
            "Đi đến khu -1",
            "Di chuyển từ khu một đến khu bốn",
            "Di chuyển từ khu 4 đến khu 2",
            "Đi đến",
            "xin chào",
            "đi đến 1 và 2",
            "đừng đi đến khu 1",
            "đi đến khu vực bàn",
            "đi đến khu A",
            "đi đến khu bê",
            "đi đến khu xê",
        ]:
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.parser.parse(text)

    def test_alias_collision(self):
        self.cfg["regions"][1]["aliases"].append("một")
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
