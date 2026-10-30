"""Strict Vietnamese grammar: validate every endpoint before dispatch."""

import re
import unicodedata
from dataclasses import dataclass


def normalize(text):
    text = unicodedata.normalize("NFKC", text).lower().replace("đ", "d")
    text = "".join(
        c for c in unicodedata.normalize("NFD", text) if unicodedata.category(c) != "Mn"
    )
    return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9\s]", " ", text)).strip()


@dataclass(frozen=True)
class Command:
    normalized: str
    regions: tuple


class CommandParser:
    def __init__(self, config):
        self.aliases = {}
        for region in config["regions"]:
            for alias in [region["id"], region["name"], *region["aliases"]]:
                key = normalize(alias)
                if key in self.aliases and self.aliases[key] != region["id"]:
                    raise ValueError("Alias collision: " + key)
                self.aliases[key] = region["id"]

    def resolve(self, name):
        name = name.strip()
        if name not in self.aliases:
            # Spoken letters are accepted only inside a complete region phrase.
            short = re.sub(r"^(khu vuc|khu) ", "", name)
            if short not in self.aliases:
                raise ValueError("Khu vực không tồn tại: " + name)
            return self.aliases[short]
        return self.aliases[name]

    def parse(self, text):
        s = normalize(text)
        if len(s) > 240:
            raise ValueError("Câu lệnh quá dài")
        route = re.fullmatch(r"(?:di chuyen|di) tu (.+?) den (.+)", s)
        if route:
            return Command(s, tuple(self.resolve(x) for x in route.groups()))
        single = re.fullmatch(r"(?:di den|di chuyen den|den) (.+)", s)
        if single:
            return Command(s, (self.resolve(single.group(1)),))
        raise ValueError("Câu lệnh chưa đúng: nói Đi đến khu vực A/B/C")
