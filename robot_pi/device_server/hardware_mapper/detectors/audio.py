"""
detectors/audio.py

Audio Detector (device_server_hardware_mapper.txt, section 17).
Enumerates ALSA devices via /proc/asound rather than depending on a
Python audio library just for discovery.
"""

import os
from typing import Any, Dict, List


def _list_alsa_cards() -> List[Dict[str, Any]]:
    cards = []
    path = "/proc/asound/cards"
    if not os.path.isfile(path):
        return cards
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if line and line[0].isdigit():
                    parts = line.split(None, 2)
                    if len(parts) >= 3:
                        cards.append({"index": parts[0], "name": parts[2].strip("[]: ")})
    except (PermissionError, OSError):
        pass
    return cards


def detect() -> Dict[str, Any]:
    try:
        cards = _list_alsa_cards()
        if cards:
            return {
                "available": True,
                "audio": {
                    "input": cards,
                    "output": cards,
                },
            }

        # On Windows / macOS / desktop systems without ALSA
        import platform
        if platform.system().lower() in ("windows", "darwin"):
            return {
                "available": True,
                "audio": {
                    "input": [{"index": 0, "name": "Default Microphone"}],
                    "output": [{"index": 0, "name": "Default Speakers / Output"}],
                },
            }

        return {
            "available": False,
            "audio": {
                "input": [],
                "output": [],
            },
        }
    except Exception as exc:  # noqa: BLE001
        return {"available": False, "error": str(exc)}
