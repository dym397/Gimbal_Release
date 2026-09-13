#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility launcher for the organized runtime source tree.

The real main program now lives at core/main_tracking_v9.py. Keeping this
thin wrapper preserves existing commands and systemd services that run:

    python3 main_tracking_v9.py
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
CORE_DIR = PROJECT_ROOT / "core"

sys.path.insert(0, str(CORE_DIR))
sys.path.insert(0, str(PROJECT_ROOT))
runpy.run_path(str(CORE_DIR / "main_tracking_v9.py"), run_name="__main__")
