"""Persistent application files live beside the EXE, never in its bundled runtime."""
from pathlib import Path
import sys


def application_dir(source_file):
    if getattr(sys, 'frozen', False):
        return Path(sys.executable).resolve().parent
    return Path(source_file).resolve().parent
