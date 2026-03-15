"""
conftest.py — pytest configuration for ATLAS test suite
Sets up sys.path so tests can import from src/python without install.
"""
import sys
from pathlib import Path

# Add src/python to path for all tests
sys.path.insert(0, str(Path(__file__).parent.parent / "src/python"))
