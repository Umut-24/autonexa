#!/usr/bin/env python3
"""
Main launcher for Custom Navigation GUI
Simplified entry point that calls the GUI module
"""

import sys
import os

# Add scripts directory to path so we can import custom_navigation
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Import from custom_navigation package
from custom_navigation.navigation_gui import main

if __name__ == '__main__':
    main()

