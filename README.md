# TeslaBMSV2 - ESP32

This software is designed to run on the SimpBMS PCB

Runs on ESP32 (tested with Lolin32 Lite)
# Testing

A simple unit test is provided to ensure that the GPIO pin assignments for `OUT1` through `OUT8` remain unique.  
Requires Python 3. Run from the project root:

    python3 tests/test_pin_uniqueness.py