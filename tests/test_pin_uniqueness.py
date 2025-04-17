#!/usr/bin/env python3
"""
Unit test to ensure that GPIO pin assignments for OUT1 through OUT8 are unique.
Run this test with: python3 tests/test_pin_uniqueness.py
"""
import re
import sys

def main():
    source_file = "src/main.cpp"
    try:
        with open(source_file, "r") as f:
            content = f.read()
    except IOError as e:
        print(f"ERROR: Could not read {source_file}: {e}")
        sys.exit(1)

    # Regex to capture OUT pins: const int OUT<number> = <pin>;
    pattern = re.compile(r'const int OUT([1-8])\s*=\s*([0-9]+)\s*;')
    matches = pattern.findall(content)
    if not matches:
        print("ERROR: No OUT pin definitions found in src/main.cpp")
        sys.exit(1)

    pins = {}
    errors = False
    for idx, pin in matches:
        pin_num = int(pin)
        name = f"OUT{idx}"
        if pin_num in pins:
            print(f"Duplicate pin assignment: {name} and {pins[pin_num]} both use GPIO {pin_num}")
            errors = True
        else:
            pins[pin_num] = name

    if errors:
        sys.exit(1)
    else:
        print("OK: All OUT pin assignments are unique.")
        sys.exit(0)

if __name__ == "__main__":
    main()