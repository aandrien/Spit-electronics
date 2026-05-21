#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
sleep 5
python3 "$SCRIPT_DIR/../Spit_RPi_Arduino.py"
