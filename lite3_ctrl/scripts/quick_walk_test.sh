#!/bin/bash
# Quick test script for Lite3 robot walking
# Usage: ./quick_walk_test.sh [simple|full]

TEST_MODE=${1:-simple}

echo "========================================"
echo "Lite3 Walking Test - Quick Start"
echo "========================================"
echo ""
echo "Test Mode: $TEST_MODE"
echo ""
echo "⚠️  SAFETY CHECKLIST:"
echo "  [ ] Robot powered on"
echo "  [ ] Clear space (3m+ clearance)"
echo "  [ ] Robot in AUTO mode (via app)"
echo "  [ ] Emergency stop ready"
echo ""
read -p "Press Enter when ready (Ctrl+C to cancel)..."

echo ""
echo "Starting test in 3 seconds..."
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1
echo "GO!"

# Launch the test
roslaunch lite3_ctrl lite3_walk_test_standalone.launch test_mode:=$TEST_MODE
