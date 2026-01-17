#!/bin/bash
# Launch SITL and run state feedback evaluation

echo "======================================"
echo "ArduSub SITL State Feedback Evaluation"
echo "======================================"
echo ""

# Change to ArduSub directory
cd "$(dirname "$0")/../../ArduSub" || exit 1

# Clean old parameter files
echo "Cleaning old parameter files..."
rm -f eeprom.bin mav.parm* >/dev/null 2>&1

# Start SITL in background
echo "Starting SITL..."
../build/sitl/bin/ardusub \
    --model vectored \
    --speedup 1 \
    --defaults ../Tools/autotest/default_params/sub.parm \
    --sim-address=127.0.0.1 \
    -I0 >/tmp/ardusub_sitl.log 2>&1 &

SITL_PID=$!
echo "SITL started with PID $SITL_PID"

# Wait for SITL to initialize
echo "Waiting for SITL to initialize (15s)..."
sleep 15

# Check if SITL is still running
if ! kill -0 $SITL_PID 2>/dev/null; then
    echo "ERROR: SITL failed to start"
    echo "Check /tmp/ardusub_sitl.log for details"
    exit 1
fi

echo "SITL ready!"
echo ""

# Run evaluation
echo "Starting evaluation..."
cd ../Tools/scripts || exit 1
python3 evaluate_state_feedback.py --test-mode all

# Cleanup
echo ""
echo "Cleaning up..."
kill $SITL_PID 2>/dev/null
wait $SITL_PID 2>/dev/null

echo ""
echo "Evaluation complete!"
echo "Results saved to /tmp/state_feedback_comparison.png"
