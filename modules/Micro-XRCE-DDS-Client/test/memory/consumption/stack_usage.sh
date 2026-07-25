#!/bin/sh
pwd

# Run Valgrind with Massif tool.
valgrind --tool=massif --stacks=yes --detailed-freq=1 --max-snapshots=1000 --massif-out-file=./test/memory/consumption.out ./test/memory/consumption/consumption_test

# Run python script.
python3 ../test/memory/consumption/stack_analysis.py ./test/memory/consumption.out 
