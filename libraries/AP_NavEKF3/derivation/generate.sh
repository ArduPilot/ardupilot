#!/usr/bin/env bash

cd "$(dirname "$0")"
rm -rf generated # ensure generated directory exists and is empty
mkdir -p generated

./generate_1.py
./generate_2.py
