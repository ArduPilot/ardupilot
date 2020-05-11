#!/bin/sh
DIR=cache/$1/$2
FILE=$DIR/$3.terrain

[ ! -d $DIR ] && mkdir -p $DIR

if [ ! -f "$FILE" ] && [ ! -f "$FILE.invalid" ]; then
    curl -H "Accept: application/vnd.quantized-mesh,application/octet-stream;q=0.9" http://assets.agi.com/stk-terrain/v1/tilesets/world/tiles/$1/$2/$3.terrain > $FILE.gz 2>/dev/null
    gunzip $FILE.gz 2>/dev/null || mv $FILE.gz $FILE.invalid
fi
