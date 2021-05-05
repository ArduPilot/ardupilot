#!/bin/sh

set -e
set -x

REPORT_DIR="reports/lcov-report"
rm -rf "$REPORT_DIR"
mkdir -p "$REPORT_DIR"

INFO_FILE="$REPORT_DIR/lcov.info"

LCOV_LOG="GCOV_lcov.log"
GENHTML_LOG="GCOV_genhtml.log"

lcov --no-external --capture --directory $PWD -o "$INFO_FILE" 2>&1 | tee $LCOV_LOG
# remove files we do not intentionally test:
lcov --remove "$INFO_FILE" ".waf*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
lcov --remove "$INFO_FILE" "$PWD/modules/gtest/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
lcov --remove "$INFO_FILE" "$PWD/build/linux/libraries/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG

genhtml "$INFO_FILE" -o "$REPORT_DIR" 2>&1 | tee $GENHTML_LOG

echo "Coverage successful. Open $REPORT_DIR/index.html"