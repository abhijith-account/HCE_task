#!/bin/bash
set -e

echo "Verifying 100% Line and Branch Coverage"

mkdir -p coverage_report

gcovr \
    --root . \
    --object-directory tests/build/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --filter src/ \
    --fail-under-line 100 \
    --fail-under-branch 100 \
    --html-details coverage_report/index.html \
    --print-summary

echo "SUCCESS: 100% Line and Branch Coverage Verified. HTML Report generated."
exit 0
