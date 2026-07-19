#!/bin/bash
set -e

echo "Verifying 100% Line and Branch Coverage"

mkdir -p coverage_report

gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --fail-under-line 100 \
    --fail-under-branch 100 \
    --txt coverage_report/coverage.txt \
    --html-details coverage_report/index.html \
    --sonarqube coverage_report/sonarqube.xml \
    --print-summary \
    --verbose

echo
echo "================ UNCOVERED LINES ================"
gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --txt-metric line \
    --txt

echo
echo "=============== UNCOVERED BRANCHES =============="
gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --txt-metric branch \
    --txt

gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --decisions \
    --html-details coverage_report/index.html

echo
echo "Detailed HTML report:"
echo "coverage_report/index.html"

echo "SUCCESS: Coverage verification complete."
