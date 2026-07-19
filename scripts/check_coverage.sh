#!/bin/bash
set -e

echo "========================================="
echo " Verifying 100% Line and Branch Coverage "
echo "========================================="

mkdir -p coverage_report

# Generate reports
gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --html-details coverage_report/index.html \
    --txt coverage_report/coverage.txt \
    --txt-metric branch \
    --print-summary

echo
echo "========== UNCOVERED LINES =========="

gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --txt \
    --txt-metric line \
    | grep -A1000 "Missing" || true

echo
echo "========= UNCOVERED BRANCHES ========="

gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --txt \
    --txt-metric branch \
    | grep -A1000 "Missing" || true

echo
echo "Checking thresholds..."

gcovr \
    --root . \
    --object-directory tests/build \
    --filter src/ \
    --exclude-unreachable-branches \
    --exclude-throw-branches \
    --fail-under-line 100 \
    --fail-under-branch 100

echo
echo "SUCCESS: 100% Line and Branch Coverage Verified."
echo "HTML report: coverage_report/index.html"
