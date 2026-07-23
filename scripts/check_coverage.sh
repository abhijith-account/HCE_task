#!/bin/bash
set -e

echo "========================================="
echo " Verifying 100% Line and Branch Coverage "
echo "========================================="

mkdir -p coverage_report

GCOVR_OPTS=(
    --root .
    --object-directory tests/build
    --exclude-throw-branches
    --exclude-unreachable-branches
    --filter src/
)

echo "Generating coverage reports..."

gcovr "${GCOVR_OPTS[@]}" \
    --html-details coverage_report/index.html \
    --txt coverage_report/coverage.txt \
    --txt-metric branch \
    --print-summary

echo
echo "========== UNCOVERED LINES =========="

gcovr "${GCOVR_OPTS[@]}" \
    --txt \
    --txt-metric line \
    | grep -A9999 "^Missing" || true

echo
echo "========= UNCOVERED BRANCHES ========="

gcovr "${GCOVR_OPTS[@]}" \
    --txt \
    --txt-metric branch \
    | grep -A9999 "^Missing" || true

echo
echo "Checking thresholds..."

gcovr "${GCOVR_OPTS[@]}" \
    --fail-under-line 100 \
    --fail-under-branch 100

echo
echo "SUCCESS: 100% Line and Branch Coverage Verified."
echo "HTML report: coverage_report/index.html"
