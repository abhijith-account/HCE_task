#!/bin/bash
set -euo pipefail

echo "======================================="
echo " Verifying 100% Line and Branch Coverage "
echo "======================================="

mkdir -p coverage_report

GCOVR_BASE_OPTS=(
    --root .
    --exclude-throw-branches
    --exclude-unreachable-branches
    --filter src/
)

echo "Generating coverage reports..."

gcovr "${GCOVR_BASE_OPTS[@]}" \
    --object-directory tests/build \
    --json coverage_report/coverage.json \
    --html-details coverage_report/index.html \
    --txt coverage_report/coverage.txt \
    --txt-metric branch \
    --print-summary

TRACE_OPTS=("${GCOVR_BASE_OPTS[@]}" --add-tracefile coverage_report/coverage.json)

echo
echo "========== UNCOVERED LINES =========="

gcovr "${TRACE_OPTS[@]}" \
    --txt \
    --txt-metric line \
    | grep -A999 "^Missing" || true

echo
echo "========== UNCOVERED BRANCHES =========="

gcovr "${TRACE_OPTS[@]}" \
    --txt \
    --txt-metric branch \
    | grep -A999 "^Missing" || true

echo
echo "Checking thresholds..."

gcovr "${TRACE_OPTS[@]}" \
    --fail-under-line 100 \
    --fail-under-branch 100

echo
echo "SUCCESS: 100% Line and Branch Coverage Verified."
echo "HTML report: coverage_report/index.html"
