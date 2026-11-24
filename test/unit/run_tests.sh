#!/usr/bin/env bash
# Runs all host test suites and prints a machine-generated grand total.
# The total is the sum of _tests_run across all suites (TEST_ASSERT* calls
# executed at runtime, NOT a static grep count). Some suites have loops that
# call the same assertion multiple times, so runtime count >= source lines.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

make clean > /dev/null
OUTPUT=$(make run 2>&1)
RC=$?

printf '%s\n' "$OUTPUT"
echo ""
echo "════════════════════════════════"
# Parse "X tests, Y failures" lines produced by TEST_SUITE_SUMMARY()
TOTAL=$(printf '%s\n' "$OUTPUT" | awk '
  /^[0-9]+ tests, [0-9]+ failures$/ {
    ta += $1; tf += $3
  }
  END { printf "%d assertions executed, %d failures\n", ta, tf }
')
echo "Grand total: $TOTAL"
echo "════════════════════════════════"
exit "$RC"
