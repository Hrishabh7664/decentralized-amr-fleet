#!/usr/bin/env bash
# Automated Scenario Benchmark Execution Script
# Runs headless benchmarks across all 4 warehouse scenarios,
# produces CSV logs, and computes the improvement percentage vs stop-and-wait baseline.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================================="
echo " Starting Headless Multi-AMR Fleet Benchmarking Suite"
echo " Project Root: $PROJECT_ROOT"
echo "=========================================================="

# Ensure results directory exists
mkdir -p "$PROJECT_ROOT/results"

# 1. Run Baseline Stop-and-Wait
echo "[1/3] Running Stop-and-Wait Baseline Benchmark..."
python3 "$SCRIPT_DIR/benchmark_stop_and_wait.py"

# 2. Run Decentralized Framework
echo "[2/3] Running Decentralized Multi-AMR Framework Benchmark..."
python3 "$SCRIPT_DIR/benchmark_framework.py"

# 3. Compare and Validate Success Criteria
echo "[3/3] Analyzing CSV metrics and validating success criteria..."
python3 "$SCRIPT_DIR/compare_benchmarks.py"

echo "Benchmarking run completed successfully."
