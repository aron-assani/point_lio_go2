#!/bin/bash
#
# Docker entrypoint for Point-LIO parameter optimization
#
# This script runs inside the Docker container and executes the optimization
# with appropriate ROS 2 environment setup.
#
# Usage:
#   docker run -v /path/to/data:/data optimization_image [OPTIONS]
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================================================
# Configuration
# ============================================================================

REPO_DIR="${REPO_DIR:=/root/point_lio_go2}"
ROSBAG_DIR="${ROSBAG_DIR:=/data/rosbag}"
OUTPUT_DIR="${OUTPUT_DIR:=/data/optimization_results}"
TRIALS="${TRIALS:=50}"
TIMEOUT="${TIMEOUT:=}"
TARGET_ATE="${TARGET_ATE:=0.5}"

# ============================================================================
# Functions
# ============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_header() {
    echo ""
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  Point-LIO Parameter Optimization Framework (Docker)             ║"
    echo "║  $1                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    echo ""
}

validate_environment() {
    log_info "Validating Docker environment..."

    # Check ROS 2
    if ! command -v ros2 &> /dev/null; then
        log_error "ros2 command not found. ROS 2 must be installed."
        exit 1
    fi
    log_success "ROS 2 found: $(ros2 --version)"

    # Check rosbag directory
    if [ ! -d "$ROSBAG_DIR" ]; then
        log_error "Rosbag directory not found: $ROSBAG_DIR"
        echo "Make sure to mount your rosbag with:"
        echo "  -v /path/to/rosbag:/data/rosbag"
        exit 1
    fi
    log_success "Rosbag directory found"

    # Check if Point-LIO is installed
    if ! ros2 pkg list | grep -q point_lio; then
        log_error "point_lio package not found in ROS 2 installation"
        exit 1
    fi
    log_success "Point-LIO package found"

    # Create output directory
    mkdir -p "$OUTPUT_DIR"
    log_success "Output directory: $OUTPUT_DIR"
}

install_dependencies() {
    log_info "Installing Python dependencies..."

    # Check if already installed
    if python3 -c "import optuna; import scipy" 2>/dev/null; then
        log_success "Dependencies already installed"
        return
    fi

    pip install -q optuna scipy numpy pyyaml matplotlib tabulate 2>/dev/null
    log_success "Dependencies installed"
}

print_config() {
    log_info "Configuration:"
    echo "  Repository: $REPO_DIR"
    echo "  Rosbag: $ROSBAG_DIR"
    echo "  Output: $OUTPUT_DIR"
    echo "  Trials: $TRIALS"
    echo "  Target ATE: ${TARGET_ATE}m"
    [ -n "$TIMEOUT" ] && echo "  Timeout: ${TIMEOUT}s"
    echo ""
}

run_optimization() {
    print_header "Starting Optimization"

    print_config

    cd "$REPO_DIR" || exit 1

    # Build command
    CMD="python3 optimize_utlidar.py"
    CMD="$CMD --rosbag '$ROSBAG_DIR'"
    CMD="$CMD --output-dir '$OUTPUT_DIR'"
    CMD="$CMD --trials $TRIALS"
    CMD="$CMD --target-ate $TARGET_ATE"

    if [ -n "$TIMEOUT" ]; then
        CMD="$CMD --timeout $TIMEOUT"
    fi

    log_info "Running optimization..."
    log_info "Command: $CMD"
    echo ""

    # Run with error handling
    if eval "$CMD"; then
        log_success "Optimization completed successfully!"
        print_results
    else
        log_error "Optimization failed!"
        exit 1
    fi
}

print_results() {
    echo ""
    print_header "Results Summary"

    RESULTS_FILE="$OUTPUT_DIR/optimization_log.json"

    if [ ! -f "$RESULTS_FILE" ]; then
        log_warning "Results file not found"
        return
    fi

    # Find best result
    BEST_ATE=$(python3 -c "
import json
with open('$RESULTS_FILE') as f:
    data = json.load(f)
    if data:
        best = min(data, key=lambda x: x['ate'])
        print(f\"{best['ate']:.4f}\")
" 2>/dev/null || echo "N/A")

    TRIALS_COUNT=$(python3 -c "
import json
with open('$RESULTS_FILE') as f:
    print(len(json.load(f)))
" 2>/dev/null || echo "N/A")

    log_info "Results:"
    echo "  Trials completed: $TRIALS_COUNT"
    echo "  Best ATE achieved: ${BEST_ATE}m"
    echo "  Results file: $RESULTS_FILE"
    echo ""
}

run_analysis() {
    if [ "$1" == "--analyze" ]; then
        print_header "Analyzing Results"

        cd "$REPO_DIR" || exit 1

        log_info "Generating analysis plots and statistics..."
        python3 analyze_optimization.py --results "$OUTPUT_DIR/optimization_log.json" --output-dir "$OUTPUT_DIR" --all

        log_success "Analysis complete!"
        echo "  Plots saved to: $OUTPUT_DIR"
        echo ""
    fi
}

# ============================================================================
# Main
# ============================================================================

main() {
    print_header "Initialization"

    validate_environment
    install_dependencies

    run_optimization

    if [ "$1" == "--analyze" ] || [ "$1" == "-a" ]; then
        run_analysis "$1"
    fi

    log_info "Optimization framework finished"
    log_success "Next steps:"
    echo "  1. Review results: cat $OUTPUT_DIR/optimization_log.json"
    echo "  2. Visualize: python3 analyze_optimization.py --all"
    echo "  3. Deploy optimized config: cp point_lio/config/utlidar.yaml point_lio/config/utlidar.yaml.optimized"
}

# ============================================================================
# Script Execution
# ============================================================================

# Handle signals gracefully
trap 'log_error "Interrupted!"; exit 130' INT TERM

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --rosbag)
            ROSBAG_DIR="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --trials)
            TRIALS="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --target-ate)
            TARGET_ATE="$2"
            shift 2
            ;;
        --analyze|-a)
            ANALYZE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --rosbag DIR      Path to rosbag (env: ROSBAG_DIR, default: /data/rosbag)"
            echo "  --output DIR      Output directory (env: OUTPUT_DIR, default: /data/optimization_results)"
            echo "  --trials N        Number of trials (env: TRIALS, default: 50)"
            echo "  --timeout S       Timeout in seconds (env: TIMEOUT)"
            echo "  --target-ate M    Target ATE in meters (env: TARGET_ATE, default: 0.5)"
            echo "  --analyze         Run post-analysis and generate plots"
            echo "  --help            Show this help message"
            echo ""
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

main

if [ "$ANALYZE" == "true" ]; then
    run_analysis "--analyze"
fi
