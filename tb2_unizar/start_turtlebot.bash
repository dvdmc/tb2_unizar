#!/bin/bash
set -euo pipefail

# Constants
readonly DEFAULT_NAMESPACE=""
readonly DEFAULT_MODE="hokuyo"
readonly VALID_MODES=("hokuyo" "optitrack" "gazebo" "teleop")
readonly TMUXINATOR_DIR="tmuxinator"

usage() {
    cat << EOF
Usage: $(basename "$0") [-n namespace] [-m mode]

Options:
    -n  Turtlebot namespace (default: ${DEFAULT_NAMESPACE})
    -m  Operation mode: ${VALID_MODES[*]} (default: ${DEFAULT_MODE})
    -h  Show this help message
EOF
}

# Parse command line arguments
namespace="${DEFAULT_NAMESPACE}"
mode="${DEFAULT_MODE}"

while getopts "n:m:h" opt; do
    case ${opt} in
        n) namespace="${OPTARG}" ;;
        m) 
            if [[ " ${VALID_MODES[*]} " =~ " ${OPTARG} " ]]; then
                mode="${OPTARG}"
            else
                echo "Error: Invalid mode '${OPTARG}'" >&2
                echo "Valid modes are: ${VALID_MODES[*]}" >&2
                exit 1
            fi
            ;;
        h) usage; exit 0 ;;
        *) usage >&2; exit 1 ;;
    esac
done

# Validate tmuxinator config exists
config_file="${TMUXINATOR_DIR}/launch_turtlebot_${mode}.yaml"
if [[ ! -f "${config_file}" ]]; then
    echo "Error: Configuration file '${config_file}' not found!" >&2
    exit 1
fi

# Launch turtlebot
echo "Launching turtlebot with namespace: ${namespace}"
tmuxinator start -n "${namespace}" -p "${config_file}" "tb_namespace=${namespace}" wait
sleep 0.1

# Attach to the tmux session
tmux attach-session -t "${namespace}"