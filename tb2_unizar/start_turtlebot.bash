#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from config file"
}

# Initialize variables with default values
tb2_namespace_comma=""
use_gnome="false"

# Arg parser
while getopts "n" opt; do
  case ${opt} in
    n )
      tb2_namespace_comma="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, use just tb2_unizar
if [ -z "$tb2_namespace_comma" ]; then
  tb2_namespace_comma="tb2_unizar"
fi
IFS=',' read -r -a tb2_namespaces <<< "$tb2_namespace_comma"

# Launch tb2 for each drone namespace
for namespace in ${tb2_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${tb2_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator start -n ${namespace} -p tmuxinator/launch_turtlebot_hokuyo.yaml \
    tb_namespace=${namespace} \
    wait"

  sleep 0.1 # Wait for tmuxinator to finish
done

# Attach to tmux session
tmux attach-session -t ${tb2_namespaces[0]}