#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIRECTORY="$(dirname "$SCRIPT_DIR")"
ROOT_DIRECTORY=$ROOT_DIRECTORY/src
cd ${ROOT_DIRECTORY}/manipulation_nodes/motion_capture_ik/scripts/ik
echo "Compiling motion_capture_ik scripts..."
python3 -m compileall -b . && find . -name "*.py" -type f -delete

rm -rf ${ROOT_DIRECTORY}/src/manipulation_nodes/motion_capture_ik/.git
echo "build motion_capture_ik Done!"
