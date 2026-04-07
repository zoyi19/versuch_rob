#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIRECTORY="$(dirname "$SCRIPT_DIR")"
ROOT_DIRECTORY=$ROOT_DIRECTORY/src

DIRECTORY_LIST=$1

while IFS= read -r dir_name; do
    if [ -n "$dir_name" ]; then
        echo "Searching for directories named '$dir_name' in $ROOT_DIRECTORY"
        find "$ROOT_DIRECTORY" -type d -name "$dir_name" -exec rm -rf {} +
    fi
done < "$DIRECTORY_LIST"

echo "Finished deleting directories."