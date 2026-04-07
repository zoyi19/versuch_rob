#!/bin/bash
# This software is licensed under the BSD License.
# Copyright (c) [year] Lejurobot. All rights reserved.

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
GIT_COMMIT=$(git rev-parse HEAD 2>/dev/null)
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
if [ -z "$GIT_COMMIT" ] || [ -z "$GIT_BRANCH" ]; then
    echo -e "\033[31mError: Failed to get git commit hash or branch name\033[0m"
    exit 1
fi

OUTPUT_DIR="/tmp/kuavo-dbgsym/${GIT_BRANCH}/${GIT_COMMIT}"
INFO_TXT="$OUTPUT_DIR/info.txt"
SYMBOL_DIR="$OUTPUT_DIR/debug/.build-id"
TAR_XZ_FILE="/tmp/kuavo-dbgsym/kuavo-dbgsym_${GIT_COMMIT}.tar.gz"
CURL_DEBUG_LOG="$CI_BUILDS_DIR/${CI_JOB_ID}/tmp/curl_debug.log"
mkdir -p "$(dirname "$CURL_DEBUG_LOG")"
touch "$CURL_DEBUG_LOG"
LIB_DIR=$(readlink -f "$SCRIPT_DIR/../installed/lib/")
BIN_DIR=$(readlink -f "$SCRIPT_DIR/../installed/bin/")

install_deps() {
    deps_packages="pv pigz binutils jq"
    # Check if packages are installed
    packages_to_install=()
    for pkg in $deps_packages; do
        if ! dpkg -l | grep -q "^ii.*$pkg"; then
            packages_to_install+=("$pkg")
        fi
    done

    # Install missing packages
    if [ ${#packages_to_install[@]} -gt 0 ]; then
        echo "Installing required packages: ${packages_to_install[*]}"
        sudo apt-get install -y "${packages_to_install[@]}"
    fi
}

extract_debug_symbol() {
    local app="$1"
    local target_dir="$2"

    # 提取 Build-ID
    local build_id=$(readelf -n "$app" | grep 'Build ID' | awk '{print $3}')
    if [ -z "$build_id" ]; then
        echo "Error: Failed to extract Build-ID from '$app'"
        return
    fi
    local dir_prefix="${build_id:0:2}"
    local file_suffix="${build_id:2}"
    local debug_file="$target_dir/$dir_prefix/$file_suffix.debug"
    
    # Check if debug symbols exist
    if ! readelf -S "$app" | grep -q '\.debug_info'; then
        # Get basename for libraries without debug symbols
        lib_basename=$(basename "$app")
        echo "$lib_basename No Symbol" >> "$INFO_TXT"
    else
        objcopy --only-keep-debug "$app" "$app.debug.tmp"

        # 移动文件
        mkdir -p "$target_dir/$dir_prefix"
        mv "$app.debug.tmp" "$debug_file"

        # 输出app名称和debug文件路径到INFO_TXT
        lib_basename=$(basename "$app")
        echo "$lib_basename debug/.build-id/$dir_prefix/$file_suffix.debug" >> "$INFO_TXT"
    fi
}

extract_debug_symbols() {
    local lib_dir="$1"
    local symbol_dir="$2"

    echo "Extracting debug symbols from $lib_dir"

    # Check if lib_dir exists
    if [ ! -d "$lib_dir" ]; then
        exit_with_fail "dir does not exist, $lib_dir"
    fi

    # Define file types to filter out
    local filter_types="py pyc make cmake txt"    
    find "$lib_dir" -type f -print0 | while IFS= read -r -d '' obj_file; do
        # Skip files with filtered extensions
        for ext in $filter_types; do
            if [[ "$obj_file" == *.$ext ]]; then
                continue 2
            fi
        done
        if file "$obj_file" | grep -Eq "shared object|executable"; then
            echo "Extracting debug symbols from $obj_file"
            extract_debug_symbol "$obj_file" "$symbol_dir"
        fi
    done
}

strip_symbols() {
    local lib_dir="$1"
    
    # Check if lib_dir exists
    if [ ! -d "$lib_dir" ]; then
        return
    fi

    find "$lib_dir" -type f -print0 | while IFS= read -r -d '' file; do
        # 通过file命令精确识别ELF格式文件
        if LC_ALL=C file "$file" | grep -qE 'ELF.*(executable|shared object)'; then
            echo "Stripping symbols: $file"
            strip --strip-unneeded "$file" 2>/dev/null || echo "Failed: $file (may be non-ELF or already stripped)"
        fi
    done
}

upload_debinfo() {
    local dir="$1"
    local zip_file="$2"

    # Check if jq is installed
    if ! command -v jq &>/dev/null; then
        echo -e "\033[31mError: jq is not installed. Please install jq first.\033[0m"
        exit 1
    fi

    # Check if OUTPUT_DIR exists
    if [ ! -d "$dir" ]; then
        echo "Error: OUTPUT_DIR does not exist: $dir"
        exit_with_fail
    fi

    # Create compressed archive with maximum compression using pv and pigz
    echo "Compressing files to $zip_file"
    if tar -c -C "$(dirname "$dir")" "$(basename "$dir")" | pv | pigz -9 > "$zip_file"; then
        echo "Successfully created archive: $zip_file ($(du -h "$zip_file" | cut -f1))"
    else
        echo "Failed to create archive" >&2
        exit_with_fail "Archive creation failed"
    fi

    # upload symbol zip to server!
    local response
    local curl_exit_code
    local connect_timeout=60
    local once_curl_max_time=600 # 包括 connect_timeout
    local retry_times=5
    local retry_delay=5
    local retry_max_time=$(( ($retry_times + 1) * ($retry_delay + $once_curl_max_time) ))
    response=$(curl -v -s -w "\n%{http_code}" \
        --connect-timeout $connect_timeout \
        --max-time $once_curl_max_time \
        --retry $retry_times \
        --retry-connrefused \
        --retry-delay $retry_delay \
        --retry-max-time $retry_max_time \
        -X POST https://crash.lejurobot.com/files/upload/compiled \
        -H "Content-Type: multipart/form-data" \
        -F "file=@$zip_file" 2> "$CURL_DEBUG_LOG")
    curl_exit_code=$?

    if [ $curl_exit_code -ne 0 ]; then
        cat "$CURL_DEBUG_LOG" >&2
        exit_with_fail "Upload failed. curl command exited with code $curl_exit_code."
    fi

    # Extract body and http_code from the response
    # Use tr -d '\r' to remove carriage returns that may come from the server
    local http_code=$(printf '%s' "$response" | tail -n1 | tr -d '\r')
    local body=$(printf '%s' "$response" | head -n -1 | tr -d '\r')

    # Check for empty response body, which can happen even with a 200 OK
    if [ -z "$body" ]; then
        cat "$CURL_DEBUG_LOG" >&2
        exit_with_fail "Upload failed: Empty response body from server (HTTP status: $http_code)."
    fi

    # Check if response is valid JSON
    if ! printf '%s' "$body" | jq . >/dev/null 2>&1; then
        cat "$CURL_DEBUG_LOG" >&2
        exit_with_fail "Upload failed: Invalid JSON response from server (HTTP status: $http_code): $body"
    fi

    # Parse application-level response status
    local status=$(printf '%s' "$body" | jq -r '.status')
    if [ "$status" != "success" ]; then
        local error_message=$(printf '%s' "$body" | jq -r '.message')
        cat "$CURL_DEBUG_LOG" >&2
        exit_with_fail "Failed to upload debug symbols. Server responded with status '$status' (HTTP code: $http_code). Reason: $error_message"
    fi
    echo "Successfully uploaded debug symbols"
}

exit_with_fail() {
    if [ -n "$1" ]; then
        echo -e "\033[31mError:$1\033[0m"
    fi
    post_cleanup
    exit 1
}

exit_with_success() {
    post_cleanup
    exit 0
}

post_cleanup() {
    # 清除缓存
    rm -rf "$OUTPUT_DIR"
    rm -rf "$TAR_XZ_FILE"
    strip_symbols "$LIB_DIR"                       # strip installed/lib
    strip_symbols "$BIN_DIR"                       # strip installed/bin
}

#@@@ MAIN @@@
echo "Branch: $GIT_BRANCH"
echo "Commit: $GIT_COMMIT"
echo "Output Directory: $OUTPUT_DIR"
echo "Symbol Directory: $SYMBOL_DIR"
echo "INFO_TXT: $INFO_TXT"

rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"
mkdir -p "$SYMBOL_DIR"
echo "Branch: $GIT_BRANCH" > "$INFO_TXT"
echo "Commit: $GIT_COMMIT" >> "$INFO_TXT"
echo "-------------------------------------" >> "$INFO_TXT"
echo " lib name              debug file" >> "$INFO_TXT"
install_deps                                   # install deps
extract_debug_symbols "$LIB_DIR" "$SYMBOL_DIR" # extract symbols
if [ -d "$BIN_DIR" ]; then
    extract_debug_symbols "$BIN_DIR" "$SYMBOL_DIR" # extract symbols
fi
strip_symbols "$LIB_DIR"                       # strip installed/lib
strip_symbols "$BIN_DIR"                       # strip installed/bin
upload_debinfo "$OUTPUT_DIR" "$TAR_XZ_FILE"    # upload 
exit_with_success                              # exit with success 
