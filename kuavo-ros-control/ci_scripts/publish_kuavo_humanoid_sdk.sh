#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../") # project: kuavo-ros-control
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
CI_SCRIPT_DIR=$(realpath "$PROJECT_DIR/ci_scripts/")
DEVEL_DIR="$PROJECT_DIR/devel"
MSG_PACKAGES="kuavo_msgs ocs2_msgs motion_capture_ik" # 依赖的消息包
VERSION=$(git -C "$PROJECT_DIR" describe --tags --always 2>/dev/null)
TESTPYPI_DEBUG="false" # 仅测试使用，为true时发布到 testpypi

notify_to_wxwork_group() {
    local msg=$1
    if [ -z "$msg" ]; then
        echo "[kuavo-humanoid-sdk] No message provided for notification"
        return 1
    fi
    
    echo -e "\033[32mSending message to WxWork Group: $msg \033[0m"
    cd $CI_SCRIPT_DIR
    chmod +x wechat_bot_notify.sh
    ./wechat_bot_notify.sh "$msg" "$WECHAT_BOT_TOKEN"
    bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "$msg" "${feishu_notify_webhook:-}"
    cd -
}

clean_cache() {
    # Clean up message directories
    echo "Cleaning message directories..."
    if [ -d "$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg" ]; then
        # Find and remove all directories under msg/ (but keep the msg directory itself)
        find "$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg" -mindepth 1 -type d -exec rm -rf {} \; 2>/dev/null || true
        echo -e "\033[32mMessage directories cleaned successfully\033[0m"
    else
        echo -e "\033[33mWarning: Message directory does not exist: $SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg\033[0m"
    fi

    # Clean up build and dist directories
    for dir in "build" "dist"; do
        if [ -d "$SDK_PROJECT_DIR/$dir" ]; then
            rm -rf "$SDK_PROJECT_DIR/$dir"
            echo -e "\033[32m${dir^} directory cleaned successfully\033[0m"
        fi
    done
}

copy_ros_msg() {
    local src_dir=$1
    local dest_dir=$2
    local msg_pkg=$3

    # echo "$src_dir/$msg_pkg"
    if [ -d "$src_dir/$msg_pkg" ]; then
        echo "src: $src_dir"
        echo -e "\033[32mCopying $msg_pkg ...\033[0m"
        if [ -d "$dest_dir/$msg_pkg" ]; then
            rm -rf "$dest_dir/$msg_pkg"
        fi
        mkdir "$dest_dir/$msg_pkg"
        cp -r "$src_dir/$msg_pkg" "$dest_dir"

        # Create __init__.py file with import statements
        echo "import os
import sys

# Add package path to sys.path if not already there
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if package_path not in sys.path:
    sys.path.append(package_path)" > "$dest_dir/$msg_pkg/__init__.py"
        
    else 
        echo -e "\033[31mError: 未找到对应的消息包，请先执行 catkin build $msg_pkg 构建\033[0m"
        exit_with_failure
    fi
}

copy_deps_msgs() {
    dest_dir="$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg"
    IFS=' ' read -r -a MSG_ARRAY <<< "$MSG_PACKAGES"
    for msg_pkg in "${MSG_ARRAY[@]}"; do
        devel_pkg_dir="$DEVEL_DIR/lib/python3/dist-packages/"
        if [ -d "$devel_pkg_dir" ]; then
            # Copy the ROS message packages from the installed directory to the destination directory
            copy_ros_msg "$devel_pkg_dir" "$dest_dir" "$msg_pkg" 
        else
            echo -e "\033[31mError: Neither the installed nor the devel directory exists. Path: $devel_pkg_dir\033[0m"
            exit_with_failure
        fi
    done
}

build_package() {
    IFS=' ' read -r -a MSG_ARRAY <<< "$MSG_PACKAGES"
    for msg_pkg in "${MSG_ARRAY[@]}"; do
       msg_dir="$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg/$msg_pkg"
        # Check if the message directory exists
        if [ ! -d "$msg_dir" ]; then
            echo -e "\033[31mError: Message directory $msg_dir does not exist\033[0m"
            exit_with_failure
        fi
    done

    # Check if setuptools and wheel are installed
    for pkg in "setuptools" "wheel"; do
        if ! pip3 list 2>/dev/null | grep -q "$pkg"; then
            echo -e "\033[33mInstalling $pkg...\033[0m"
            pip3 install "$pkg" 2>/dev/null || {
                echo -e "\033[31mWarning: Failed to install $pkg with pip3, trying with pip...\033[0m"
                pip install "$pkg" || echo -e "\033[31mError: Failed to install $pkg\033[0m"
            }
        fi
    done
    
    # Build the package
    pushd $SDK_PROJECT_DIR
    echo -e "\033[32mBuilding package...\033[0m"
    # 设置环境变量 KUAVO_HUMANOID_SDK_VERSION
    KUAVO_HUMANOID_SDK_VERSION="$VERSION" python3 setup.py sdist bdist_wheel --build-number $(date +%Y%m%d%H%M%S)
    popd
}

publish_to_pypi() {
    username="__token__"
    password="$PYPI_API_TOKEN"
    if [ -z "$password" ]; then
        msg="PYPI_API_TOKEN is not set. Please set it in your environment variables."
        echo -e "\033[31mError: $msg \033[0m"
        exit_with_failure "$msg"
    fi
    
    pushd $SDK_PROJECT_DIR
    # Check if there are packages in the dist directory
    if [ ! -d "dist" ] || [ -z "$(ls -A dist 2>/dev/null)" ]; then
        echo -e "\033[31mError: No packages found in the dist directory\033[0m"
        popd
        exit_with_failure
    fi
    
    # Determine if we're publishing to TestPyPI or PyPI
    if [ "$TESTPYPI_DEBUG" = "true" ]; then
        echo -e "\033[32mPublishing package to TestPyPI...\033[0m"
        # 使用临时文件捕获输出，同时实时显示
        temp_file=$(mktemp)
        if python3 -m twine upload --verbose --repository testpypi --username "$username" --password "$password" dist/* 2>&1 | tee "$temp_file"; then
            response=$(cat "$temp_file")
            if echo "$response" | grep -q "200 OK"; then
                echo -e "\033[32mSuccessfully published to TestPyPI\033[0m"
            else
                echo -e "\033[31mFailed to publish to TestPyPI (No 200 OK): $response\033[0m"
                rm "$temp_file"
                exit_with_failure
            fi
        else
            echo -e "\033[31mTwine upload command failed.\033[0m"
            rm "$temp_file"
            exit_with_failure
        fi
        rm "$temp_file"
    else
        echo -e "\033[32mPublishing package to PyPI...\033[0m"
        temp_file=$(mktemp)
        if python3 -m twine upload --verbose --repository pypi --username "$username" --password "$password" dist/* 2>&1 | tee "$temp_file"; then
            response=$(cat "$temp_file")
            if echo "$response" | grep -q "200 OK"; then
                echo -e "\033[32mSuccessfully published to PyPI\033[0m"
            else
                echo -e "\033[31mFailed to publish to PyPI (No 200 OK): $response\033[0m"
                rm "$temp_file"
                exit_with_failure
            fi
        else
            echo -e "\033[31mTwine upload command failed.\033[0m"
            rm "$temp_file"
            exit_with_failure
        fi
        rm "$temp_file"
    fi
    popd
}

exit_with_failure() {
    # Check if we're in a pushed directory and pop if needed
    if [ -n "$OLDPWD" ]; then
        popd 2>/dev/null || true
    fi
    clean_cache
    if [ -n "$1" ]; then
        echo -e "\033[31mExit with failure: $1\033[0m"
        notify_msg="[Kuavo Humanoid SDK] 发布失败, 错误信息: $1"
        notify_to_wxwork_group "$notify_msg"
    fi
    exit 1
}

exit_with_success() {
    local branch="$1"
    clean_cache
    version=""
    if [[ "$branch" == *"beta"* ]]; then
        version=" Beta 版本"
    elif [[ "$branch" == *"master"* ]]; then
        version="正式版本"
    else 
        version="非正式版本"    
    fi
    msg="[Kuavo Humanoid SDK] 🎉 最新$version($VERSION)已经发布 🚀: https://pypi.org/project/kuavo-humanoid-sdk/$VERSION"
    echo -e "\033[32m$msg\033[0m"
    notify_to_wxwork_group "$msg"
    exit 0
}

exit_with_cancel() {
    local branch="$1"
    local msg="$2"
     if [ -n "$OLDPWD" ]; then
        popd 2>/dev/null || true
    fi
    clean_cache
    echo -e "\033[33m[kuavo-humanoid-sdk]取消发布, 原因是: $msg\033[0m"
    exit 0
}

check_and_format_version() {
    local branch="$1"
    local -n __version_ref="$2"
    
    if [ $? -ne 0 ] || [ -z "$__version_ref" ]; then
        exit_with_failure "Failed to get version from git describe"
    fi

    # 通过git获取(e.g.): 1.1.0-324-g792046c35, 1.2.0 ...
    # Remove the hash part (g followed by alphanumeric characters) from the version
    local version1=$(echo "$__version_ref" | sed 's/-g[0-9a-f]\+//') # 删除 hash后缀
    if [ "$branch" == "beta" ]; then
        # Replace hyphens with 'b' in the version string
        version1=$(echo "$version1" | sed 's/-/b/g')      # beta 版本: 1.1.0-324  ---> 1.1.0b324
        if [[ ! "$version1" == *"b"* ]]; then
            # 避免在beta分支上发布 1.1.0 的情况(1.1.0这样的版本号是给正式版使用的) --> 1.1.0b0
            version1="${version1}b0"  # Append 'b0' if version does not contain 'b'
        fi
    elif [ "$branch" == "master" ]; then
        version1=$(echo "$version1" | sed 's/-/.post/g')  # master 正式版: 1.1.0-324  ---> 1.1.0.post324
        
        # master 分支正式版只会在打tag的时候才发布
        # For master branch, ensure version is in proper release format (e.g., 1.1.0)
        if [[ ! "$version1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            exit_with_cancel "$branch" "master 分支正式版只会在打 tag 的时候才发布, 当前版本: $version1"
        fi
    else 
        # Replace hyphens with 'a' in the version string
        version1=$(echo "$version1" | sed 's/-/a/g')      # 其他 版本: 1.1.0-324  ---> 1.1.0a324
        if [[ ! "$version1" == *"a"* ]]; then
            # 避免在dev分支上发布 1.1.0 的情况(1.1.0这样的版本号是给正式版使用的) --> 1.1.0a0
            version1="${version1}a0"  # Append 'a0' if version does not contain 'a'
        fi
    fi
    
    __version_ref="$version1"
}

usage() {
    echo "Usage: $0 [--branch <branch_name>] [--help]"
    echo "Options:"
    echo "  --branch <branch_name>    Specify the branch name"
    echo "  --help                    Display help message"
}

#@@@@@ MAIN SCRIPT @@@@@
# Parse command line arguments
BRANCH=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --branch)
            if [[ -n "$2" ]]; then
                BRANCH="$2"
                shift 2
            else
                echo -e "\033[31mError: --branch requires an argument\033[0m"
                exit 1
            fi
            ;;
        -h|--help)
            echo "Usage: $0 [--branch <branch_name>] [-h|--help]"
            echo "Options:"
            echo "  --branch <branch_name>    Specify the branch name"
            echo "  -h, --help                Display this help message"
            exit 0
            ;;
        *)
            usage
            exit 1
            ;;
    esac
done

# Check if branch is specified
if [ -z "$BRANCH" ]; then
    usage 
    exit 1
fi

install_dependencies() {
    echo "Installing dependencies..."
    pip install --upgrade requests-toolbelt
    pip install urllib3 twine pyopenssl cryptography
}


check_and_format_version "$BRANCH" VERSION
echo -e "\033[32m[kuavo-humanoid-sdk] Current version: $VERSION\033[0m"
echo -e "\033[32m[kuavo-humanoid-sdk] Current branch: $BRANCH\033[0m"
install_dependencies
clean_cache     
copy_deps_msgs  
build_package   
publish_to_pypi
exit_with_success "$BRANCH"
