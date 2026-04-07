#!/bin/bash

set -e  # 遇到错误立即退出

# 打印调试信息
echo "Current repo path: $CI_PROJECT_DIR"
echo "Kuavo RL repo URL: $kuavo_rl_repo_url"
echo "Target Kuavo RL repo Path: $temp_target_kuavo_rl_repo_path"
echo "Commit branch: $CI_COMMIT_BRANCH"
echo "Commit SHA: $CI_COMMIT_SHA"

# 检查 Kuavo RL 仓库是否已存在
if [ -d "$temp_target_kuavo_rl_repo_path/.git" ]; then
    echo "Repository already exists at $temp_target_kuavo_rl_repo_path. Fetching latest changes..."
    cd "$temp_target_kuavo_rl_repo_path"
    git fetch --prune origin
else
    echo "Cloning repository $kuavo_rl_repo_url to $temp_target_kuavo_rl_repo_path..."
    git clone "$kuavo_rl_repo_url" "$temp_target_kuavo_rl_repo_path"
    cd "$temp_target_kuavo_rl_repo_path"
fi

# 检查分支是否存在
if git show-ref --verify --quiet "refs/remotes/origin/$CI_COMMIT_BRANCH"; then
    echo "Branch $CI_COMMIT_BRANCH already exists. Resetting to match remote..."
    git checkout .
    git checkout "$CI_COMMIT_BRANCH" -f
    git reset --hard "origin/$CI_COMMIT_BRANCH"
else
    echo "Branch $CI_COMMIT_BRANCH does not exist. Creating from default branch..."
    git checkout dev -f
    git reset --hard "origin/dev"
    # 强制创建相同名字的分支
    git checkout -B "$CI_COMMIT_BRANCH"
fi

# 创建同步映射
declare -A SYNC_MAP=(
    ["src/kuavo_assets"]="src/kuavo_assets"
    ["src/kuavo-ros-control-lejulib"]="src/kuavo-ros-control-lejulib"
    # ["src/mujoco"]="src/mujoco"
    ["src/humanoid-control/humanoid_wbc"]="src/humanoid-control/humanoid_wbc"
)

# 同步文件夹
for source in "${!SYNC_MAP[@]}"; do
    target="${SYNC_MAP[$source]}"
    echo "Syncing $source to $temp_target_kuavo_rl_repo_path/$target..."
    mkdir -p "$(dirname "$target")"
    rsync -av --delete "$CI_PROJECT_DIR/$source/" "$temp_target_kuavo_rl_repo_path/$target/"
done

# # 同步MPC config
# echo "Syncing MPC config..."
# SOURCE_DIR="$CI_PROJECT_DIR/src/humanoid-control/humanoid_controllers/config"
# TARGET_DIR="$temp_target_kuavo_rl_repo_path/src/humanoid-control/humanoid_controllers/config"

# find "$SOURCE_DIR" -type d \( -name "command" -o -name "mpc" \) | while read -r source_path; do
#     # 计算相对路径
#     relative_path="${source_path#$SOURCE_DIR/}"
#     target_path="$TARGET_DIR/$relative_path"

#     # 同步文件夹
#     echo "Syncing $source_path to $target_path..."
#     mkdir -p "$target_path"
#     rsync -av --delete "$source_path/" "$target_path/"
# done

git status --porcelain 

# 检测是否有修改，包括新增文件
if git status --porcelain | grep -q '^[ MADRC?]'; then
    echo "Changes detected. Committing and pushing..."
    git add .

    # 提交信息包括 commit hash
    commit_message="Sync from kuavo-ros-control, commit: $CI_COMMIT_SHA, branch: $CI_COMMIT_BRANCH"
    git commit -m "$commit_message"

    git push origin "$CI_COMMIT_BRANCH"
    echo "Sync complete!"
    echo "notify_rl..."
    cd $CI_PROJECT_DIR/ci_scripts
    sudo chmod +x wechat_bot_notify.sh
    ./wechat_bot_notify.sh "Kuavo-RL 仓库分支 $CI_COMMIT_BRANCH 底层库有修改, 已经成功同步，请访问 https://www.lejuhub.com/highlydynamic/kuavo-RL/-/commits/$CI_COMMIT_BRANCH 查看详情" "$WECHAT_BOT_TOKEN"
    ./feishu_notify.sh "Kuavo-RL 仓库分支 $CI_COMMIT_BRANCH 底层库有修改, 已经成功同步，请访问 https://www.lejuhub.com/highlydynamic/kuavo-RL/-/commits/$CI_COMMIT_BRANCH 查看详情" "${feishu_notify_webhook:-}"
  

else
    echo "No changes detected. Exiting..."
    exit 0
fi
