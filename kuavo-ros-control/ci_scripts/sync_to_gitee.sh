#!/bin/bash

set -u
set -o pipefail

prepare_gitee_ssh() {
  local ssh_home ssh_dir private_key known_hosts
  ssh_home="$(getent passwd "$(id -u)" 2>/dev/null | cut -d: -f6)"
  if [ -z "$ssh_home" ]; then
    ssh_home="${HOME:-/root}"
  fi

  ssh_dir="${ssh_home}/.ssh"
  private_key="${ssh_dir}/id_rsa"
  known_hosts="${ssh_dir}/known_hosts"

  mkdir -p "$ssh_dir"
  touch "$known_hosts"
  chmod 700 "$ssh_dir"
  chmod 600 "$known_hosts"

  if [ ! -f "$private_key" ]; then
    echo "Error: private key not found: $private_key"
    return 1
  fi

  echo "Using SSH key: $private_key"
  ssh-keygen -lf "$private_key" || true

  ssh-keygen -R gitee.com -f "$known_hosts" >/dev/null 2>&1 || true
  ssh-keygen -R 198.18.1.98 -f "$known_hosts" >/dev/null 2>&1 || true
  if ! ssh-keyscan -H -t rsa,ecdsa,ed25519 gitee.com 2>/dev/null >> "$known_hosts"; then
    echo "Warning: failed to fetch host keys for gitee.com"
  fi
  if ! ssh-keyscan -H -t rsa,ecdsa,ed25519 198.18.1.98 2>/dev/null >> "$known_hosts"; then
    echo "Warning: failed to fetch host keys for 198.18.1.98"
  fi
  sort -u "$known_hosts" -o "$known_hosts"
  chmod 600 "$known_hosts"

  export GIT_SSH_COMMAND="ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o CheckHostIP=no -o UserKnownHostsFile='$known_hosts' -i '$private_key'"
}

# Improved debug by echoing the two environment values
origin_gitee_url="$opensource_project_origin_gitee"
echo "origin_gitee_url is: $origin_gitee_url"

# Convert git SSH URL to HTTPS URL for notifications
# e.g. git@gitee.com:leju-robot/kuavo-ros-opensource.git -> https://gitee.com/leju-robot/kuavo-ros-opensource
origin_gitee_https_url=$(echo "$origin_gitee_url" | sed 's|git@gitee.com:|https://gitee.com/|; s|\.git$||')

cd "$temp_target_repo_path/git_repo" || exit 1
echo "Current working directory: $(pwd)"

# Check if the remote exists and get its URL if it does
existing_url=$(git remote get-url origin_gitee 2>/dev/null)

if [ "$existing_url" ]; then
    echo "The remote 'origin_gitee' already exists."
    # Check if the existing URL is the one we want
    if [ "$existing_url" != "$origin_gitee_url" ]; then
        echo "The remote URL does not match. Updating the remote 'origin_gitee'."
        # Remove the old remote and add the new one
        git remote remove origin_gitee
        git remote add origin_gitee "$origin_gitee_url"
        echo "The remote 'origin_gitee' has been updated to $origin_gitee_url."
    else
        echo "The remote URL matches the expected URL. Skip add origin_gitee"
    fi
else
    # If the remote doesn't exist, add it
    git remote add origin_gitee "$origin_gitee_url"
    echo "The remote 'origin_gitee' has been added with URL $origin_gitee_url."
fi

prepare_gitee_ssh || exit 1

max_attempts=3
push_exit_code=1 # Initialize to failure status
is_quota_exceeded=false

for attempt in $(seq 1 "$max_attempts"); do
  echo "Attempt $attempt/$max_attempts: Pushing branch $CI_COMMIT_BRANCH to origin_gitee..."
  push_output=$(git push -f origin_gitee "$CI_COMMIT_BRANCH" 2>&1)
  push_exit_code=$?
  echo "$push_output"

  if [ $push_exit_code -eq 0 ]; then
    echo "Push successful on attempt $attempt."
    break # Exit loop on success
  fi

  if echo "$push_output" | grep -q "size exceeds limit"; then
    echo "Push failed on attempt $attempt with exit code $push_exit_code. Detected quota exceeded error."
    is_quota_exceeded=true
    break # No point retrying if quota is exceeded
  fi

  echo "Push failed on attempt $attempt with exit code $push_exit_code."

  if [ $attempt -lt $max_attempts ]; then
    wait_time=$((attempt * 60))
    echo "Waiting for $wait_time seconds before retrying..."
    sleep $wait_time
  else
    echo "Maximum retry attempts ($max_attempts) reached. Push failed."
  fi
done

if [ $push_exit_code -eq 0 ]; then
   echo "Sync succeeded. Sending success notification."
   bash "$CI_PROJECT_DIR/ci_scripts/wechat_bot_notify.sh" "OCS2 开源仓库(码云)分支 $CI_COMMIT_BRANCH 同步成功，请访问 ${origin_gitee_https_url} 来查看" "$WECHAT_BOT_TOKEN"
   bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "OCS2 开源仓库(码云)分支 $CI_COMMIT_BRANCH 同步成功，请访问 ${origin_gitee_https_url} 来查看" "${feishu_notify_webhook:-}"
else
    if [ "$is_quota_exceeded" = true ]; then
        echo "Sync failed due to quota exceeded (is_quota_exceeded=$is_quota_exceeded). Sending quota exceeded notification."
        bash "$CI_PROJECT_DIR/ci_scripts/wechat_bot_notify.sh" "@黄贤贤 OCS2 开源仓库(Gitee) 空间已满，请清理仓库。当前分支: $CI_COMMIT_BRANCH。请点击链接进行 GC 清理: ${origin_gitee_https_url}/settings#git-gc" "$WECHAT_ERR_BOT_TOKEN"
        bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "OCS2 开源仓库(Gitee) 空间已满，请清理仓库。当前分支: $CI_COMMIT_BRANCH。请点击链接进行 GC 清理: ${origin_gitee_https_url}/settings#git-gc" "${feishu_notify_webhook:-}"
    else
        echo "Sync failed (push_exit_code=$push_exit_code, is_quota_exceeded=$is_quota_exceeded). Sending failure notification."
        bash "$CI_PROJECT_DIR/ci_scripts/wechat_bot_notify.sh" "@黄贤贤 OCS2 开源仓库(码云)分支 $CI_COMMIT_BRANCH 同步失败，请跟进。 请访问 ${CI_PIPELINE_URL} 来查看" "$WECHAT_ERR_BOT_TOKEN"
        bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "OCS2 开源仓库(码云)分支 $CI_COMMIT_BRANCH 同步失败，请跟进。 请访问 ${CI_PIPELINE_URL} 来查看" "${feishu_notify_webhook:-}"
    fi
    exit 1  # 让 CI 失败，方便排查问题
fi
