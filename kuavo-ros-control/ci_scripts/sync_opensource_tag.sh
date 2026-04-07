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
    echo "carlos_Error: private key not found: $private_key"
    return 1
  fi

  ssh-keygen -R gitee.com -f "$known_hosts" >/dev/null 2>&1 || true
  ssh-keygen -R 198.18.1.98 -f "$known_hosts" >/dev/null 2>&1 || true
  ssh-keyscan -H -t rsa,ecdsa,ed25519 gitee.com 2>/dev/null >> "$known_hosts" || true
  ssh-keyscan -H -t rsa,ecdsa,ed25519 198.18.1.98 2>/dev/null >> "$known_hosts" || true
  sort -u "$known_hosts" -o "$known_hosts"
  chmod 600 "$known_hosts"

  export GIT_SSH_COMMAND="ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o CheckHostIP=no -o UserKnownHostsFile=$known_hosts -i $private_key"
}

echo "carlos_Starting tag synchronization script..."

git fetch origin --tags
echo "carlos_Fetched tags from origin."

CURRENT_COMMIT_HASH=$(git rev-parse --short HEAD)
echo "carlos_Current commit hash: $CURRENT_COMMIT_HASH"

cd "$temp_target_repo_path/git_repo" || { echo "carlos_Error: Could not navigate to $temp_target_repo_path/git_repo"; exit 1; }
echo "carlos_Navigated to target repository: $temp_target_repo_path/git_repo"

COMMIT_WITH_HASH=$(git log --all --grep="$CURRENT_COMMIT_HASH" --pretty=format:"%H" -n 1)
echo "carlos_Found commit with hash: $COMMIT_WITH_HASH"

if [ -n "$COMMIT_WITH_HASH" ]; then
  git tag "$CI_COMMIT_TAG" "$COMMIT_WITH_HASH" || { echo "carlos_Error: Could not create tag $CI_COMMIT_TAG"; exit 1; }
  echo "carlos_Created tag $CI_COMMIT_TAG on commit $COMMIT_WITH_HASH"

  git push origin "$CI_COMMIT_TAG" || { echo "carlos_Error: Could not push tag $CI_COMMIT_TAG to origin"; exit 1; }
  echo "carlos_Pushed tag $CI_COMMIT_TAG to origin"

  # Check if the remote exists and get its URL if it does
  existing_url=$(git remote get-url origin_gitee 2>/dev/null)
  origin_gitee_url=$opensource_project_origin_gitee
  if [ "$existing_url" != "$origin_gitee_url" ]; then
      echo "carlos_The remote URL does not match. Updating the remote 'origin_gitee'."
      git remote remove origin_gitee
      git remote add origin_gitee "$origin_gitee_url"
      echo "carlos_The remote 'origin_gitee' has been updated to $origin_gitee_url."
  else
      echo "carlos_The remote URL matches the expected URL. Skip add origin_gitee"
  fi

  # Retry logic for pushing tag to origin_gitee
  prepare_gitee_ssh || exit 1

  max_retries=5
  attempt=0
  push_status=1 # Default to failure

  while [ $attempt -lt $max_retries ]; do
    attempt=$((attempt + 1))
    echo "carlos_Attempt $attempt of $max_retries: Pushing tag $CI_COMMIT_TAG to origin_gitee..."
    git push origin_gitee "$CI_COMMIT_TAG"
    push_status=$?

    if [ $push_status -eq 0 ]; then
      echo "carlos_Push successful."
      break # Exit loop on success
    fi

    sleep_duration=$((attempt * 60))
    echo "carlos_Push failed with status $push_status. Retrying in ${sleep_duration} seconds..."
    sleep $sleep_duration
  done

  if [ $push_status -ne 0 ]; then
    echo "carlos_Error: Could not push tag $CI_COMMIT_TAG to origin_gitee after $max_retries attempts."
    exit 1
  fi
  echo "carlos_Pushed tag $CI_COMMIT_TAG to origin_gitee"

  cd "$CI_PROJECT_DIR/ci_scripts" || { echo "carlos_Error: Could not navigate to $CI_PROJECT_DIR/ci_scripts"; exit 1; }
  echo "carlos_Navigated to $CI_PROJECT_DIR/ci_scripts"

  sudo chmod +x wechat_bot_notify.sh
  echo "carlos_Made wechat_bot_notify.sh executable"

  ./wechat_bot_notify.sh "OCS2 开源仓库版本号 $CI_COMMIT_TAG 同步成功， 请访问 https://www.lejuhub.com/highlydynamic/craic_code_repo/-/tags/$CI_COMMIT_TAG 查看详情" "$WECHAT_BOT_TOKEN"
  ./feishu_notify.sh "OCS2 开源仓库版本号 $CI_COMMIT_TAG 同步成功， 请访问 https://www.lejuhub.com/highlydynamic/craic_code_repo/-/tags/$CI_COMMIT_TAG 查看详情" "${feishu_notify_webhook:-}"
  echo "carlos_Sent success notification to WeChat"
else
  cd "$CI_PROJECT_DIR/ci_scripts" || { echo "carlos_Error: Could not navigate to $CI_PROJECT_DIR/ci_scripts"; exit 1; }
  echo "carlos_Navigated to $CI_PROJECT_DIR/ci_scripts"

  sudo chmod +x wechat_bot_notify.sh
  echo "carlos_Made wechat_bot_notify.sh executable"

  ./wechat_bot_notify.sh "OCS2 开源仓库版本号 $CI_COMMIT_TAG 同步失败，未找到对应的提交哈希 $CURRENT_COMMIT_HASH" "$WECHAT_ERR_BOT_TOKEN"
  ./feishu_notify.sh "OCS2 开源仓库版本号 $CI_COMMIT_TAG 同步失败，未找到对应的提交哈希 $CURRENT_COMMIT_HASH" "${feishu_notify_webhook:-}"
  echo "carlos_Sent failure notification to WeChat error bot"
fi

echo "carlos_Tag synchronization process completed."
