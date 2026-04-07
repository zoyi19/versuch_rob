#!/bin/bash
# NOTE: This script is no longer used. GitHub sync is now handled by the independent 
# sync_to_github job in .gitlab-ci.yml which includes compilation verification.
# This file is kept for reference only.

echo "WARNING: This script is deprecated. GitHub sync is now handled by the sync_to_github CI job."
exit 0

cd $temp_target_repo_path_github/git_repo
echo "Current working directory: $(pwd)"

# Create or switch to branch
if git show-ref --verify --quiet refs/heads/$CI_COMMIT_BRANCH; then
    echo "Branch $CI_COMMIT_BRANCH exists in GitHub repo. Checking out.";
    git checkout -f $CI_COMMIT_BRANCH;
else
    echo "Branch $CI_COMMIT_BRANCH does not exist in GitHub repo. Creating and pushing the branch.";
    git checkout -b $CI_COMMIT_BRANCH;
fi

# Reset and clean  
git reset --hard origin_github/$CI_COMMIT_BRANCH 2>/dev/null || echo "Branch doesn't exist on remote yet"
git clean -df

# Check if the remote exists and get its URL if it does
existing_url=$(git remote get-url origin_github 2>/dev/null)

if [ "$existing_url" ]; then
    echo "The remote 'origin_github' already exists."
    # Check if the existing URL is the one we want
    if [ "$existing_url" != "$origin_github_url" ]; then
        echo "The remote URL does not match. Updating the remote 'origin_github'."
        # Remove the old remote and add the new one
        git remote remove origin_github
        git remote add origin_github "$origin_github_url"
        echo "The remote 'origin_github' has been updated to $origin_github_url."
    else
        echo "The remote URL matches the expected URL. Skip add origin_github"
    fi
else
    # If the remote doesn't exist, add it
    git remote add origin_github "$origin_github_url"
    echo "The remote 'origin_github' has been added with URL $origin_github_url."
fi

# Remove files larger than 80MB (excluding .git directory)
echo "Checking for files larger than 80MB..."
large_files=$(find . -type f -size +80M ! -path "./.git/*" 2>/dev/null)

if [ -n "$large_files" ]; then
    echo "Found files larger than 80MB that will be removed:"
    echo "$large_files"
    
    # Remove each large file and echo the deletion
    echo "$large_files" | while read -r file; do
        if [ -f "$file" ]; then
            file_size=$(du -h "$file" | cut -f1)
            echo "Removing large file: $file (${file_size})"
            rm -f "$file"
        fi
    done
    
    echo "Completed removal of large files."
else
    echo "No files larger than 80MB found."
fi

# Add all files and commit
git add . && git add -u

# Check if there are changes to commit
if ! git diff --cached --quiet; then
    echo "Committing changes to GitHub..."
    git commit -am "sync $CI_PROJECT_URL/commit/$CI_COMMIT_SHA"
else
    echo "No changes to commit for GitHub sync."
fi

cd ~
echo "Current working directory: $(pwd)"
cd -
echo "GitHub Public Key Fingerprint: "
ssh-keygen -lf ~/.ssh/id_rsa.pub 2>/dev/null || echo "No SSH key found"
echo "GitHub Public Key: "
cat ~/.ssh/id_rsa.pub 2>/dev/null || echo "No SSH public key found"

max_attempts=3
push_exit_code=1 # Initialize to failure status

for attempt in $(seq 1 $max_attempts); do
  echo "Attempt $attempt/$max_attempts: Pushing branch $CI_COMMIT_BRANCH to origin_github..."
  git push -f origin_github $CI_COMMIT_BRANCH
  push_exit_code=$?

  if [ $push_exit_code -eq 0 ]; then
    echo "Push successful on attempt $attempt."
    break # Exit loop on success
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
   bash $CI_PROJECT_DIR/ci_scripts/wechat_bot_notify.sh "OCS2 开源仓库(GitHub)分支 $CI_COMMIT_BRANCH 同步成功" "$WECHAT_BOT_TOKEN"
   bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "OCS2 开源仓库(GitHub)分支 $CI_COMMIT_BRANCH 同步成功" "${feishu_notify_webhook:-}"
else
    bash $CI_PROJECT_DIR/ci_scripts/wechat_bot_notify.sh "@黄怀贤 OCS2 开源仓库(GitHub)分支 $CI_COMMIT_BRANCH 同步失败，请跟进。 请访问 ${CI_PIPELINE_URL} 来查看" "$WECHAT_ERR_BOT_TOKEN"
    bash "$CI_PROJECT_DIR/ci_scripts/feishu_notify.sh" "OCS2 开源仓库(GitHub)分支 $CI_COMMIT_BRANCH 同步失败，请跟进。 请访问 ${CI_PIPELINE_URL} 来查看" "${feishu_notify_webhook:-}"
fi
