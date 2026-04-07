#!/bin/bash
set -e 

SOURCE_DIR="$CI_PROJECT_DIR"
SOURCE_DIR_NAME="${SOURCE_DIR##*/}"
TARGET_REPO="kuavo-wiki-website"
TARGET_REPO_PATH="/Users/carlos/Desktop/kuavo-wiki-website"
TARGET_DOCS_PATH="/Users/carlos/Desktop/kuavo-wiki-website/docs/basic_usage"
TARGET_DOCS_COPY_PATH="$TARGET_DOCS_PATH/$SOURCE_DIR_NAME"
TARGET_EN_DOCS_PATH="/Users/carlos/Desktop/kuavo-wiki-website/i18n/en/docusaurus-plugin-content-docs/current/basic_usage"
TARGET_EN_DOCS_COPY_PATH="$TARGET_EN_DOCS_PATH/$SOURCE_DIR_NAME"
echo "carlos_current_commit hash: $CI_COMMIT_SHA"
COMMIT_URL="https://www.lejuhub.com/highlydynamic/kuavo-ros-control/-/commit/$CI_COMMIT_SHA"


if [ ! -d "$TARGET_REPO_PATH" ]; then
    echo "carlos_target_repository_not_found: Target repository not found. Cloning..."
    git clone -b $TARGET_BRANCH ssh://git@www.lejuhub.com:10026/carlos/kuavo-wiki-website.git "$TARGET_REPO_PATH"
    cd "$SOURCE_DIR"
else
    echo "carlos_target_repository_found: Target repository found. Updating..."
    cd "$TARGET_REPO_PATH"
    echo "carlos_fetching_origin: Fetching origin $TARGET_BRANCH..."
    git fetch origin $TARGET_BRANCH
    echo "carlos_resetting_to_origin: Resetting to origin/$TARGET_BRANCH..."
    git checkout $TARGET_BRANCH
    git reset --hard origin/$TARGET_BRANCH
    git pull origin $TARGET_BRANCH
    echo "carlos_removing_target_docs_copy_path: Removing $TARGET_DOCS_COPY_PATH..."
    rm -rf "$TARGET_DOCS_COPY_PATH"
    echo "carlos_copying_target_docs_path: Copying $TARGET_DOCS_PATH to $TARGET_DOCS_COPY_PATH..."

    echo "carlos_copying_target_en_docs_path: Copying $TARGET_EN_DOCS_PATH to $TARGET_EN_DOCS_COPY_PATH..."
    rm -rf "$TARGET_EN_DOCS_COPY_PATH"
    echo "carlos_copying_target_en_docs_path_done: Copying $TARGET_EN_DOCS_PATH to $TARGET_EN_DOCS_COPY_PATH..."

    cd "$SOURCE_DIR"
fi

cp ./docs/sidebars.js $TARGET_REPO_PATH/sidebars.js

echo "开始同步文档..."
python3 "$SOURCE_DIR/ci_scripts/sync_docs.py" "$SOURCE_DIR" "$TARGET_DOCS_PATH" "$TARGET_EN_DOCS_COPY_PATH"
echo "文档同步完成"

cd "$TARGET_REPO_PATH"
has_changes=$(git status --porcelain)
if [ -z "$has_changes" ]; then
    echo "carlos_no_changes_to_commit: No changes to commit. Skipping git add and commit."
else
    echo "carlos_committing_changes: Committing changes..."
    git add .
    git commit -m "docs: Sync docs files from kuavo-ros-control repo Source: $COMMIT_URL"
    git push origin $TARGET_BRANCH
fi

