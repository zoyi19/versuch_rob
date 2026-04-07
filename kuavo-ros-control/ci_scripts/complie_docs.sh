#!/bin/bash
set -e

echo "documentation_compilation: Documentation compilation process started. Checking for changes in Markdown files."

# Define paths and variables
SOURCE_DIR="."
PAGES_DIR="./src/pages"
INDEX_FILE="./src/pages/index.md"
TARGET_DOCS_PATH="./new-docs/basic_usage"
TARGET_EN_DOCS_PATH="./new-docs/i18n/en/docusaurus-plugin-content-docs/current/basic_usage"

# Determine the target branch, defaulting to "beta" if not provided
TARGET_BRANCH="${CI_MERGE_REQUEST_TARGET_BRANCH_NAME:-beta}"

# Define the list of trigger files
TRIGGER_FILES=(ci_scripts/sync_docs.py docs/sidebars.js ci_scripts/complie_docs.sh)

# Fetch the target branch
echo "Fetching the target branch: $TARGET_BRANCH"
git fetch origin "$TARGET_BRANCH"

# Get the list of changed files
changed_files=$(git diff --name-only origin/"$TARGET_BRANCH"...HEAD)

echo "After get the changed_files"

# Check if any Markdown files have changed
if echo "$changed_files" | grep -E '\.md$' > /dev/null; then
    markdown_changed=true
else
    markdown_changed=false
fi

# Check if any of the trigger files have changed
trigger_files_changed=false
for file in "${TRIGGER_FILES[@]}"; do
  if [[ "$changed_files" == *"$file"* ]]; then
    trigger_files_changed=true
    break
  fi
done

# --- Debugging output ---
echo "Changed files: $changed_files"
echo "Markdown changed: $markdown_changed"  # Will output 'true' or 'false'
echo "Trigger files changed: $trigger_files_changed" # Will output 'true' or 'false'
# ------------------------

# Perform the action if either condition is met
if $markdown_changed || $trigger_files_changed; then
    echo "changes_detected: Detected changes in Markdown files or trigger files. Proceeding with documentation compilation."

    # Install necessary dependencies
    npm install -f

    # Synchronize documentation
    echo "同步文档开始..."
    python3 "$SOURCE_DIR/ci_scripts/sync_docs.py" "$SOURCE_DIR" "$TARGET_DOCS_PATH" "$TARGET_EN_DOCS_PATH"
    echo "同步文档完成"

    # Ensure pages directory exists
    if [[ ! -d "$PAGES_DIR" ]]; then
        echo "creating_pages_directory: Pages directory does not exist. Creating: $PAGES_DIR"
        mkdir -p "$PAGES_DIR"
    else
        echo "pages_directory_exists: Pages directory already exists: $PAGES_DIR"
    fi

    # Ensure index file exists
    if [[ ! -f "$INDEX_FILE" ]]; then
        echo "creating_index_file: Index file does not exist. Creating: $INDEX_FILE"
        echo "" > "$INDEX_FILE"
    else
        echo "index_file_exists: Index file already exists: $INDEX_FILE"
    fi

    # Build the documentation
    npm run build

    echo "compilation_completed: Documentation compilation process completed successfully."
else
    echo "no_changes_detected: No Markdown file changes detected between the current branch and the target branch ($TARGET_BRANCH). Skipping documentation compilation."
fi
