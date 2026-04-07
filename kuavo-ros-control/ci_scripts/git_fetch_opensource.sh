# Define the function to fetch with retries
fetch_with_retry() {
  local repo_path="$1"
  local git_dir="$repo_path/git_repo/"
  local max_retries=6
  local attempt=0
  local fetch_status=1 # Default to failure

  echo "Attempting to fetch in repository: $git_dir"

  # Check if the directory exists
  if [ ! -d "$git_dir" ]; then
    echo "Error: Directory $git_dir does not exist."
    return 1
  fi

  # Change directory
  cd "$git_dir" || { echo "Error: Failed to cd into $git_dir"; return 1; }

  while [ $attempt -lt $max_retries ]; do
    attempt=$((attempt + 1))
    echo "Attempt $attempt of $max_retries: Running git fetch..."
    git fetch origin
    fetch_status=$?

    if [ $fetch_status -eq 0 ]; then
      echo "Git fetch successful in $git_dir."
      cd - > /dev/null # Go back to the previous directory silently
      return 0
    fi

    local sleep_duration=$((attempt * 60))
    echo "Git fetch failed with status $fetch_status. Retrying in ${sleep_duration} seconds..."
    sleep $sleep_duration
  done

  echo "Error: Git fetch failed in $max_retries attempts."
  cd - > /dev/null # Go back to the previous directory silently
  return 1
}

# Example usage (replace with actual variable if needed elsewhere in the script)
# temp_target_repo_path="/path/to/your/repo" # Define this variable appropriately
fetch_with_retry "$temp_target_repo_path"
if [ $? -ne 0 ]; then
  echo "Script failed due to fetch error."
  exit 1
fi
exit 0
# Rest of the script...
