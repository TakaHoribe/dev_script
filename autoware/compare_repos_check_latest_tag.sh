#!/bin/bash

# This script compares two .repos files and identifies repositories with different versions or that exist only in one file.
# If the -t option is specified, it also checks the latest semantic version tag (x.x.x format) from GitHub for repositories
# with version numbers in this format.

# Usage:
#   ./compare_repos.sh [-t] file1.repos file2.repos
# Options:
#   -t    Check the latest semantic version tag for repositories with x.x.x version format.
#   -h    Display this help message.

# Function to display help message
show_help() {
  cat << EOF
This script compares two .repos files and identifies repositories with different versions or that exist only in one file.
If the -t option is specified, it also checks the latest semantic version tag (x.x.x format) from GitHub for repositories
with version numbers in this format.

Usage:
  ./compare_repos.sh [-t] file1.repos file2.repos
Options:
  -t    Check the latest semantic version tag for repositories with x.x.x version format.
  -h    Display this help message.
EOF
}

# Get options and filenames from arguments
check_latest_tag=false
while getopts "th" opt; do
  case $opt in
    t) check_latest_tag=true ;;
    h) show_help; exit 0 ;;
    *) show_help; exit 1 ;;
  esac
done
shift $((OPTIND - 1))

file1=$1
file2=$2

# Check if files exist
if [[ ! -f "$file1" || ! -f "$file2" ]]; then
  echo "Error: Both files must exist."
  exit 1
fi

# Function to get the latest semantic version tag from a GitHub repository
get_latest_semver_tag() {
  local repo_url=$1
  local repo_name=$(echo "$repo_url" | sed -E 's|https://github.com/||;s|\.git$||')
  local tags=$(curl -s "https://api.github.com/repos/$repo_name/tags" | grep '"name":' | sed -E 's|.*"name": "([^"]+)".*|\1|')
  local latest_tag=""
  
  for tag in $tags; do
    if [[ "$tag" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      if [[ -z "$latest_tag" || "$tag" > "$latest_tag" ]]; then
        latest_tag=$tag
      fi
    fi
  done
  echo "$latest_tag"
}

# Read repository information into associative arrays
declare -A repos1
declare -A repos2

while read -r line; do
  if [[ "$line" =~ url:\ (.*) ]]; then
    url="${BASH_REMATCH[1]}"
  elif [[ "$line" =~ version:\ (.*) ]]; then
    version="${BASH_REMATCH[1]}"
    repos1["$url"]="$version"
  fi
done < <(grep -E 'url:|version:' "$file1")

while read -r line; do
  if [[ "$line" =~ url:\ (.*) ]]; then
    url="${BASH_REMATCH[1]}"
  elif [[ "$line" =~ version:\ (.*) ]]; then
    version="${BASH_REMATCH[1]}"
    repos2["$url"]="$version"
  fi
done < <(grep -E 'url:|version:' "$file2")

# Display repositories with different versions
echo "Repositories with different versions:"
for url in "${!repos1[@]}"; do
  if [[ -n "${repos2[$url]}" && "${repos1[$url]}" != "${repos2[$url]}" ]]; then
    echo "URL: $url"
    echo "  File1 version: ${repos1[$url]}"
    echo "  File2 version: ${repos2[$url]}"
    
    # Check if either version is in x.x.x format and get the latest semantic version tag if -t option is specified
    if $check_latest_tag && ([[ "${repos1[$url]}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || [[ "${repos2[$url]}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]); then
      latest_tag=$(get_latest_semver_tag "$url")
      echo "  Latest tag: $latest_tag"
    fi
  fi
done

# Display repositories only in one file
echo
echo "Repositories only in File1:"
for url in "${!repos1[@]}"; do
  if [[ -z "${repos2[$url]}" ]]; then
    echo "  URL: $url"
  fi
done

echo
echo "Repositories only in File2:"
for url in "${!repos2[@]}"; do
  if [[ -z "${repos1[$url]}" ]]; then
    echo "  URL: $url"
  fi
done
