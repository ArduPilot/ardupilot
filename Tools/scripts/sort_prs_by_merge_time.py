# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "gitpython",
#     "requests",
# ]
# ///

#!/usr/bin/env python3
"""
Sort GitHub pull requests by merge time (oldest first) and show the full list of
commits that entered the main repository as a result of each PR.

Usage:
    python sort_prs_by_merge_time.py pr_urls.txt > sorted_pr_list.txt

Environment variable:
    GITHUB_TOKEN - optional, increases API rate limit
"""

import sys
import os
import re
import time
import argparse
from datetime import datetime
from typing import List, Tuple, Optional, Dict, Any

import requests
import git

API_BASE = "https://api.github.com"

def extract_repo_pr(url: str) -> Optional[Tuple[str, str, int]]:
    pattern = r"https?://github\.com/([^/]+)/([^/]+)/pull/(\d+)"
    match = re.match(pattern, url)
    if not match:
        return None
    owner, repo, pr_num = match.groups()
    return owner, repo, int(pr_num)

def get_pr_merge_info(owner: str, repo: str, pr_num: int, token: Optional[str]) -> Optional[Dict[str, Any]]:
    """Return full PR data: merged_at, merge_commit_sha, merge_commit_sha, and list of commits."""
    headers = {"Accept": "application/vnd.github.v3+json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    
    url = f"{API_BASE}/repos/{owner}/{repo}/pulls/{pr_num}"
    
    max_attempts = 3
    for attempt in range(max_attempts):
        print(f"{url} attempt {attempt+1}/{max_attempts}")
        resp = requests.get(url, headers=headers)
        if resp.status_code == 200:
            data = resp.json()
            if not data.get("merged_at"):
                return None
            # Fetch the list of commits in the PR
            commits_url = data.get("commits_url")
            commit_list = []
            if commits_url:
                commit_resp = requests.get(commits_url, headers=headers)
                if commit_resp.status_code == 200:
                    for c in commit_resp.json():
                        commit_list.append(c["sha"])
            return {
                "title": data["title"],
                "author": data["user"]["login"],
                "merged_at": data["merged_at"],
                "merge_commit_sha": data.get("merge_commit_sha"),
                "commits": commit_list,
                "mergeable_state": data.get("mergeable_state", ""),
                "merged_by": data.get("merged_by", {})
            }
        elif resp.status_code == 403 and "rate limit" in resp.text.lower():
            reset_time = int(resp.headers.get("X-RateLimit-Reset", 0))
            wait_time = max(reset_time - time.time(), 0) + 5
            sys.stderr.write(f"Rate limit hit. Waiting {wait_time:.0f}s...\n")
            time.sleep(wait_time)
        else:
            sys.stderr.write(f"Error fetching PR #{pr_num}: HTTP {resp.status_code}\n")
            return None
    return None

def main():
    parser = argparse.ArgumentParser(description="Sort PRs by merge time and list all final commit hashes.")
    parser.add_argument("input_file", nargs="?", help="File containing PR URLs (one per line).")
    args = parser.parse_args()

    urls = []
    if args.input_file:
        with open(args.input_file) as f:
            urls = [line.strip() for line in f if line.strip()]
    else:
        urls = [line.strip() for line in sys.stdin if line.strip()]

    if not urls:
        sys.stderr.write("No URLs provided.\n")
        sys.exit(1)

    token = os.environ.get("GITHUB_TOKEN")
    pr_data = []  # (url, merged_at, final_commit_list)

    for url in urls:
        extracted = extract_repo_pr(url)
        if not extracted:
            sys.stderr.write(f"Invalid URL: {url}\n")
            continue

        owner, repo, pr_num = extracted
        info = get_pr_merge_info(owner, repo, pr_num, token)
        if not info:
            sys.stderr.write(f"PR #{pr_num} is not merged or inaccessible: {url}\n")
            continue

        merged_at = datetime.fromisoformat(info["merged_at"].replace('Z', '+00:00'))

        pr_data.append((url, merged_at, info))

    # Sort by merge time (oldest first)
    pr_data.sort(key=lambda x: x[1])

    print()
    print("PRs in chronological order in which they were merged:")
    print()
    # Output: for each PR, list all final commit SHAs
    for url, date, info in pr_data:
        print(f"{url}")
        print(f"\t{info['title']} by {info['author']} at {date}")
        print(f"\t{len(info['commits'])} commit(s) ending in:")
        print(f"\t{info['merge_commit_sha']}")
        print()

if __name__ == "__main__":
    main()
