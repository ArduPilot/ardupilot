#!/usr/bin/env python3
'''
tool to create lists of ArduPilot PRs for discussion
'''

import requests
from operator import itemgetter
from html.parser import HTMLParser


class PRListParser(HTMLParser):
    def __init__(self, pr_number: int):
        HTMLParser.__init__(self)
        self.recording = 0
        self.data = []
        self.pr_number = pr_number
        self.inside_pr_tag = False
        self.checks_passed = None

    def handle_starttag(self, tag, attributes):
        if self.inside_pr_tag and tag == "a":
            for name, value in attributes:
                if name == "class" and "color-fg-danger" in value:
                    self.checks_passed = False
                elif name == "class" and "color-fg-success" in value:
                    self.checks_passed = True
        if tag != "div":
            return
        if self.recording:
            self.recording += 1
            return
        for name, value in attributes:
            if name == "id" and value == "issue_" + str(self.pr_number):
                self.inside_pr_tag = True
                break
        else:
            return
        self.recording = 1

    def handle_endtag(self, tag):
        if tag == "div" and self.recording:
            self.recording -= 1

    def handle_data(self, data):
        if self.recording:
            self.data.append(data)


def check_for_label(issue: dict, desired_label: str) -> bool:
    for label in issue["labels"]:
        if label["name"] == desired_label:
            return True
    return False


def check_for_merge_on_ci_pass_passing_ci(pr: dict) -> bool:
    # so this block comment is for using the github api, but I could never get
    # the status of the commit from the api.
    # I think I would have to provide an auth key, but I'm not going to bother
    # with that for something this small.
    # Webscraping it is then!

    # get last commit sha and owner for the pr
    # pr_number = pr["number"]
    # pr_number = "21347"
    # pr_repo_name = pr["repo"]["name"]
    # pr_repo_name = "ardupilot"
    # last_commit_sha = requests.get(f"https://api.github.com/repos/ArduPilot/
    # {pr_repo_name}/pulls/{pr_number}/commits").json()[-1]["sha"]
    # print("last commit sha: ",last_commit_sha)
    # headers = {"accept": "application/vnd.github+json"}
    # test_status = requests.get(f"https://api.github.com/repos/ArduPilot/
    # {pr_repo_name}/statuses/{last_commit_sha}", headers=headers)
    # owner = "blah"

    pr_number = pr["number"]
    p = PRListParser(pr_number)
    p.feed(cached_merge_on_ci_pass_text)
    if p.checks_passed:
        return True
    return False


def print_heading_and_list(heading: str, items: list):
    print(f"\n**{heading}:**", end="\n\n")
    print("\n".join(items))


# load up all of the pages
# TO-DO: Check for proper responses
pr_search = requests.get(
    "https://api.github.com/search/issues?q=is:pr%20is:open%20label:DevCallTopic%20repo:ArduPilot/ardupilot%20sort:updated-desc"
).json()
issue_search = requests.get(
    "https://api.github.com/search/issues?q=is:issue%20label:DevCallTopic%20repo:ArduPilot/ardupilot"
).json()
merge_on_ci_pass_search = requests.get(
    "https://api.github.com/search/issues?q=is:pr%20is:open%20label:MergeOnCIPass%20repo:ArduPilot/ardupilot"
).json()
cached_merge_on_ci_pass_text = requests.get(
    "https://github.com/ArduPilot/ardupilot/pulls?q=is%3Aopen+is%3Apr+label%3AMergeOnCIPass"
).text

pr_sorted = sorted(pr_search["items"], key=itemgetter("number"))
issue_sorted = sorted(issue_search["items"], key=itemgetter("number"))
merge_on_ci_pass_sorted = sorted(
    merge_on_ci_pass_search["items"], key=itemgetter("number")
)

# create lists of each of the items to show later
pr_list = []
for pr in pr_sorted:
    pr_list.append(
"UTCXXXX - %s\n\
    - %s\n\
" % (pr["html_url"], pr["title"]))

issue_list = []
for issue in issue_search["items"]:
    if not check_for_label(issue, "ReleaseAdmin"):
        issue_list.append(
"UTCXXXX - %s\n\
    - %s\n\
" % (issue["html_url"], issue["title"]))

release_admin_list = [
    issue["html_url"]
    for issue in issue_search["items"]
    if check_for_label(issue, "ReleaseAdmin")
]
merge_on_ci_pass_list = [
    pr["html_url"]
    for pr in merge_on_ci_pass_sorted
    if check_for_merge_on_ci_pass_passing_ci(pr)
]

if merge_on_ci_pass_list:
    print_heading_and_list("MergeOnCIPass that have passed CI", merge_on_ci_pass_list)

if pr_list:
    print_heading_and_list("Pull Requests", pr_list)

if issue_list:
    print_heading_and_list("Issues", issue_list)

if release_admin_list:
    print_heading_and_list("Release Admins", release_admin_list)
