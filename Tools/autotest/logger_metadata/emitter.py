'''
AP_FLAKE8_CLEAN
'''


class Emitter(object):
    git_sha = None
    git_branch = None


def html_comment_safe(value):
    """Keep metadata valid inside an HTML comment."""
    while '--' in value:
        value = value.replace('--', '- -')
    return value
