#!/usr/bin/env python
'''
update a wordpress wiki page
Andrew Tridgell
May 2013

See http://codex.wordpress.org/XML-RPC_WordPress_API/Posts
'''

import xmlrpclib, sys

from optparse import OptionParser
parser = OptionParser("update_wiki.py [options] <file>")
parser.add_option("--username", help="Wordpress username", default=None)
parser.add_option("--password", help="Wordpress password", default=None)
parser.add_option("--url", help="Wordpress URL", default=None)
parser.add_option("--post-title", help="title of page", default=None)
parser.add_option("--post-id", help="ID of page", default=None)
parser.add_option("--blog-id", help="ID of wiki", default='')
parser.add_option("--list", action='store_true', help="list titles", default=False)

(opts, args) = parser.parse_args()

if opts.url is None:
    print("You must supply a base URL for the wordpress site")
    sys.exit(1)

if opts.username is None or opts.password is None:
    print("You must supply a username and password")
    sys.exit(1)

if len(args) == 0 and not opts.list:
    print("You must supply a filename containing the content to load")
    sys.exit(1)

server = xmlrpclib.ServerProxy(opts.url + '/xmlrpc.php')

def list_posts():
    '''list posts'''
    posts = server.wp.getPosts(opts.blog_id, opts.username, opts.password,
                               { 'post_type' : 'wiki', 'number' : 1000000 },
                               [ 'post_title', 'post_id' ])
    for p in posts:
        try:
            print('post_id=%s post_title="%s"' % (p['post_id'], p['post_title']))
        except Exception:
            pass

def find_post_by_title(title):
    '''find a post given the title'''
    posts = server.wp.getPosts(opts.blog_id, opts.username, opts.password,
                               { 'post_type' : 'wiki', 'number' : 1000000 },
                               [ 'post_title', 'post_id' ])
    for p in posts:
        if p['post_title'] == title:
            return p['post_id']
    return None

if opts.list:
    list_posts()
    sys.exit(0)

if opts.post_title is None and opts.post_id is None:
    print("You must supply a post_title or post_id")
    sys.exit(1)

post_id = opts.post_id

if post_id is None:
    post_id = find_post_by_title(opts.post_title)
    if post_id is None:
        print('Failed to find post_title "%s"' % opts.post_title)
        sys.exit(1)

file = open(args[0])
content = file.read()
file.close()

print("Fetching existing content")
post = server.wp.getPost(opts.blog_id, opts.username, opts.password, post_id)
if str(post['post_content']).strip() == str(content).strip():
    print("Content unchanged")
    sys.exit(0)

print("Uploading new post_content")
r = server.wp.editPost(opts.blog_id, opts.username, opts.password, post_id, { 'post_content' : content })
if r == True:
    print("Upload OK")
    sys.exit(0)
print 'result: ', r
sys.exit(1)
