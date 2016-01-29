#!/usr/bin/python
import xml.dom.minidom as minidom
from sys import exit, argv, stderr, stdout
import re
import argparse

parser = argparse.ArgumentParser(description="Format XML")
parser.add_argument('infile', nargs=1)
parser.add_argument('outfile', nargs='?')

args = parser.parse_args()

f = open(args.infile[0],'r')
text = f.read()
f.close()

dom = minidom.parseString(text)

def contains_only_text(node):
    childNodes = node.childNodes[:]
    for child in childNodes:
        if child.nodeType != child.TEXT_NODE:
            return False
    return True

def foreach_tree(doc, root, func, level=0):
    func(doc, root, level)

    childNodes = root.childNodes[:]
    for node in childNodes:
        foreach_tree(doc, node, func, level+1)

def strip_indent(doc, node, level):
    if node.nodeType == node.TEXT_NODE and re.match(r"^\s+$", node.nodeValue):
        node.parentNode.removeChild(node)
        node.unlink()

def strip_comment_whitespace(doc, node, level):
    if node.nodeType == node.COMMENT_NODE:
        node.nodeValue = re.sub(r"\s+", " ", node.nodeValue)

def strip_comments_completely(doc, node, level):
    if node.nodeType == node.COMMENT_NODE:
        node.parentNode.removeChild(node)
        node.unlink()

def strip_text_whitespace(doc, node, level):
    if node.nodeType == node.TEXT_NODE:
        node.nodeValue = re.sub(r"\s+", " ", node.nodeValue).strip()

def strip_text_completely(doc, node, level):
    if node.nodeType == node.TEXT_NODE:
        node.parentNode.removeChild(node)
        node.unlink()

def auto_indent(doc, node, level):
    if level > 0 and not contains_only_text(node.parentNode):
        node.parentNode.insertBefore(doc.createTextNode("\n%s" % (" "*4*level)), node)
        if node.nextSibling is None:
            node.parentNode.appendChild(doc.createTextNode("\n%s" % (" "*4*(level-1))))

def next_non_text_sibling(node):
    ret = node.nextSibling
    while ret is not None and ret.nodeType == node.TEXT_NODE:
        ret = ret.nextSibling
    return ret

def auto_space(doc, node, level):
    if level > 0 and node.childNodes is not None and len(node.childNodes) > 1 and next_non_text_sibling(node) is not None:
        node.parentNode.insertBefore(doc.createTextNode("\n"), node.nextSibling)

foreach_tree(dom, dom.documentElement, strip_indent)
foreach_tree(dom, dom.documentElement, strip_comment_whitespace)
foreach_tree(dom, dom.documentElement, strip_text_whitespace)
foreach_tree(dom, dom.documentElement, auto_indent)
foreach_tree(dom, dom.documentElement, auto_space)

if args.outfile is not None:
    f = open(args.outfile, 'w')
    f.truncate()
else:
    f = stdout

f.write("<?xml version='1.0'?>\n")
f.write(dom.documentElement.toxml())
f.write("\n")

f.close()
