#! /usr/bin/python

"""
This script generates markdown files for all the MAVLink message definition XML at: 
https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0
  
The files can be imported into a gitbook to display the messages as HTML

The script runs on both Python2 and Python 3. The following libraries must be imported: lxml, requests, bs4.

The file is run in mavlink/doc/ with no arguments. It writes the files to /messages/
"""

import lxml.etree as ET
import requests
from bs4 import BeautifulSoup as bs
import re
import os # for walk


xsl_file_name = "mavlink_to_html_table_gitbook.xsl"
xml_message_definitions_dir_name = "../message_definitions/v1.0/"

output_dir = "./messages/"
output_dir_html=output_dir+"_html/"
if not os.path.exists(output_dir_html):
    os.makedirs(output_dir_html)


# File for index
index_file_name = "README.md"
index_file_name = output_dir + index_file_name

# Get XSLT
with open(xsl_file_name, 'r') as content_file:
    xsl_file = content_file.read()
xslt = ET.fromstring(xsl_file)

#initialise text for index file. 
index_text="""<!-- THIS FILE IS AUTO-GENERATED (DO NOT UPDATE GITBOOK): https://github.com/mavlink/mavlink/blob/master/doc/mavlink_gitbook.py -->
# XML Definition Files & Dialects

MAVLink definitions files can be found in [mavlink/message definitions](https://github.com/mavlink/mavlink/blob/master/message_definitions/).
These can roughly be divided into:
- [Standard definitions](#standard-definitions) - core definitions shared by many flight stacks
- [Test definitions](#test-definitions) - definitions to support testing and validation
- [Dialects](#dialects) - *protocol-* and *vendor-specific* messages, enums and commands


## Standard Definitions

The following XML definition files are considered standard/core (i.e. not dialects):

- [minimal.xml](minimal.md) - the minimum set of entities (messages, enums, MAV_CMD) required to set up a MAVLink network.
- [standard.xml](standard.md) - the standard set of entities that are implemented by almost all flight stacks (at least 2, in a compatible way).
  This `includes` [minimal.xml](minimal.md).
- [common.xml](../messages/common.md) - the set of entities that have been implemented in at least one core flight stack.
  This `includes` [standard.xml](minimal.md)

> **Note** We are still working towards moving the truly standard entities from **common.xml** to **standard.xml**
  Currently you should include [common.xml](../messages/common.md)
  
 In addition:
 - [development.xml](development.md) - XML definitions that are _proposed_ for inclusion in the standard definitions.
   These are work in progress.


## Test Definitions

The following definitions are used for testing and dialect validation:

- [all.xml](all.md) - This includes all other XML files, and is used to verify that there are no ID clashes (and can potentially be used by GCS to communicate with any core dialect).
- [test.xml](test.md) - Test XML definition file.


## Dialects  {#dialects}

MAVLink *dialects* are XML definition files that define *protocol-* and *vendor-specific* messages, enums and commands.

> **Note** Vendor forks of MAVLink may contain XML entities that have not yet been pushed into the main repository (and will not be documented).

Dialects may *include* other MAVLink XML files, which may in turn contain other XML files (up to 5 levels of XML file nesting are allowed - see `MAXIMUM_INCLUDE_FILE_NESTING` in [mavgen.py](https://github.com/ArduPilot/pymavlink/blob/master/generator/mavgen.py#L44)).
A typical pattern is for a dialect to include [common.xml](../messages/common.md) (containing the *MAVLink standard definitions*), extending it with vendor or protocol specific messages.

The dialect definitions are:
"""

index_text_trailer="""
"""

#Fix up the BeautifulSoup output so to fix build-link errors in the generated gitbook.
## BS puts each tag/content in its own line. Gitbook generates anchors using the spaces/newlines. 
## This puts displayed text content immediately within tags so that anchors/links generate properly
def fix_content_in_tags(input_html):
    #print("fix_content_in_tags was called")
    def remove_space_between_content_tags(matchobj):
        stripped_string=matchobj.group(1).strip()
        return '>%s<' % stripped_string

    input_html=re.sub(r'\>(\s+?\w+?.*?)\<', remove_space_between_content_tags, input_html,flags=re.DOTALL)
    return input_html
    
def fix_external_dialect_link(input_html):
    #print("fix_external_dialect_link was called")
    def fixupexternaldialecturls(matchobj):
        return matchobj.group(1).strip()

    input_html=re.sub(r'<a href="../../external/.*?>(.*?)</a>', fixupexternaldialecturls, input_html,flags=re.DOTALL)
    return input_html

def fix_include_file_extension(input_html):
    ## Fixes up file extension .xml.md.unlikely (easier than fixing up the XSLT to strip file extensions!)
    input_html=input_html.replace('.xml.md.unlikely','.md')
    return input_html

def fix_replace_space_marker(input_html):
    ## Above we remove hidden space. I can't seem to regexp just that type of space, so use space markers in text
    input_html=input_html.replace('xxx_space_xxx',' ')
    return input_html

def strip_text_before_string(original_text,strip_text):
    # Strip out all text before some string
    index=original_text.find(strip_text)
    stripped_string=original_text
    if index !=-1 :
        stripped_string = stripped_string[index:] 
    return stripped_string
    
def fix_add_implicit_links_items(input_html):
    # Makes screaming snake case into anchors. Special fix for MAV_CMD.
    #print("fix_add_implicit_link was called")
    def make_text_to_link(matchobj):
        #print("make_entry_to_link was called: %s" % matchobj.group(0))
        item_string = matchobj.group(2)
        item_url=item_string
        if item_string == 'MAV_CMD':
            item_url='mav_commands'
        returnString = '%s<a href="#%s">%s</a>%s' % (matchobj.group(1),item_url,item_string,matchobj.group(3))
        #print("returnstring: %s" % returnString)
        return returnString

    input_html=re.sub(r'([\`\(\s,]|^)([A-Z]{2,}(?:_[A-Z0-9]+)+)([\`\)\s\.,:]|$)', make_text_to_link, input_html,flags=re.DOTALL)
    return input_html
    
    
def inject_top_level_docs(input_html,filename):
    #Inject top level heading and other details.
    print('FILENAME (prefix): %s' % filename)
    insert_text='<!-- THIS FILE IS AUTO-GENERATED: https://github.com/mavlink/mavlink/blob/master/doc/mavlink_gitbook.py -->'
    if filename == 'common':
        insert_text+="""
# MAVLINK Common Message Set

The MAVLink *common* message set contains *standard* definitions that are managed by the MAVLink project.
The definitions cover functionality that is considered useful to most ground control stations and autopilots.
MAVLink-compatible systems are expected to use these definitions where possible (if an appropriate message exists) rather than rolling out variants in their own [dialects](../messages/README.md).

The original definitions are defined in [common.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml).

> **Tip** The common set `includes` [minimal.xml](minimal.md), which contains the *minimal set* of definitions for any MAVLink system.
  These definitions are [reproduced at the end of this topic](#minimal).

"""
    elif filename == 'minimal':
        insert_text+="""
# MAVLink Minimal Set

The MAVLink *minimal* set contains the minimal set of definitions for a viable MAVLink system.

The message set is defined in [minimal.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/minimal.xml) and is managed by the MAVLink project.

> **Tip** The minimal set is included (imported into) other xml definition files, including the [MAVLink Common Message Set (common.xml)](minimal.md).

"""
    elif filename == 'ardupilotmega':
        insert_text+="""
# Dialect: ArduPilotMega

These messages define the ArduPilot specific message set, which is custom to [http://ardupilot.org](http://ardupilot.org).

This topic is a human-readable form of the XML definition file: [ardupilotmega.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml).

> **Warning** The ArduPilot MAVLink fork of [ardupilotmega.xml](https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml) may contain messages that have not yet been merged into this documentation.
"""
    elif filename == 'development':
        insert_text+="""
# Dialect: development

This dialect contains messages that are proposed for inclusion in the [standard set](standard.md), in order to ease development of prototype implementations.
They should be considered a 'work in progress' and not included in production builds.

This topic is a human-readable form of the XML definition file: [development.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/development.xml).
"""
    elif filename == 'all':
        insert_text+="""
# Dialect: all

This dialect is intended to `include` all other [dialects](../messages/README.md) in the mavlink/mavlink repository (including [external dialects](https://github.com/mavlink/mavlink/tree/master/external/dialects#mavlink-external-dialects)).

Dialects that are in **all.xml** are guaranteed to not have clashes in messages, enums, enum ids, and MAV_CMDs.
This ensure that:
- Systems based on these dialects can co-exist on the same MAVLink network.
- A Ground Station might (optionally) use libraries generated from **all.xml** to communicate using any of the dialects.

> **Warning** New dialect files in the official repository must be added to **all.xml** and restrict themselves to using ids in their own allocated range.
  A few older dialects are not included because these operate in completely closed networks or because they are only used for tests.
  
This topic is a human-readable form of the XML definition file: [all.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/all.xml).
"""
    else:
        insert_text+='\n# Dialect: %s' % filename.rsplit('.',1)[0]
        insert_text+='\n\n*This is a human-readable form of the XML definition file: [%s](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/%s).*' % (filename, filename)
    insert_text+="""

<span></span>
> **Note** MAVLink 2 messages have an ID > 255 and are marked up using **(MAVLink 2)** in their description.

<span id="mav2_extension_field"></span>
> **Note** MAVLink 2 extension fields that have been added to MAVLink 1 messages are displayed in blue.

<style>
td {
    vertical-align:top;
}
</style>
"""
    # Include HTML in generated content
    insert_text+='\n\n{%% include "_html/%s.html" %%}' % filename
    input_html=insert_text+'\n\n'+input_html

    if filename == 'common':
        input_html+="""
# Minimal.xml {#minimal}

The minimal set of definitions required for any MAVLink system are included from [minimal.xml](minimal.md).
These are listed below.


{% include "_html/minimal.html" %}"""

    
    #print(input_html)
    return input_html
    
dialect_files = set()
all_files = set()    

for subdir, dirs, files in os.walk(xml_message_definitions_dir_name):
    #Generate html for all the XML files
    for file in files:
        print(file)
        if not file.endswith('.xml'): #only process xml files.
           continue
        xml_file_name = xml_message_definitions_dir_name+file
        with open(xml_file_name, 'r') as content_file:
            xml_file = content_file.read()
            dom = ET.fromstring(xml_file)
            transform = ET.XSLT(xslt)
            newdom = transform(dom)

            #Prettify the HTML using BeautifulSoup
            soup=bs(str(newdom), "lxml")
            prettyHTML=soup.prettify()

            #Strip out text before <html> tag in XSLT output
            prettyHTML=strip_text_before_string(prettyHTML,'<html>')
            prettyHTML = fix_content_in_tags(prettyHTML)
            
            #Replace invalid file extensions (workaround for xslt)
            prettyHTML = fix_include_file_extension(prettyHTML)

            #Replace space markers with intentional space
            prettyHTML = fix_replace_space_marker(prettyHTML)

            #Fix up links to external dialects to not be links
            prettyHTML = fix_external_dialect_link(prettyHTML)
            
            #Fix up plain text mav symbols to be internal links
            prettyHTML = fix_add_implicit_links_items(prettyHTML)     
            
            
            #Write output html file
            output_file_name_html = file.rsplit('.',1)[0]+".html"

            output_file_name_html_withdir = output_dir_html+output_file_name_html
            print("Output filename (html): %s" % output_file_name_html)

            with open(output_file_name_html_withdir, 'w') as out:
                out.write(prettyHTML)

            # Create sortable list of output file names
            #Write output markdown file
            output_file_name_prefix = file.rsplit('.',1)[0]
            all_files.add(output_file_name_prefix)
            if not file=='common.xml' and not file=='standard.xml' and not file=='minimal.xml' and not file=='test.xml' and not file=='development.xml':
                dialect_files.add(output_file_name_prefix)


# Generate the markdown files
for file_prefix in all_files:
    print(file_prefix)
    markdown_text=''
    #Inject a heading and doc-type intro (markdown format)
    markdown_text = inject_top_level_docs(markdown_text,file_prefix)

    output_file_name_md_withdir = output_dir+file_prefix+'.md'
    print("Output filename (md): %s" % output_file_name_md_withdir)

    with open(output_file_name_md_withdir, 'w') as out:
        out.write(markdown_text)

            
for the_file in sorted(dialect_files):
    index_text+='\n* [%s.xml](%s.md)' % (the_file,the_file)
index_text+='\n\n'
index_text+=index_text_trailer
            
#Write the index
with open(index_file_name, 'w') as content_file:
    content_file.write(index_text)

print("COMPLETED")



