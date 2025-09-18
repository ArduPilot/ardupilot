# **Playbook: Generating git format-patch Diffs (v4)**

## **1\. Objective**

To generate a precise, machine-readable, and human-understandable diff for a file change, formatted in the style of a git format-patch output. This ensures the change is encapsulated with its metadata, the exact code modifications, and is structurally valid for use with Git tools.

## **2\. Core Principles**

* **Verifiability:** The original file content must be present in my context. I will **never** generate a patch based on an assumed or hallucinated file state.  
* **Path Integrity:** The patch must target the correct file path. I will **never** generate a patch without knowing the file path. If the path is not provided, I must ask for it.  
* **Hunk Integrity:** Each non-contiguous block of changes in a file **must** be represented by its own separate hunk (@@ ... @@ section). Combining separate changes into a single hunk will corrupt the patch.  
* **Accuracy:** The diff must represent the change perfectly. Line numbers, context lines, additions, and deletions must be exact.  
* **Format Purity:** The final output must be a raw text block, using Unix-style line endings (\\n), without any surrounding text or formatting.  
* **Structural Integrity:** The patch must be complete, including the header, body, diff stats, and the mandatory patch terminator.

## **3\. Anatomy of a format-patch File**

A patch file contains a header, a commit message, diff statistics, one or more hunks, and a terminator.

From \<commit\_hash\> \<timestamp\>  
From: Gemini \<gemini@google.com\>  
Date: \<current\_date\>  
Subject: \[PATCH\] \<Short, imperative summary of the change\>

\<Detailed explanation of the change.\>  
\---  
 path/to/the/file.ext | 4 \++--  
 1 file changed, 2 insertions(+), 2 deletions(-)

diff \--git a/path/to/the/file.ext b/path/to/the/file.ext  
index \<hash1\>..\<hash2\> \<file\_mode\>  
\--- a/path/to/the/file.ext  
\+++ b/path/to/the/file.ext  
@@ \-\<hunk\_1\_old\> \+\<hunk\_1\_new\> @@  
 ... content for the first hunk ...  
@@ \-\<hunk\_2\_old\> \+\<hunk\_2\_new\> @@  
 ... content for the second hunk ...  
\--   
2.43.0

## **4\. Step-by-Step Generation Process**

### **Step 1: Gather and Verify Information**

**Prerequisites:** Before proceeding, I must have:

1. The complete **Original File Content**.  
2. The full **File Path**.  
3. The complete **Modified File Content**.

If any information is missing, I will stop and request it.

### **Step 2-4: Header, Message, and Stats**

I will generate the metadata headers, the commit message, and the diff statistics as previously defined.

### **Step 5: Generate the Technical Diff Hunks**

This process is critical for ensuring patch integrity.

1. **Write File Headers:** I will write the diff \--git, index, \--- a/..., and \+++ b/... lines.  
2. **Identify All Change Blocks:** I will scan both files and identify every separate, non-contiguous block of added (+) and/or removed (-) lines.  
3. **Iterate Through Each Block:** For each block of changes identified in the previous step, I will perform the following procedure to generate a distinct hunk:  
   * **Establish Context:** I will select up to **3 lines** of unchanged code directly before the block and up to **3 lines** directly after it.  
   * **Calculate Hunk Header (@@ ... @@):** I will precisely calculate the start line and line count for both the original (-old,old\_count) and new (+new,new\_count) versions of the file for this specific block.  
   * **Assemble the Hunk:** I will print the calculated @@ ... @@ header, followed by the context and change lines for this block, each prefixed with the correct character ( , \-, or \+).  
4. I will repeat this process until a separate, valid hunk has been generated for every block of changes.

### **Step 6: Terminate the Patch**

1. After the last line of the final hunk, I will add a line containing only \-- .  
2. On the next line, I will add a placeholder Git version (e.g., 2.43.0).

## **6\. Self-Correction Checklist**

* \[ \] Do I have the full, original file content and path?  
* \[ \] Is the metadata and commit message correctly formatted?  
* \[ \] **(Crucial)** Have I identified all non-contiguous change blocks and created a separate, correctly calculated hunk for **each one**?  
* \[ \] Does the patch end with the correct \-- \\n\<version\> terminator?  
* \[ \] Is the final output a single, raw text block, ready for direct use?