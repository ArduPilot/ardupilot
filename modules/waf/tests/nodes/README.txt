Node concepts, without a particular order
-----------------------------------------

when finding a source dir:
 1. eliminate src files that do not exist anymore
 2. look in os.listdir or try os.stat

when declaring a build file:
 1. construct the folder structure in the build dir to avoid the manual mkdirs
 2. create the nodes for the src dir too if the folders exist

when looking for a resource:
 1. return either a source file or a build file, the build file will have the priority
 2. find the intermediate source and build nodes

The calls to os.listdir should be cached somehowa cache for os.listdir

Eliminate source dir nodes when they do not exist anymore

When build nodes do not exist, do not delete them but remove the signatures

Using the testcase
------------------

Execute
../waf distclean configure build

