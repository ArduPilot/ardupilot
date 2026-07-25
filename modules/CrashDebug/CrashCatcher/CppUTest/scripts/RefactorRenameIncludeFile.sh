#!/bin/bash +x
dirs_to_look_in="src tests include mocks"    
from_name=$1
to_name=$2
SVN=svn

findFilesWithInclude()
{
    files_with_include=$(find $dirs_to_look_in -name "*.[hc]" -o -name "*.cpp" | xargs  grep -l "#include \"$1\"")
    if [ "$files_with_include" != "" ]
        then
            files_with_include+=" "
        fi
        
    files_with_include+=$(find $dirs_to_look_in -name "*.[hc]" -o -name "*.cpp" | xargs  grep -l "#include <$1>")
    echo $files_with_include
}

checkForPriorNameUseIncludes()
{
    files=$(findFilesWithInclude $1)

    if [ "$files" != "" ] 
        then
            echo "name already included: $1 included in ${files}"
            exit
    fi
}

checkForFileNameExists()
{
    files=$(find $dirs_to_look_in -name $1)
    if [ "$files" != "" ] 
        then
            echo "name already in use: $1 found in ${files}"
            exit
    fi
}

searchAndReplaceIncludes()
{
    files=$(findFilesWithInclude $1)
    if [ "$files" = "" ] 
        then
            echo "No files found including $1"
            exit
    fi
    echo "Changing include $1 to $2 in: $files"
    set -x
    sed -i "" -e "s/#include \"$1\"/#include \"$2\"/g" $files
    sed -i "" -e "s/#include <$1>/#include <$2>/g" $files
    set +x
}

renameIncludeFile()
{
    file=$(find $dirs_to_look_in -name $1)
    if [ $(echo $file | wc -l) != 1 ]
        then
            echo "More than one potential file to rename $file"
            exit
        fi
    set -x
    from_module_name=$(basename $1 .h)
    to_module_name=$(basename $2 .h)
    sed -i "" -e "s/$from_module_name/$to_module_name/g" $file
    path=$(dirname $file)
    $SVN mv $file $path/$2 
    set +x
}

checkForFileNameExists $to_name
checkForPriorNameUseIncludes $to_name
searchAndReplaceIncludes $from_name $to_name
renameIncludeFile $1 $2
