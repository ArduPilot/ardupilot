#!/usr/bin/env bash
set -e

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

# Initialize variables
mode="format"
xml_dir="."
keep_old=0

while getopts "h?cd:o" opt; do
    case "$opt" in
    h|\?)
        show_help
        exit 0
        ;;
    c)  mode="check"
        ;;
    d)  xml_dir=${OPTARG}
        ;;
    o)  keep_old=1
        ;;
    esac
done

shift $(($OPTIND - 1))

xml_file="$1"

if [ "$xml_file" == "" ]
then
    xml_files=$(find $xml_dir -name "*.xml")
else
    xml_files="$xml_dir/$xml_file"
fi
echo "processing file(s) $xml_files"

ret=0
for f in $xml_files
do
    xmllint -format "${f}" > "${f}".new
    case "$mode" in
    format)
        if ! cmp "${f}" "${f}".new >/dev/null 2>&1
        then
            echo "formatting $f"
            if [ $keep_old -eq 1 ]
            then
                cp "${f}" "${f}".old
            fi
            cp "${f}".new "${f}"
        fi
        ;;
    check)
        if ! cmp "${f}" "${f}".new >/dev/null 2>&1
        then
            echo "file $f needs formatting - run ./scripts/format_xml.sh $f"
            ret=1
        fi
        ;;
    esac
    rm "${f}".new
done

exit $ret
