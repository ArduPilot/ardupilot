#!/bin/bash
INPUT_FILE=$1
TEMP_FILE1=${INPUT_FILE}1.tmp
TEMP_FILE2=${INPUT_FILE}2.tmp
TEMP_FILE3=${INPUT_FILE}3.tmp
ERROR_FILE=$2
OUTPUT_FILE=$3
HTML_OUTPUT_FILE=$3.html
TEST_RESULTS=$4

flattenGcovOutput() {
while read line1
do
  read line2
  read line3
  echo $line2 " " $line3 
  read junk
done < ${INPUT_FILE}
}

getRidOfCruft() {
sed '-e s/^Lines executed://g' \
    '-e s/^[0-9]\./  &/g' \
    '-e s/^[0-9][0-9]\./ &/g' \
    '-e s/^.*\/usr\/.*$//g' \
    "-e s/of.*:creating '/ /g" \
    "-e s/.gcov'//g" \
    '-e s/^.*\.$//g' 
}

flattenPaths() {
sed \
    -e 's/\/[^/][^/]*\/[^/][^/]*\/\.\.\/\.\.\//\//g' \
    -e 's/\/[^/][^/]*\/[^/][^/]*\/\.\.\/\.\.\//\//g' \
    -e 's/\/[^/][^/]*\/[^/][^/]*\/\.\.\/\.\.\//\//g' \
    -e 's/\/[^/][^/]*\/\.\.\//\//g'
}

getFileNameRootFromErrorFile() {
sed '-e s/gc..:cannot open .* file//g' ${ERROR_FILE}
}

writeEachNoTestCoverageFile() {
while read line
do
  echo "  0.00%  " ${line}
done 
}

createHtmlOutput() {
    echo "<table border="2" cellspacing="5" cellpadding="5">"
    echo "<tr><th>Coverage</th><th>File</th></tr>"
    sed "-e s/.*%   /<tr><td>&<\/td><td>/" \
        "-e s/[a-zA-Z0-9_]*\.[ch][a-z]*/<a href='file:\.\/&.gcov'>&<\/a><\/td><\/tr>/" 
    echo "</table>"
    sed "-e s/.*/&<br>/g" < ${TEST_RESULTS}
}

flattenGcovOutput | getRidOfCruft | flattenPaths  > ${TEMP_FILE1}
#getFileNameRootFromErrorFile | writeEachNoTestCoverageFile | flattenPaths > ${TEMP_FILE2}
#cat ${TEMP_FILE1}  ${TEMP_FILE2} | sort | uniq > ${OUTPUT_FILE}
cat ${TEMP_FILE1} | sort | uniq > ${OUTPUT_FILE}
#createHtmlOutput < ${OUTPUT_FILE} > ${HTML_OUTPUT_FILE}
echo >> ${OUTPUT_FILE}
cat ${ERROR_FILE} >> ${OUTPUT_FILE}
rm -f ${TEMP_FILE1} ${TEMP_FILE2} 
