#!/usr/bin/env bash

set -e
set -x

echo "Remove previous param file"
rm -f ParameterMetaDataBackup.xml
rm -f ParameterMetaData.xml

echo "Create first parameter file"
./Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter --format xml_mp
echo "Remove the last line"
sed -i -e '$d' ParameterMetaData.xml
echo "Copy parameters to the complete file"
cp ParameterMetaData.xml ParameterMetaDataBackup.xml

echo "Create the second parameter file"
./Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane --format xml_mp
echo "Remove the two first lines and the last one"
sed -i -e '1d' -e '2d' -e '$d' ParameterMetaData.xml
echo "Append parameters to the complete file"
cat ParameterMetaData.xml >> ParameterMetaDataBackup.xml

./Tools/autotest/param_metadata/param_parse.py --vehicle Rover --format xml_mp
echo "Remove the two first lines and the last one"
sed -i -e '1d' -e '2d' -e '$d' ParameterMetaData.xml
echo "Append parameters to the complete file"
cat ParameterMetaData.xml >> ParameterMetaDataBackup.xml

./Tools/autotest/param_metadata/param_parse.py --vehicle ArduSub --format xml_mp
echo "Remove the two first lines and the last one"
sed -i -e '1d' -e '2d' -e '$d' ParameterMetaData.xml
echo "Append parameters to the complete file"
cat ParameterMetaData.xml >> ParameterMetaDataBackup.xml

./Tools/autotest/param_metadata/param_parse.py --vehicle AntennaTracker --format xml_mp
echo "Remove the two first lines"
sed -i -e '1d' -e '2d' ParameterMetaData.xml
echo "Append parameters to the complete file"
cat ParameterMetaData.xml >> ParameterMetaDataBackup.xml

echo "Remove vehile param file"
rm -f ParameterMetaData.xml
echo "Rename complete param file"
mv ParameterMetaDataBackup.xml ParameterMetaData.xml
