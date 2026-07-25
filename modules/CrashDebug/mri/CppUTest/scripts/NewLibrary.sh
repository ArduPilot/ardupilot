#!/bin/bash -x
TEMPLATE_DIR=${CPPUTEST_HOME}/scripts/templates
LIBRARY=$1

if [ -e ${LIBRARY} ] ; then
	echo "The directory ${LIBRARY} already exists"
	exit 1;
fi

echo "Copy template project to  ${LIBRARY}"
cp -R ${TEMPLATE_DIR}/ProjectTemplate/Project ${LIBRARY}
find ${LIBRARY} -name \.svn | xargs rm -rf

echo "Update to the new LIBRARY name"
substituteProjectName="-e s/Project/${LIBRARY}/g -i .bak"
 
cd ${LIBRARY}

sed ${substituteProjectName} *.*
sed ${substituteProjectName} Makefile

for name in BuildTime.h BuildTime.cpp BuildTimeTest.cpp ; do
	mv Project${name} ${LIBRARY}${name}
done

cd ..

sed -e "s/DIRS = /DIRS = ${LIBRARY} /g" -i .bak Makefile

find  ${LIBRARY} -name \*.bak | xargs rm -f

echo "#include \"../${LIBRARY}/AllTests.h\"" >> AllTests/AllTests.cpp

echo "You have to manually add the library reference to the AllTests Makefile"
echo "and maybe change the order of the library builds in the main Makefile"
