#!/bin/sh
echo "test-patterns12.log generated " `date` > test-patterns12.log
for req in databinding/examples/6/09/*/*-soap12.xml; do
  name=`basename $req`
  echo "** $name"
  echo "" >> test-patterns12.log
  echo "*** REQUEST $name ***" >> test-patterns12.log
  cat $req >> test-patterns12.log
  echo "" >> test-patterns12.log
  echo "*** RESPONSE ***" >> test-patterns12.log
  ./autotest 12288 < $req >> test-patterns12.log || echo "\n>> SKIPPED $name\n"
done
