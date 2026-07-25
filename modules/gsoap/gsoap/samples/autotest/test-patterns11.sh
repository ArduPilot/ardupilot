#!/bin/sh
echo "test-patterns11.log generated " `date` > test-patterns11.log
for req in databinding/examples/6/09/*/*-soap11.xml; do
  name=`basename $req`
  echo "** $name"
  echo "" >> test-patterns11.log
  echo "*** REQUEST $name ***" >> test-patterns11.log
  cat $req >> test-patterns11.log
  echo "" >> test-patterns11.log
  echo "*** RESPONSE ***" >> test-patterns11.log
  ./autotest 12288 < $req >> test-patterns11.log || echo "\n>> SKIPPED $name\n"
done
