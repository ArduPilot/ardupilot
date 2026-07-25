#!/bin/sh
echo "test-self.log generated " `date` > test-self.log
for req in *.req.xml; do
  echo "** $req"
  echo "" >> test-self.log
  echo "*** REQUEST $req ***" >> test-self.log
  cat $req >> test-self.log
  echo "" >> test-self.log
  echo "*** RESPONSE ***" >> test-self.log
  ./autotest 12288 < $req >> test-self.log || echo "\n>> ERROR $req\n"
done
