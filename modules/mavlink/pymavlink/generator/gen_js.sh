#!/bin/sh

for protocol in 1.0 2.0; do
 for xml in ../../message_definitions/v1.0/*.xml; do
     base=$(basename $xml .xml)
     mkdir -p javascript/implementations/mavlink_${base}_v${protocol}

     # Generate MAVLink implementation
     ../tools/mavgen.py --lang=JavaScript_NextGen --wire-protocol=$protocol --output=javascript/implementations/mavlink_${base}_v${protocol}/mavlink.js $xml || exit 1



     # Create package.json file
     cat >javascript/implementations/mavlink_${base}_v${protocol}/package.json <<EOF
 {
    "name" : "mavlink_${base}_v${protocol}",
    "version" : "0.0.3",
    "description" : "NextGen Implementation of the MAVLink protocol",
    "keywords" : ["mavlink", "arduino", "ardupilot", "ros", "robot", "uav", "drone", "awesome"],
    "homepage": "https://github.com/ardupilot/mavlink",
    "bugs" : "https://github.com/ardupilot/mavlink/issues",
    "license" : {
        "type" : "LGPL-3.0",
        "url" : "http://opensource.org/licenses/LGPL-3.0"
      },
    "contributors" : ["Bruce Crevensten <bruce.crevensten@gmail.com>","David Buzz <davidbuzz@gmail.com>"],
    "main" : "mavlink.js",
    "repository" : {
      "type" : "git",
      "url" : "https://github.com/ardupilot/mavlink"
      },
    "dependencies" : {
      "underscore" : "",
      "winston": "",
      "jspack": "file:local_modules/jspack",
      "long": "file:local_modules/long"
    },
    "devDependencies" : {
      "should" : "",
      "mocha" : "",
      "sinon" : ""
    }
}
EOF

    #symlink our custom jspack and 'long' so the package.json above can find them without needing multiple copies
    rm ./javascript/implementations/mavlink_${base}_v${protocol}/local_modules
    ln -s ../../local_modules ./javascript/implementations/mavlink_${base}_v${protocol}/

 done
done

