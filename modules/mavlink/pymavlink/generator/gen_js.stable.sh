#!/bin/sh

for protocol in 1.0 2.0; do
 for xml in ../../message_definitions/v1.0/*.xml; do
     base=$(basename $xml .xml)
     mkdir -p javascript_stable/implementations/mavlink_${base}_v${protocol}

     # Generate MAVLink implementation
     ../tools/mavgen.py --lang=JavaScript_Stable --wire-protocol=$protocol --output=javascript_stable/implementations/mavlink_${base}_v${protocol}/mavlink.js $xml || exit 1

     # Create package.json file
     cat >javascript_stable/implementations/mavlink_${base}_v${protocol}/package.json <<EOF
 {
    "name" : "mavlink_${base}_v${protocol}",
    "version" : "0.0.1",
    "description" : "Stable JS Implementation of the MAVLink protocol",
    "keywords" : ["mavlink", "arduino", "megapilot", "ros", "robot", "uav", "drone", "awesome"],
    "homepage": "https://github.com/mavlink/mavlink",
    "bugs" : "https://github.com/mavlink/mavlink/issues",
    "license" : {
        "type" : "LGPL-3.0",
        "url" : "http://opensource.org/licenses/LGPL-3.0"
      },
    "contributors" : ["Bruce Crevensten <bruce.crevensten@gmail.com>"],
    "main" : "mavlink.js",
    "repository" : {
      "type" : "git",
      "url" : "https://github.com/mavlink/mavlink"
      },
    "dependencies" : {
      "underscore" : "",
      "jspack":"",
      "winston": ""
    },
    "devDependencies" : {
      "should" : "",
      "mocha" : "",
      "sinon" : ""
    }
}
EOF

 done
done
