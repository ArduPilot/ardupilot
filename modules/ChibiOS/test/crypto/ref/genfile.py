import sys,getopt,string

filenames = None
fileOut = None
filePath = None
blocksize = 1024
i = 0

license ='/*\n'\
'    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.\n'\
'\n'\
'    Licensed under the Apache License, Version 2.0 (the "License");\n'\
'    you may not use this file except in compliance with the License.\n'\
'    You may obtain a copy of the License at\n'\
'\n'\
'        http://www.apache.org/licenses/LICENSE-2.0\n'\
'\n'\
'    Unless required by applicable law or agreed to in writing, software\n'\
'    distributed under the License is distributed on an "AS IS" BASIS,\n'\
'    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n'\
'    See the License for the specific language governing permissions and\n'\
'    limitations under the License.\n'\
'*/\n'

opts,args = getopt.getopt(sys.argv[1:],'f:o:p:')
for o,a in opts:
	if o == '-f':
		filenames = a
	if o == '-o':
		fileOut = a
	if o == '-p':
		filePath = a
	

filenames = filenames.split(",")

fc = open (filePath+"/"+fileOut+".c","w")
fh = open (filePath+"/"+fileOut+".h","w")
fc.write(license)
fc.write("\n#include \"hal.h\"\n\n")
fh.write(license)
fh.write("#ifndef TEST_"+fileOut.upper()+"_H_\n")
fh.write("#define TEST_"+fileOut.upper()+"_H_\n\n")
	
for fn in filenames:
	print "opening ",fn
	i = 0
	f = open(fn+".enc","rb")
	block = f.read(blocksize)
	d = fn.split("_")


	
	fc.write("const uint8_t ref"+d[0].upper()+"_"+d[1].upper()+"_"+d[2].upper()+"[]={\n")
	fh.write("extern const uint8_t ref"+d[0].upper()+"_"+d[1].upper()+"_"+d[2].upper()+"[];\n")
	str = ""
	for ch in block:
		i   += 1
		str += "0x"+format(ord(ch), '02X')+","
		if i == 10:
			str += "\n"
			i = 0
	fc.write(str)
	fc.write("\n};\n")
fh.write("#endif //TEST_"+fileOut.upper()+"_H_\n")
	
	

