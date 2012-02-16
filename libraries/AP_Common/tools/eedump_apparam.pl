#!/usr/bin/perl


$file = $ARGV[0];


open(IN,$file) || die print "Failed to open file: $file : $!";

read(IN,$buffer,1);
read(IN,$buffer2,1);
if (ord($buffer2) != 0x41 && ord($buffer) != 0x50) {
	print "bad header ". $buffer ." ".$buffer2. "\n";
	exit;
}
read(IN,$buffer,1);
if (ord($buffer) != 5) {
	print "bad version";
	exit;
}

# spare
read(IN,$buffer,1);

$a = 0;

while (read(IN,$buffer,1)) {	
	$pos = (tell(IN) - 1);

	if (ord($buffer) == 0xff) {
		printf("end sentinel at %u\n", $pos);
		last;
	}
    
    read(IN,$buffer2,1);
    read(IN,$buffer3,1);
    
    if (ord($buffer3) == 0) { #none
        $size = 0;
        $type = "NONE";
    } elsif (ord($buffer3) == 1) { #int8
        $size = 1;
        $type = "INT8";
    } elsif (ord($buffer3) == 2) { #int16
        $size = 2;
        $type = "INT16";
    } elsif (ord($buffer3) == 3) { #int32
        $size = 4;
        $type = "INT32";
    } elsif (ord($buffer3) == 4) { #float
        $size = 4;
        $type = "FLOAT";
    } elsif (ord($buffer3) == 5) { #vector 3
        $size = 3*4;
        $type = "VECTOR3F";
    } elsif (ord($buffer3) == 6) { #vector6
        $size = 6*4;
        $type = "VECTOR6F";
    } elsif (ord($buffer3) == 7) { #matrix
        $size = 3*3*4;
        $type = "MATRIX6F";
    } elsif (ord($buffer3) == 8) { #group
        $size = 0;
        $type = "GROUP";
    } else {
        print "Unknown type\n";
        $size = 0;
    }

	printf("%04x: type %u ($type) key %u group_element %u size %d\n ", $pos, ord($buffer3),ord($buffer),ord($buffer2), $size);

	for ($i = 0; $i < ($size); $i++) {
		read(IN,$buffer,1);
		printf(" %02x", ord($buffer));
	}
	print "\n";
}

close IN;
