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
if (ord($buffer) != 6) {
	print "bad version";
	exit;
}

# spare
read(IN,$buffer,1);

$a = 0;

while (read(IN,$buffer,1)) {	
	$pos = (tell(IN) - 1);

	my $key = ord($buffer);

	if ($key == 0xff) {
		printf("end sentinel at %u\n", $pos);
		last;
	}
    
	read(IN,$buffer2,1);
	read(IN,$buffer3,1);
	read(IN,$buffer4,1);

	my $itype = ord($buffer2)&0x3F;
	my $group_element = (ord($buffer2)>>6) | (ord($buffer3)<<8) | (ord($buffer4)<<16);
    
	if ($itype  == 0) { #none
		$size = 0;
		$type = "NONE";
	} elsif ($itype == 1) { #int8
		$size = 1;
		$type = "INT8";
	} elsif ($itype  == 2) { #int16
		$size = 2;
		$type = "INT16";
	} elsif ($itype == 3) { #int32
		$size = 4;
		$type = "INT32";
	} elsif ($itype == 4) { #float
		$size = 4;
		$type = "FLOAT";
	} elsif ($itype == 5) { #vector 3
		$size = 3*4;
		$type = "VECTOR3F";
	} elsif ($itype == 6) { #vector6
		$size = 6*4;
		$type = "VECTOR6F";
	} elsif ($itype == 7) { #matrix
		$size = 3*3*4;
		$type = "MATRIX6F";
	} elsif ($itype == 8) { #group
		$size = 0;
		$type = "GROUP";
	} else {
		print "Unknown type\n";
		$size = 0;
	}

	printf("%04x: type %u ($type) key %u group_element %u size %d\n ", $pos, $itype, $key, $group_element, $size);

	for ($i = 0; $i < ($size); $i++) {
		read(IN,$buffer,1);
		printf(" %02x", ord($buffer));
	}
	print "\n";
}

close IN;
