#!/usr/bin/perl


$file = $ARGV[0];


open(IN,$file) || die print "Failed to open file: $file : $!";

read(IN,$buffer,1);
read(IN,$buffer2,1);
if (ord($buffer) != 0x41 && ord($buffer2) != 0x50) {
	print "bad header ". $buffer ." ".$buffer2. "\n";
	exit;
}
read(IN,$buffer,1);
if (ord($buffer) != 2) {
	print "bad version";
	exit;
}

# spare
read(IN,$buffer,1);

$a = 0;

while (read(IN,$buffer,1)) {	
	$pos = (tell(IN) - 1);

	$size = ((ord($buffer) & 63));

	read(IN,$buffer,1);

	if (ord($buffer) == 0xff) {
		printf("end sentinel at %u\n", $pos);
		last;
	}

	printf("%04x: key %u size %d\n ", $pos, ord($buffer), $size + 1);

	for ($i = 0; $i <= ($size); $i++) {
		read(IN,$buffer,1);
		printf(" %02x", ord($buffer));
	}
	print "\n";
}

close IN;