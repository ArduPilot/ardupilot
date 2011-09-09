open(out,">commands.xml");
print out qq!<?xml version="1.0" encoding="us-ascii"?><Commands>!;

open(FH,"command description.txt");
while (my $line = <FH>) {

$line =~ s/ (indefinitely)//;

	$line =~ s/\s\s/\t/g;
	$line =~ s/\t+/!/g;
	$line =~ /^(....)\s*!([^!]+)!(.*)$/i;
	$cmd = $1;
	$name = $2;
	$data = $3;
	($p1,$p2,$p3,$p4) = split('!',$data);

	chomp($p1);
	chomp($p2);
	chomp($p3);
	chomp($p4);
	
	#print out $line;
	
	next if (!$cmd && !$name);
	print "$cmd ! $name ! $p1 ! $p2 ! $p3 ! $p4\n";
	print out qq!<$name><ID>$cmd</ID><p1>$p1</p1><p2>$p2</p2><p3>$p3</p3><p4>$p4</p4></$name>\n!;
}
close FH;
print out qq!</Commands>!;