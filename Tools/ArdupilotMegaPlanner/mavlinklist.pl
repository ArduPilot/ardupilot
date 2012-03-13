$dir = "C:/Users/hog/Desktop/DIYDrones/ardupilot-mega/libraries/GCS_MAVLink/include/common/";
$dir2 = "C:/Users/hog/Desktop/DIYDrones/ardupilot-mega/libraries/GCS_MAVLink/include/ardupilotmega/";

$fname = "MAVLinkTypes0.9.cs";

&doit();

# mavlink 1.0 with old structs
$dir = "C:/Users/hog/Desktop/DIYDrones/ardupilot-mega/libraries/GCS_MAVLink/include_v1.0/common/";
$dir2 = "C:/Users/hog/Desktop/DIYDrones/ardupilot-mega/libraries/GCS_MAVLink/include_v1.0/ardupilotmega/";

$fname = "MAVLinkTypes.cs";

#&doit();

<STDIN>;

exit;

sub doit
{

opendir(DIR,$dir) || die print $!;
@files2 = readdir(DIR);
closedir(DIR);

opendir(DIR,$dir2) || die print $!;
@files = readdir(DIR);
closedir(DIR);

push(@files,@files2);


push(@files,"../mavlink_types.h");

open(OUT,">$fname");

$crcs = 0;

%done = {};

print OUT <<EOF;
using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    partial class MAVLink
    {
EOF

foreach $file (@files) {
	print "$file\n";
	
	if ($done{$file} == 1) {
		next;
	}
	
	$done{$file} = 1;
	
	open(F,$dir.$file) || open(F,$dir2.$file);
	
	$start = 0;
	
	while ($line = <F>) {
		if ($line =~ /(MAVLINK_MESSAGE_LENGTHS|MAVLINK_MESSAGE_CRCS) (.*)/ && $crcs < 2) {
			print OUT "\t\tpublic byte[] $1 = new byte[] $2;\n";
			$crcs++;
		}
	
		if ($line =~ /enum (MAV_.*)/) {
			$start = 1;
			print OUT "\t\tpublic ";
		}
	
		if ($line =~ /#define (MAVLINK_MSG_ID[^\s]+)\s+([0-9]+)/) {
			if ($line =~ /MAVLINK_MSG_ID_([0-9]+)_LEN/) {   
				next;
			} else {			
				print OUT "\t\tpublic const byte ".$1 . " = " . $2 . ";\n";
				$no = $2;
			}
		}
		if ($line =~ /typedef struct(.*)/) {
			if ($1 =~ /__mavlink_system|param_union/) {
				last;
			}
			$start = 1;
			print OUT "\t\t[StructLayout(LayoutKind.Sequential,Pack=1)]\n";
			#__mavlink_gps_raw_t
			$structs[$no] = $1;
		}
		if ($start) {
			$line =~ s/MAV_CMD_NAV_//;
			
			$line =~ s/MAV_CMD_//;
            
            $line =~ s/\/\/\/</\/\/\//;
		
			$line =~ s/typedef/public/;
			$line =~ s/uint8_t/public byte/;
			$line =~ s/int8_t/public byte/;
			$line =~ s/char/public byte/;
			$line =~ s/^\s+float/public float/;
			$line =~ s/uint16_t/public ushort/;
			$line =~ s/uint32_t/public uint/;
			$line =~ s/uint64_t/public ulong/;
			$line =~ s/int16_t/public short/;
			$line =~ s/int32_t/public int/;
			$line =~ s/int64_t/public long/;
			$line =~ s/typedef/public/;
			
			$line =~ s/}.*/};\n/;
			
			if ($line =~ /\[(.*)\].*;/) { # array
				  print OUT "\t\t[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=". $1 .")] \n";
				    	$line =~ s/\[.*\]//;
				    	$line =~ s/public\s+([^\s]+)/public $1\[\]/o;
			}
			
			print OUT "\t\t".$line;
		}		
		if ($line =~ /}/) {
			$start = 0;
		}
	
	}
	
	close(F);
}

print OUT "Type[] mavstructs = new Type[] {";
for ($a = 0; $a <= 256;$a++)
{
	if (defined($structs[$a])) {
		print OUT "typeof(".$structs[$a] .") ,";
	} else {
		print OUT "null ,";
	}
}
print OUT "};\n\n";

print OUT <<EOF;
	}
}

EOF

close OUT;

}



1;