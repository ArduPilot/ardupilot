$dir = "C:/Users/hog/Documents/Arduino/libraries/GCS_MAVLink/message_definitions/";
#$dir = "C:/Users/hog/Desktop/DIYDrones&avr/pixhawk-mavlink-c91adfb/include/common/";

opendir(DIR,$dir) || die print $!;
@files = readdir(DIR);
closedir(DIR);

open(OUT,">MAVLinkTypes.cs");

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

	if (!($file eq "common.xml" || $file eq "ardupilotmega.xml"))
	{
		next;
	}
	
	print "$file\n";
	
	open(F,$dir.$file);
	
	$start = 0;
	
	while ($line = <F>) {
		if ($line =~ /enum name="(MAV_.*)"/) {
			$start = 1;
			print OUT "\t\tpublic enum $1\n\t\t{ \n";
		}
	
		if ($line =~ /<message id="([0-9]+)" name="([^"]+)">/) {
			$name = lc($2);
		
			print OUT "\t\tpublic const byte MAVLINK_MSG_ID_".uc($name) . " = " . $1 . ";\n";
			print OUT "\t\t[StructLayout(LayoutKind.Sequential,Pack=1)]\n";
			print OUT "\t\tpublic struct __mavlink_".$name."_t\n\t\t{\n";
			$no = $1;
			
			$start = 1;
			
			#__mavlink_gps_raw_t
			$structs[$no] = "__mavlink_".$name."_t";
		} # __mavlink_heartbeat_t 
		
		$line =~ s/MAV_CMD_NAV_//;
		
		$line =~ s/MAV_CMD_//;
		
		if ($line =~ /<entry value="([0-9]+)" name="([^"]+)">/) 
		{
		
		
			print OUT "\t\t\t$2 = $1,\n";
		
		}
		
		#<field type="uint8_t" name="type">
		if ($line =~ /<field type="([^"]+)" name="([^"]+)">(.*)<\/field>/) 
		{
		
			$type = $1;
			$name = $2;
			$desc = $3;
			
			print "$type = $name\n";
		
			$type =~ s/byte_mavlink_version/public byte/;
		
			$type =~ s/array/public byte/;
			
			
		
			$type =~ s/uint8_t/public byte/;
			$type =~ s/int8_t/public byte/;
			$type =~ s/float/public float/;
			$type =~ s/uint16_t/public ushort/;
			$type =~ s/uint32_t/public uint/;
			$type =~ s/uint64_t/public ulong/;
			$type =~ s/int16_t/public short/;
			$type =~ s/int32_t/public int/;
			$type =~ s/int64_t/public long/;	

			if ($type =~ /\[(.*)\]/) { # array
				  print OUT "\t\t\t[MarshalAs(UnmanagedType.ByValArray, SizeConst=". $1 .")] \n";
				    	$type =~ s/\[.*\]//;
				    	$type =~ s/public\s+([^\s]+)/public $1\[\]/o;
			}			
			
			print OUT "\t\t\t$type $name; ///< $desc\n";
		
		}
		
		if ($start && ($line =~ /<\/message>/ || $line =~ /<\/enum>/)) {
			print OUT "\t\t};\n\n";
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

1;