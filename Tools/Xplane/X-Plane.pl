#!/usr/bin/perl -w

# Heavily modified and adapted by Jason Short
# from http://www.perl.com - Autopilots in Perl
# By Jeffrey Goff on July 12, 2004

use strict;
use IO::Socket;
use IO::Select;
$| = 1;

use constant DATA_num_channels => 113;
use constant DATA_num_elements => 8;
use constant DATA_header_size	=> 5;
use constant DATA_packet_size	=> DATA_num_channels * DATA_num_elements + DATA_header_size;
use constant DATA_max_element	=> 7;
use constant DATA_element_size => (DATA_num_elements + 1) * 4; # 4-byte elements
use constant DATA_inbound_pack => "A4c";
use constant DATA_inbound_header => ('DATA',0);

my $rh;
my $x_plane_ip		= '127.0.0.1';
my $ser_port	 	= 5331;
my $receive_port	= 49005;
my $transmit_port	= 49000;
my $throttle_test = 0;
my $check_a = 0;
my $check_b = 0;
my $count = 0;

my $roll_out = 0;
my $pitch_out = 0;
my $throttle_out = 0;
my $rudder_out = 0;
my $wp_distance = 0;
my $wp_index = 0;
my $control_mode = 0;
my $control_mode_alpha;
my $bearing_error = 0;
my $alt_error = 0;
my $energy_error = 0;

my $extraInt = 0; # for debugging
my $int_1 = 0;
my $int_2 = 0;
my $int_3 = 0;
my $int_4 = 0;

my $diyd_header = "DIYd";

my $Xplane_in_sock = IO::Socket::INET->new(
	LocalPort => $receive_port,
	LocalAddr => $x_plane_ip,
	Proto		 => 'udp')
	or die "error creating receive port for $x_plane_ip: $@\n";

my $receive = IO::Select->new();


my $Xplane_out_sock = IO::Socket::INET->new(
	PeerPort	=> $transmit_port,
	PeerAddr	=> $x_plane_ip,
	Proto		 => 'udp')
	or die "error creating transmit port for $x_plane_ip: $@\n";

my $transmit = IO::Select->new();



#open sockets on 7070
my $arduSocket = IO::Socket::INET->new(	Proto => 'tcp',
											PeerAddr =>	'127.0.0.1',
										   	PeerPort => $ser_port
										   	) or die print "Can not connect to Serial $!";
										   	#,Type = 'SOCK_STREAM'
										   	
										   	
$receive->add($Xplane_in_sock);
$receive->add($arduSocket);
$transmit->add($Xplane_out_sock);
$arduSocket->autoflush(1);




my $DATA_buffer = {};

# These formats are represented by X-Plane as floating point
my @float_formats = qw( f deg mph pct );


my $DATA_packet = {
	# The outer hash reference contains a sparse array of data frames
	0 => {
		# The inner hash reference contains a sparse array of data elements
		0 => { type => 'f', label => 'Frame Rate'},
	},
	
	3 => {
		6 => { type => 'f', label => 'Air Speed'},
		7 => { type => 'f', label => 'Ground Speed'},
	},

	8 => {
		0 => { type => 'deg', label => 'Pitch'},
		1 => { type => 'deg', label => 'Roll'},
		2 => { type => 'deg', label => 'heading'},
	},
	
	11 => {
		0 => { type => 'f', label => 'elev'},
		1 => { type => 'f', label => 'aileron'},
	},
	
	18 => {
		0 => { type => 'deg', label => 'Pitch'},
		1 => { type => 'deg', label => 'Roll'},
		2 => { type => 'deg', label => 'heading'},
	},
	
	20 => {
		0 => { type => 'deg', label => 'Latitude'},
		1 => { type => 'deg', label => 'Longitude'},
		2 => { type => 'f', label => 'Altitude'},
	},
	
	25 => {
		0 => { type => 'deg', label => 'Throttle cmd'},
	},
	
	26 => {
		0 => { type => 'deg', label => 'Throttle'},
	},

};


# X-Plane uniformly sends 4-byte floats outbound,
# but accepts a mixture of floats and integers inbound.

sub create_pack_strings {
	for my $row (values %$DATA_packet) {
		$row->{unpack} = 'x4';
		$row->{pack} = 'l';
		for my $j (0..DATA_max_element) {
			if(exists $row->{$j}) {
				my $col = $row->{$j};
				$row->{pack} .= (grep { $col->{type} eq $_ } @float_formats) ? 'f' : 'l';
				$row->{unpack} .= 'f';
			}
			else {
				$row->{pack} .= 'f';
				$row->{unpack} .= 'x4';
			}
		}
	}
}


# {{{ Transmit a message
sub transmit_message {
	my ($socket,$pack_format,@message) = @_;
	my ($server) = $socket->can_write(60);
	$server->send(pack($pack_format,@message));
}


sub receive_DATA 	{
	my ($message) = @_;
	$DATA_buffer = { };
	for (my $i = 0; $i < (length($message) - &DATA_header_size - 1) / DATA_element_size; $i++) {
		my $channel = substr($message, $i * DATA_element_size + DATA_header_size, DATA_element_size);

		my $index = unpack "l", $channel;
		next unless exists $DATA_packet->{$index};

		my $row = $DATA_packet->{$index};
		my @element = unpack $row->{unpack}, $channel;
		my $ctr = 0;
		for my $j (0..DATA_max_element) {
			next unless exists $row->{$j};
			my $col = $row->{$j};
			$DATA_buffer->{$index}{$j} = $element[$ctr];
			$ctr++;
		}
	}
}



# {{{ transmit_DATA
sub transmit_DATA {
	my ($socket, @message) = @_;
	my $pack_str = DATA_inbound_pack;
	for(my $packet = 0;
		$packet < @message;
		$packet += (DATA_num_elements + 1)) {
		$pack_str .= $DATA_packet->{$message[$packet]}{pack};
	}
	transmit_message($socket,$pack_str,DATA_inbound_header,@message,0);
}


# {{{ Fill in a DATA channel
sub _fill_channel {
	my ($packet) = @_;
	my @buffer = (-999) x DATA_num_elements;
	for(0..7) {
		$buffer[$_] = $packet->{$_} if defined $packet->{$_};
	}
	return @buffer;
}


# {{{ Send outbound messages if there are any
sub transmit_socket {
	my ($socket,$ch) = @_;
	if($ch eq 'g') {
		transmit_DATA($socket, 12, $DATA_buffer->{12}{0} ? 0 : 1,(-999) x 7);

	}elsif($ch eq 't') {
		transmit_DATA($socket, 25, $throttle_out,$throttle_out,$throttle_out,$throttle_out,(-999) x 4);

	}elsif($ch eq 'c') {
		transmit_DATA($socket, 11, $pitch_out, $roll_out, $rudder_out, (-999), ($roll_out * 5) , -999, -999, -999 );

	}elsif($ch eq 'j') {
		transmit_DATA($socket, 8, $pitch_out, $roll_out, $rudder_out , (-999) x 5);

	}elsif($ch eq 'i') {
		my @engine = map { $_ != -999 ? $_ + 0.1 : -999 } _fill_channel($DATA_buffer->{23});
		transmit_DATA( $socket, 23, @engine );

	}elsif($ch eq 'k') {
		my @engine =
			map { $_ != -999 ? $_-0.1 : -999 }
			_fill_channel($DATA_buffer->{23});
		transmit_DATA( $socket, 23, @engine );
	}
}


sub parseXplane {
	
	#convert data	
	my $lat 		= int($DATA_buffer->{20}{0} * 10000000);
	my $lng 		= int($DATA_buffer->{20}{1} * 10000000);
	my $altitude 	= int($DATA_buffer->{20}{2} * 3.048);		# altitude to meters
	my $speed 		= int($DATA_buffer->{3}{7} * 044.704);		# spped to m/s * 100
	my $airspeed 	= int($DATA_buffer->{3}{6} * 044.704);		# speed to m/s * 100 
	my $pitch 		= int($DATA_buffer->{18}{0} * 100);
	my $roll 		= int($DATA_buffer->{18}{1} * 100);
	my $heading 	= int($DATA_buffer->{18}{2} * 100);
	
	# format and publish the IMU style data to the Ardupilot
	my $outgoing = pack("CCs<4", 8,4, $roll, $pitch, $heading, $airspeed);
	$check_a = $check_b = 0;
	for( split(//,$outgoing) )
	{
		$check_a += ord;
		$check_a %= 256;
		$check_b += $check_a;
		$check_b %= 256;
		#print ord . "\n";
	}
	$outgoing = pack("a4CCs<4CC","DIYd", 8,4, $roll, $pitch, $heading, $airspeed, $check_a,$check_b);
	$arduSocket->send($outgoing);

	$count++;
	if ($count == 10){
		# count of 10 = 5 hz, 50 = 1hz
		$count = 0;
		$outgoing = pack("CCl<2s<3", 14,3, $lng, $lat, $altitude, $speed, $heading);
		$check_a = $check_b = 0;
		for( split(//,$outgoing) )
		{
			$check_a += ord;
			$check_a %= 256;
			$check_b += $check_a;
			$check_b %= 256;
			#print ord . "\n";
		}
		$outgoing = pack("a4CCl<2s<3CC","DIYd", 14,3, $lng, $lat, $altitude, $speed, $heading, $check_a, $check_b);
		$arduSocket->send($outgoing);
	}
	#print "lat = $lat, long = $lng, alt = $altitude, ASpeed = $airspeed, $heading\n";
	#print "alt = $altitude,speed = $speed \n";
	
	$count = 0 if ($count > 50);
}

sub parseMessage
{
	my ($data) = @_;
	my @out = unpack("s8C2",$data);
	
	$roll_out = $out[0] *1 / 4500  if (defined $out[0]);
	$roll_out = 1 if($roll_out > 1);
	$roll_out = -1 if($roll_out < -1);
	
	$pitch_out = $out[1] *1 / 4500  if (defined $out[1]);
	$pitch_out = 1 if($pitch_out > 1);
	$pitch_out = -1 if($pitch_out < -1);


	$throttle_out = $out[2] / 100  if (defined $out[2]);
	

	$rudder_out = $out[3] * 1 / 4500  if (defined $out[3]);
	$rudder_out = 1 if($rudder_out > 1);
	$rudder_out = -1 if($rudder_out < -1);

	# normal reading
	$wp_distance	= $out[4] if (defined $out[4]);
	$bearing_error 	= $out[5] / 100  if (defined $out[5]);
	$alt_error 		= $out[6] if (defined $out[6]);
	$energy_error 	= $out[7] if (defined $out[7]);
	$wp_index 		= $out[8] if (defined $out[8]);
	$control_mode 	= $out[9] if (defined $out[9]);

	#debugging - send any three ints for debugging!
	$int_1	= $out[4] * 1  if (defined $out[4]);
	$int_2 	= $out[5] * 1  if (defined $out[5]);
	$int_3 	= $out[6] * 1  if (defined $out[6]);
	$int_4 	= $out[7] * 1  if (defined $out[7]);
	
	#output_int((int)airspeed);							// 	4	bytes 8,9
	#output_int((int)airspeed_error);					// 	5	bytes 10,11
	#output_int((int)altitude_error);					// 	6	bytes 12,13
	#output_int((int)energy_error);						// 	7	bytes 14,15

	SWITCH: {
	  $control_mode == 0 && do { 	$control_mode_alpha = "Manual"; last SWITCH; };
	  $control_mode == 1 && do { 	$control_mode_alpha = "CIRCLE"; last SWITCH; };
	  $control_mode == 2 && do { 	$control_mode_alpha = "STABILIZE"; last SWITCH; };
	  $control_mode == 5 && do { 	$control_mode_alpha = "FLY_BY_WIRE_A"; last SWITCH; };
	  $control_mode == 6 && do { 	$control_mode_alpha = "FLY_BY_WIRE_B"; last SWITCH; };
	  $control_mode == 10 && do { 	$control_mode_alpha = "AUTO"; last SWITCH; };
	  $control_mode == 11 && do { 	$control_mode_alpha = "RTL"; last SWITCH; };
	  $control_mode == 12 && do { 	$control_mode_alpha = "LOITER"; last SWITCH; };
	  $control_mode == 13 && do { 	$control_mode_alpha = "TAKEOFF"; last SWITCH; };
	  $control_mode == 14 && do { 	$control_mode_alpha = "LAND"; last SWITCH; };
	}

	$bearing_error = sprintf("%.1f", $bearing_error);
	if ($count){
		#printf("roll: %.3f,  pitch: %.3f,   thr: %.3f, rud: %.3f \n", $roll_out,$pitch_out,$throttle_out,$rudder_out);
		print "wp_dist:$wp_distance \tbearing_err:$bearing_error \talt_error:$alt_error \tenergy_err:$energy_error \tWP:$wp_index \tmode:$control_mode_alpha\r";
		#print "wp_dist:$wp_distance \tbearing_err:$bearing_error \tloiter_sum:$int_3 \tLoiter total:$int_4 \tWP:$wp_index \tmode:$control_mode_alpha\r";
		
		#print "next_WP.alt:$int_1 \toffset_altitude:$int_2 \talt_error:$alt_error \ttarget_altitude:$int_4 \tWP:$wp_index \tmode:$control_mode_alpha\n";
		#print "test_alt:$int_1 \ttarget_altitude:$int_2 \tnext_wp.alt:$int_3 \toffset_altitude:$int_4 \tWP:$wp_index \tmode:$control_mode_alpha\n";
		#print "throttle_cruise: $int_1, landing_pitch: $int_2, throttle: $throttle_out, altitude_error: $int_3,  WP: $wp_index,   mode: $control_mode\n";
	}
	transmit_socket($transmit,'t');
	transmit_socket($transmit,'j');
	transmit_socket($transmit,'c');
}

sub readline {
	my ($rh) = @_;
	my $temp = "";
	my $step = 0;
	my $message = "";
	
	while (1) {
		$rh->recv($message,1);
		$temp .= $message;
		
		if ($temp =~ /^AAA/) {
			$step++;
			if ($temp =~ "\n" && $step >= 19) { last; } # data
		}
		if ($temp =~ /\n/ && $step == 0) { # normal message
			last;
		}
	}
	return $temp;
}

sub main_loop {
	my ($receive,$transmit) = @_;
	my $recv_data = "";

	while(1) {
		my ($rh_set) = IO::Select->select($receive, undef, undef, .1);


		foreach $rh (@$rh_set) {
			if ($rh == $Xplane_in_sock) {
				my $message;
				$rh->recv($message,DATA_packet_size);
				receive_DATA($message);
				parseXplane();
				
			}elsif ($rh == $arduSocket) {
				my $message = '';
				
				$message = &readline($rh);
				
				if($message =~ '^AAA'){
					$message = substr $message, 3, 18; 
					parseMessage($message);
				}elsif($message =~ '^MSG'){
					print "$message\n";
				}else{
					#print "er:$message \n";
				}
				$rh->flush();
				#print "$message \n";
			}
		}

	}
}

create_pack_strings();
main_loop($receive, $transmit);
