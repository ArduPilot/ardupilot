clc
clear

% This is a example of using MATLAB to test a sensor drive using TCP
% we send a NMEA messages at 1hz, note that this assumes that SITL is
% running at real time.

% NMEA is a basic one way protocol but more complex protocols can be
% implemented in the same way

% Use the same TCP/UDP libbary that is used for MALTAB SITL
addpath(genpath('../../SITL/examples/JSON/MATLAB/tcp_udp_ip_2.0.6'))

% if this dosn't work try the MALTAB SITL example first
pnet('closeall')

% Init the TCP port, 5763 is serial 2
u = pnet('tcpconnect','127.0.0.1',5763);

flipflop = true;
while(true)
    
    if flipflop
        % send MTW temp message
        water_temp = 10 + randn();
        NMEA_string = sprintf('$YXMTW,%0.1f,C',water_temp);
    else 
        % send VHW speed message
        water_speed_knots = 5 + randn()*2;
        water_speed_kph = water_speed_knots * 1.852;
        NMEA_string = sprintf('$VWVHW,,T,,M,%0.1f,N,%0.1f,F',water_speed_knots,water_speed_kph);
    end
    flipflop = ~flipflop;

    % Calculate the correct checksum
    NMEA_string = add_checksum(NMEA_string);

    % send to ap
    pnet(u,'printf',sprintf('%s\r\n',NMEA_string));
    pnet(u,'writepacket');

    % also print to MATLAB console
    fprintf("%s\n",NMEA_string);

    % 1hz (ish)
    pause(1);
end

function NMEA_string_out = add_checksum(NMEA_string_in)  
    checksum = uint8(0);
    for i = 2:numel(NMEA_string_in)
        checksum = bitxor(checksum,uint8(NMEA_string_in(i)),'uint8');
    end    
    NMEA_string_out = sprintf('%s*%s',NMEA_string_in,dec2hex(checksum));
end
