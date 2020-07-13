function [pwm, reset] = AP_receve(time)
global u
persistent connected frame_time last_sim_time frame_count last_SITL_frame past_time

if isempty(u) || time == 0
    pnet('closeall') % close any connections left open from past runs
    u = pnet('udpsocket',9002);
    pnet(u,'setwritetimeout',1);
    pnet(u,'setreadtimeout',0);
    
    connected = false;
    frame_time = tic;
    last_sim_time = 0;
    frame_count = 0;
    last_SITL_frame = -1;
    past_time = -1;
end
if past_time == time
    error('Receve repeat time');
end
past_time = time;

print_frame_count = 1000; % print the fps every x frames
bytes_read =  4 + 4 + 16*2; % the number of bytes received in each packet
reset = false;

% Wait for data  
while true
    in_bytes = pnet(u,'readpacket',bytes_read);
    if in_bytes > 0
        % if there is another frame waiting, read it straight away
        if in_bytes > bytes_read
            if in_bytes == u.InputBufferSize
                % buffer got full, reset
                % should only happen if we have been paused in Matlab for some time
                fprintf('Buffer reset\n')
                continue;
            end
            continue;
        end
        
        % read in data from AP
        magic = pnet(u,'read',1,'UINT16','intel');
        double(pnet(u,'read',1,'UINT16','intel')); % Cant set frame rate in Simulink
        SITL_frame = pnet(u,'read',1,'UINT32','intel');
        pwm = double(pnet(u,'read',16,'UINT16','intel'))';
        
        % check the magic value is what expect
        if magic ~= 18458
            warning('incorrect magic value')
            continue;
        end
        
        % Check if the fame is in expected order
        if SITL_frame < last_SITL_frame
            % Controller has reset
            connected = false;
            reset = true;
            fprintf('Controller reset\n')
        elseif SITL_frame == last_SITL_frame
            % duplicate frame, skip
            fprintf('Duplicate input frame\n')
            continue;
        elseif SITL_frame ~= last_SITL_frame + 1 && connected
            fprintf('Missed %i input frames\n',SITL_frame - last_SITL_frame - 1)
        end
        last_SITL_frame = SITL_frame;
        break;
    end
end

if ~connected
    % use port -1 to indicate connection to address of last recv pkt
    connected = true;
    [ip, port] = pnet(u,'gethost');
    fprintf('Connected to %i.%i.%i.%i:%i\n',ip,port)
end
frame_count = frame_count + 1;

%print a fps and runtime update
if rem(frame_count,print_frame_count) == 0
    total_time = toc(frame_time);
    frame_time = tic;
    sim_time = time - last_sim_time;
    last_sim_time = time;
    time_ratio = sim_time/total_time;
    fprintf("%0.2f fps, %0.2f%% of realtime\n",print_frame_count/total_time,time_ratio*100)
end

end


