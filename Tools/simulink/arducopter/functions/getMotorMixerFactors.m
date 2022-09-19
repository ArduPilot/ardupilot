% Function to get the copters number of motors and there configuration to
% calculate the motor mixer factors for roll, pitch and yaw
function [num_motors, axis_facs_motors] = getMotorMixerFactors(frame_class, frame_type)
    d2r = pi/180;
    axis_facs_motors = struct;
    num_motors = 0;
    % Preliminary cell array for the motors axis factors consisting of the
    % roll/pitch factor in degree (first element of cells) and the yaw
    % factor
    axis_facs_motors_pre = cell(0,0);
    
    %% Get configuration
    
    switch frame_class
        case 1  % Quad
            num_motors = 4;
            axis_facs_motors_pre = cell(4,1);
            switch frame_type
                case 0  % Plus
                    axis_facs_motors_pre = { [90, 1]; [-90, 1]; [0, -1]; [180, -1] };
                case 1  % X
                    axis_facs_motors_pre = { [45, 1]; [-135, 1]; [-45, -1]; [135, -1] };
                case 2  % V
                    axis_facs_motors_pre = { [45, 0.7981]; [-135, 1.0]; [-45, -0.7981]; [135, -1.0] };
                case 3  % H
                    axis_facs_motors_pre = { [45, -1]; [-135, -1]; [-45, 1]; [135, 1] };
                case 13 % DJIX
                    axis_facs_motors_pre = { [45, 1]; [-45, -1]; [-135, 1]; [135, -1] };
                case 14 % Clockwise ordering
                    axis_facs_motors_pre = { [45, 1]; [135, -1]; [-135, 1]; [-45, -1] };
            end
        case 2  % Hexa
            num_motors = 6; 
        case {3, 4}  % Octa and OctaQuad
            num_motors = 8;
            axis_facs_motors_pre = cell(8,1);
            switch frame_type
                case 0  % Plus
                    axis_facs_motors_pre = { [0, -1]; [180, -1]; [45, 1]; [135, 1]; ...
                        [-45, 1]; [-135, 1]; [-90, -1]; [90, -1] };
                case 1  % X
                    axis_facs_motors_pre = { [22.5, -1]; [-157.5, -1]; [67.5, 1]; [157.5, 1]; ...
                        [-22.5, 1]; [-122.5, 1]; [-67.5, -1]; [112.5, -1] };
                case 2  % V
                    axis_facs_motors_pre = { [0.83,  0.34, -1]; [-0.67, -0.32, -1]; [0.67, -0.32, 1]; [-0.50, -1.00, 1]; ...
                        [1.00,  1.00, 1]; [-0.83,  0.34, 1]; [-1.00,  1.00, -1]; [0.5,  -1.00, -1] };
            end
        case 5  % Y6  
            num_motors = 6;
        case 12 % DodecaHexa
            num_motors = 12;
        case 14 % Deca
            num_motors = 14;
    end
    
    %% Check if configuration is defined
    % Leave function if frame class is not configured yet
    if num_motors == 0
       disp("Frame class unknown. Please add the configuration to the function.");
       return;
    end
    % Leave function if axis factors are not yet defined for frame
    if isempty(axis_facs_motors_pre)
        disp("Motor axis factors are not yet defined for frame class!");
        disp("The factors can be found in the setup_motors function within AP_MotorsMatrix.cpp.");
       return;        
    end
    
    %% Calculate axis factors (roll/pitch) from preliminary array
    % Create the output struct that stores the factors for the different
    % axis in separate arrays. Each column of the subarrays represents one motor.
    axis_facs_motors = struct('roll', zeros(1,num_motors), ...
        'pitch', zeros(1,num_motors), ...
        'yaw', zeros(1,num_motors) , ...
        'throttle', zeros(1, num_motors) );
    for i=1:num_motors
        % Check if factors for roll and pitch are defined separately and
        % therefore in radian -> dimension of a cell would be 3
        if ( length(axis_facs_motors_pre{i}) >= 3 )
            axis_facs_motors.roll(1,i) = axis_facs_motors_pre{i}(1);
            axis_facs_motors.pitch(1,i) = axis_facs_motors_pre{i}(2);
        else
            axis_facs_motors.roll(1,i) = cos( (axis_facs_motors_pre{i}(1)+90)*d2r );
            axis_facs_motors.pitch(1,i) = cos( (axis_facs_motors_pre{i}(1))*d2r );
        end
        % The factors for yaw can be acquired directly from the preliminary
        % array
        axis_facs_motors.yaw(1,i) = axis_facs_motors_pre{i}(2);
        % The factors for throttle are 1.0 by default (see AP_MotorsMatrix.h,
        % line 87)
        axis_facs_motors.throttle(1,i) = 1.0;
    end
    
    %% Normalization of factors (AP_MotorsMatrix.cpp, line 1218)
    roll_fac_max = 0.0;
    pitch_fac_max = 0.0;
    yaw_fac_max = 0.0;
    throttle_fac_max = 0.0;
    
    % Find maximum factor
    for i=1:num_motors
       roll_fac_max = max(roll_fac_max, axis_facs_motors.roll(1,i));
       pitch_fac_max = max(pitch_fac_max, axis_facs_motors.pitch(1,i));
       yaw_fac_max = max(yaw_fac_max, axis_facs_motors.yaw(1,i));
       throttle_fac_max = max(throttle_fac_max, axis_facs_motors.throttle(1,i));
    end
    
    % Scale factors back to -0.5 to +0.5 for each axis
    for i=1:num_motors
       if(roll_fac_max ~= 0)
           axis_facs_motors.roll(1,i) = 0.5 * axis_facs_motors.roll(1,i) / roll_fac_max;
       end
       if(pitch_fac_max ~= 0)
           axis_facs_motors.pitch(1,i) = 0.5 * axis_facs_motors.pitch(1,i) / pitch_fac_max;
       end
       if(yaw_fac_max ~= 0)
           axis_facs_motors.yaw(1,i) = 0.5 * axis_facs_motors.yaw(1,i) / yaw_fac_max;
       end
       if(throttle_fac_max ~= 0)
           axis_facs_motors.throttle(1,i) = 0.5 * axis_facs_motors.throttle(1,i) / throttle_fac_max;
       end
    end
end