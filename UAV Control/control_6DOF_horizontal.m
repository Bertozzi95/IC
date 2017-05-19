% Control of attitude and altitude for coordinate turns. 
%
% More information can be found on Principles of Guidance, Navigation and
% Control of UAVs of Elkain et al. and in the Chapter 7, Section The Steady
% Turn of Dynamics of Flight of Etkins.

function [Controllers, input] = control_6DOF_horizontal(Controllers, desired, actual_state)
    
%     input = zeros(4,1);
%     
%     angles = quaternion_to_Euler(states.x(7:10));
%     
%     vel = sqrt(states.x(4)^2+states.x(5)^2+states.x(6)^2); 
%     
%     phi = angles(1);
%     theta = angles(2);
%     
%     % Velocities
%     u0 = states.x_6DOF(1);
%     v0 = states.x_6DOF(2);
%     w0 = states.x_6DOF(3);
%     vTrue = sqrt((u0^2)+(v0^2)+(w0^2));
% 
%     if (vTrue == 0)
%         beta = 0;
%     else
%         beta = asin(v0/vTrue); %angle of sideslip
%     end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Pitch PID with system output pitch and input elevator
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Pitch,input(3)] = PID_attitude_step(desired(3), actual_state(3), Controllers.PID_Pitch);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Roll PID with system output roll and input aileron
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Roll,input(2)] = PID_attitude_step(desired(2), actual_state(2), Controllers.PID_Roll);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Yaw PID with system output yaw and input rudder
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Controllers.PID_Yaw,input(4)] = PID_attitude_step(0, actual_state(3), Controllers.PID_Yaw);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PID for airspeed hold mode - y = velocity in m/s and u = throttle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [Controllers.PID_Vel,input(1)] = PID_step(desired(1), actual_state(1), Controllers.PID_Vel);
    
end
