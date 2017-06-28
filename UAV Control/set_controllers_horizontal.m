% Set controllers gains for the horizontal flight mode.

function Controllers = set_controllers_horizontal(dt, angle_ini, x_vel_ini, alt, max_control, desired_angle_ini)
    
    Controllers.function = @control_6DOF_horizontal;

    % Pitch PID with system output pitch and input dpitch
    Controllers.PID_Pitch.Dt = dt;
    Controllers.PID_Pitch.Kp = 1;
    Controllers.PID_Pitch.Ki = 0.3;
    Controllers.PID_Pitch.Kd = 0.0001;
    Controllers.PID_Pitch.Tf = 0;
    Controllers.PID_Pitch.P = 0;
    Controllers.PID_Pitch.D = 0;
    Controllers.PID_Pitch.I = 0;
    Controllers.PID_Pitch.Old_Error = angdiff(angle_ini(2),desired_angle_ini(3));
    Controllers.PID_Pitch.Old_Output = angle_ini(2);
    Controllers.PID_Pitch.Old_Input = 0;
    Controllers.PID_Pitch.Min_Input = -max_control(3);
    Controllers.PID_Pitch.Max_Input = max_control(3);

    % Roll PID with system output roll and input droll
    Controllers.PID_Roll.Dt = dt;
    Controllers.PID_Roll.Kp = 0.5;
    Controllers.PID_Roll.Ki = 0.03;
    Controllers.PID_Roll.Kd = 0.0001;
    Controllers.PID_Roll.Tf = 0;
    Controllers.PID_Roll.P = 0;
    Controllers.PID_Roll.D = 0;
    Controllers.PID_Roll.I = 0;
    Controllers.PID_Roll.Old_Error = angdiff(angle_ini(1),desired_angle_ini(2));
    Controllers.PID_Roll.Old_Output = angle_ini(1);
    Controllers.PID_Roll.Old_Input = 0;
    Controllers.PID_Roll.Min_Input = -max_control(2);
    Controllers.PID_Roll.Max_Input = max_control(2);

    % Yaw PID with system output beta and input dbeta
    Controllers.PID_Yaw.Dt = dt;
    Controllers.PID_Yaw.Kp = -0.005;
    Controllers.PID_Yaw.Ki = -0.0;
    Controllers.PID_Yaw.Kd = 0.0;
    Controllers.PID_Yaw.Tf = 0;
    Controllers.PID_Yaw.P = 0;
    Controllers.PID_Yaw.D = 0;
    Controllers.PID_Yaw.I = 0;
    Controllers.PID_Yaw.Old_Error = 0;
    Controllers.PID_Yaw.Old_Output = 0;
    Controllers.PID_Yaw.Old_Input = 0;
    Controllers.PID_Yaw.Min_Input = -max_control(4);
    Controllers.PID_Yaw.Max_Input = max_control(4);

    % Modified values for the vTrue PID (controlled by throttle)
    % with Kp = 0.00625, Ki = 0.000408
    Controllers.PID_Vel.Dt = dt;
    Controllers.PID_Vel.Kp = 10;
    Controllers.PID_Vel.Ki = 0;
    Controllers.PID_Vel.Kd = 0;
    Controllers.PID_Vel.Tf = 0;
    Controllers.PID_Vel.D = 0;
    Controllers.PID_Vel.I = 0;
    Controllers.PID_Vel.Old_Error = 0;
    Controllers.PID_Vel.Old_Output = x_vel_ini; 
    Controllers.PID_Vel.Old_Input = 0;
    Controllers.PID_Vel.Min_Input = 0;
    Controllers.PID_Vel.Max_Input = max_control(1);

    % Feedforward term to compensate altitude loss during coordinate turn
    Controllers.Feedforward_alt.gain = 0;

end
 
