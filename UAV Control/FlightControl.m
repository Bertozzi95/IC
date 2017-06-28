%X-PLane connecting to MATLAB test script
%% Import XPC
addpath('../')
import XPlaneConnect.*
Example;
%% Setup
% Create variables and open connection to X-Plane

disp('Start');
Socket = openUDP('127.0.0.1', 49009);
%% Ensure connected
getDREFs('sim/test/test_float', Socket);
%% Amostrando o tempo de amostragem
time1 = getDREFs('sim/time/total_flight_time_sec',Socket);
time2 = getDREFs('sim/time/total_flight_time_sec',Socket);
dif = time2-time1;
%% Get the control informations
%ctrl[LatStick,LonStick,Rudder,Throttle,Gear,Flaps]
%Elevator: ctrl(1)
%Aileron: ctrl(2)
%Rudder: ctrl(3)
ctrl = getCTRL(0,Socket);
%% Getting the Beta Angle (Sideslip)
    beta = getDREFs('sim/flightmodel/position/beta', Socket);
    beta = degtorad(beta);
%% Starter values for the PID
%Get the position of the aircraft into posi vector
%posi = [Lat,Lon,Alt,Pitch,Roll,Yaw,Gear]
posi = getPOSI(0,Socket);

dt = 0.1; %tempo de amostragem do xplane
angle_ini = [degtorad(posi(5)),degtorad(posi(4))]; %Roll, Pitch em rad
x_vel_ini = getDREFs('sim/flightmodel/position/true_airspeed', Socket);
max_control = [1,degtorad(15),degtorad(15),degtorad(20)]; %throttle,aileron,elevator,rudder deflection em rad
desired = [0,0,degtorad(3),degtorad(0)]; %Throttle, Roll, Pitch BETA SEMPRE 0
alt = posi(3);
%% Control through the PIDs 
    %Setting the animatedlines
    figure;
    subplot(2,3,1)
    roll_graph = animatedline('LineWidth',0.5,'Color',[1 0 0]);
    title('Roll');
    roll_ref = animatedline('LineWidth',0.5,'Color',[0 0 0]);
    
    subplot(2,3,2);
    beta_graph = animatedline('LineWidth',0.5,'Color',[0 0 1]);
    title('Beta');
    beta_ref = animatedline('LineWidth',0.5,'Color',[0 0 0]);

    subplot(2,3,3);
    pitch_graph = animatedline('LineWidth',0.5,'Color',[0 1 0]);
    title('Pitch');
    pitch_ref = animatedline('LineWidth',0.5,'Color',[0 0 0]);
    
    subplot(2,3,5);
    rudder_graph = animatedline('LineWidth',0.5,'Color',[0 0 1]);
    title('Rudder');
    rudder_ref = animatedline('LineWidth',0.5,'Color',[0 0 0]);
    
    subplot(2,3,4);
    aileron_graph = animatedline('LineWidth',0.5,'Color',[0 0 1]);
    title('Aileron');
    aileron_ref = animatedline('LineWidth',0.5,'Color',[0 0 0]);
    
    subplot(2,3,6);
    elevator_graph = animatedline('LineWidth',0.5,'Color',[0 0 1]);
    title('Elevator');
    elevator_ref = animatedline('LineWidth',0.5,'Color',[0 0 1]);
%DIsable the auto_pilot
sendDREFs('sim/cockpit/autopilot/autopilot_mode',0, Socket);

%Set the initial state to the controllers
result_Controllers = set_controllers_horizontal(dt, angle_ini, x_vel_ini, alt, max_control, desired);

i = 0;
%Execute the PID control forever
while 1
   
    %Read values
    beta = getDREFs('sim/flightmodel/position/beta', Socket);
    beta = degtorad(beta);
    airspeed = getDREFs('sim/flightmodel/position/true_airspeed', Socket);
    posi = getPOSI(0,Socket);
    
    %Setting roll pitch beta in the graph
    addpoints(roll_graph,i,double(posi(5)));
    addpoints(pitch_graph,i,double(posi(4)));
    addpoints(beta_graph,i,double(radtodeg(beta)));
    
    %Setting references for beta pitch roll
    addpoints(beta_ref,i,0)
    addpoints(pitch_ref,i,radtodeg(desired(3)))
    addpoints(roll_ref,i,radtodeg(desired(2)))
    
    %Setting references for rudder elevator aileron
    %addpoints(rudder_ref,i,0)
    %addpoints(elevator_ref,i,0)
    %addpoints(aileron_ref,i,0)
    
    drawnow limitrate
    
    %Create ACTUAL_STATE [VEL, ROLL, PITCH, BETA]
    actual_state = [airspeed,degtorad(posi(5)),degtorad(posi(4)),beta];
    
    %run PID using control_6DOF_horizontal
    %result=[Throttle,Aileron,Elevator,Rudder]
    [result_Controllers,input] = control_6DOF_horizontal(result_Controllers,desired,actual_state);
    
    
    %Converting from the propotion to absolute degrees
    ctrl_input(1) = 0;
    ctrl_input(2)=(radtodeg(input(2))/15);
    ctrl_input(3)=(radtodeg(input(3))/15);
    ctrl_input(4)=(radtodeg(input(4))/20);
    
    
    %Write into XPlane
    %ctrl[LatStick,LonStick,Rudder,Throttle,Gear,Flaps]
    ctrl_values = [ctrl_input(3),ctrl_input(2),ctrl_input(4),-998,-998,-998]; %-998 to not overwrite the current value
    sendCTRL(ctrl_values,0,Socket);
    
    %Setting rudder elevator rudder in the graph
    addpoints(rudder_graph,i,double(radtodeg(input(4))));
    addpoints(elevator_graph,i,double(radtodeg(input(2))));
    addpoints(aileron_graph,i,double(radtodeg(input(3))));
   
    
    
    
    %Somente testes
    %disp(i);
    i=i+1;
    pause(0.01);
end
%% Exit
 closeUDP(Socket);
 disp('End');