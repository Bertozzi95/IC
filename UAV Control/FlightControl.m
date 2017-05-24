%X-PLane connecting to MATLAB test script
%% Import XPC
addpath('../')
import XPlaneConnect.*
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
%% Get the position of the aircraft into posi vector
%posi = [Lat,Lon,Alt,Pitch,Roll,Yaw,Gear]
posi = getPOSI(0,Socket);
%% Get the control informations
%ctrl[LatStick,LonStick,Rudder,Throttle,Gear,Flaps]
%Elevator: ctrl(1)
%Aileron: ctrl(2)
%Rudder: ctrl(3)
ctrl = getCTRL(0,Socket);
%% Getting the Beta Angle (Sideslip)
    beta = getDREFs('sim/flightmodel/position/beta', Socket);
    beta = degtorad(beta);
%% Getting Airspeed
    airspeed = getDREFs('sim/flightmodel/position/true_airspeed', Socket);
    %disp (airspeed);
%% Testing the PID
dt = 0.1; %tempo de amostragem do xplane
angle_ini = [degtorad(posi(5)),degtorad(posi(4))]; %Roll, Pitch em rad
x_vel_ini = airspeed; % local_vx,local_vy,local_vz ??
max_control = [1,degtorad(15),degtorad(15),degtorad(20)]; %throttle,aileron,elevator,rudder deflection em rad
desired = [0,degtorad(0),degtorad(0)]; %Throttle, Roll, Pitch BETA SEMPRE 0
alt = posi(3);
%% Control through the PIDs 
%Create ACTUAL_STATE [VEL, ROLL, PITCH, BETA] 
set_controllers_horizontal(dt, angle_ini, x_vel_ini, alt, max_control, desired);
for i=0:1000
    %Ler valores atuais
    
    actual_state = [
    %rodar PID
    %Escrever pro XP
end
%% Exit
 closeUDP(Socket);
 disp('End');