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
%% Amostrando o tmepo de amostragem
time1 = getDREFs('sim/time/total_flight_time_sec',Socket);
time2 = getDREFs('sim/time/total_flight_time_sec',Socket);
dif = time2-time1;
disp(['tempo1: ' num2str(time1)]);
disp(['tempo2: ' num2str(time2)]);
disp(['dif: ' num2str(dif)]);
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
    %disp(beta);
%% Getting Airspeed
    airspeed = getDREFs('sim/flightmodel/position/true_airspeed', Socket);
    %disp (airspeed);
%% Show the values into the screen (for test and check)
%fprintf('Location: (%4f, %4f, %4f) Aileron:%2f Elevator:%2f Rudder:%2f\n',posi(1),posi(2),posi(3),ctrl(2),ctrl(1),ctrl(3));
%fprintf('Roll: %4f, Pitch: %4f\n',posi(5),posi(4));
%% Testing the PID
dt = 0.1; %tempo de amostragem do xplane
angle_ini = [degtorad(posi(5)),degtorad(posi(4))]; %Roll, Pitch em rad
x_vel_ini = airspeed; % local_vx,local_vy,local_vz ??
max_control = [1,degtorad(15),degtorad(15),degtorad(20)]; %throttle,aileron,elevator,rudder deflection em rad
desired = [0,degtorad(0),degtorad(0)]; %Throttle, Roll, Pitch BETA SEMPRE 0
alt = posi(3);
%CRIAR ACTUAL_STATE [VEL, ROLL, PITCH, BETA] 
%
set_controllers_horizontal(dt, angle_ini, x_vel_ini, alt, max_control, desired);
for i=0:1000
    %Ler valores atuais
    %rodar PID
    %Escrever pro XP
end
%% Exit
 closeUDP(Socket);
 disp('End');