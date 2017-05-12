%
% MSE 480/780: Robotics Lab 2 - Inverse Kinematics
%
% By: Vivek Vyas & Zahra Haeri
clc; clear all; close all;
%% Variables
d1 = 6.193; %cm
L2 = 14.605; %cm
L3 = 18.733; %cm
d5 = 9.843; %cm
%% Case 1 - Initialization
Tot = [ 0 0 1 28.58 ;...
 0 -1 0 0 ;...
 1 0 0 20.8 ;...
 0 0 0 1 ];

%% Case 2 - Initialization
Tot = [ 1 0 0 15 ;...
 0 -1 0 5 ;...
 0 0 -1 10 ;...
 0 0 0 1 ];
%% Case 3 - Initialization
Tot = [ 1 0 0 15 ;...
 0 -1 0 5 ;...
 0 0 -1 1 ;...
 0 0 0 1 ];
%% Case 4 - Initialization
Tot = [ 1 0 1 15 ;...
 0 -1 0 -5 ;...
 0 0 -1 5 ;...
 0 0 0 1 ];

%% Calculations - Theta 1, 2, 3, 4, 5, 6
d = [Tot(1,4), Tot(2,4), Tot(3,4)] ; %FIXED, YAY!
a = [Tot(1,3), Tot(2,3), Tot(3,3)] ; %FIXED, YAY!
dc = d - d5*a; % dc = (dcx, dcy, dcz)
dcx = dc(1);
dcy = dc(2);
dcz = dc(3);
% Theta 1
theta1_caseA = atan2(dcy, dcx);
theta1_cos = cos(theta1_caseA);
theta1_sin = sin(theta1_caseA);
theta1 = theta1_caseA;
% Theta 3
theta3_sin = (dcx^2 + dcy^2 + (dcz - d1)^2 - L2^2 - L3^2)/(2*L2*L3);
theta3_cos = 1*sqrt(1 - theta3_sin^2);
theta3 = atan2(theta3_sin, theta3_cos);
% Theta 2
% theta2_sin = ( ( -L3*theta3_cos*( dcz - d1 ) ) + ...
% ( (L2 + L3*theta3_sin)*-1*sqrt( dcx^2 + dcy^2 ) ) );
% theta2_cos = ( ( -L2 - L3*theta3_sin )*( dcz - d1 ) + ...
% ( -L3*theta3_cos*-1*sqrt( dcx^2 + dcy^2 ) ) );
% theta2 = atan2(theta2_sin, theta2_cos);
A = [L2+L3*theta3_sin L3*theta3_cos;...
 L3*theta3_cos -(L2+L3*theta3_sin)];
B = [dcz-d1; sqrt(dcx^2+dcy^2)];
theta2Vec = A\B;
theta2_cos = theta2Vec(1);
theta2_sin = theta2Vec(2);
theta2 = atan2(theta2_sin, theta2_cos);
alpha1_cos = cos(pi/2);
alpha1_sin = sin(pi/2);
A_01 =[cos(theta1), -1*sin(theta1)*alpha1_cos, sin(theta1)*alpha1_sin;
 sin(theta1), cos(theta1)*alpha1_cos, -1*alpha1_sin*cos(theta1);
 0 , alpha1_sin , alpha1_cos ];
alpha2_cos = cos(0);
alpha2_sin = sin(0);
A_12 = [cos(theta2+pi/2), -1*sin(theta2+pi/2)*alpha2_cos,
sin(theta2+pi/2)*alpha2_sin;
 sin(theta2+pi/2), cos(theta2+pi/2)*alpha2_cos, -
1*alpha2_sin*cos(theta2+pi/2);
 0 , alpha2_sin , alpha2_cos ];
alpha3_cos = cos(0);
alpha3_sin = sin(0);
A_23 =[cos(theta3-pi/2), -1*sin(theta3-pi/2)*cos(alpha3_cos), sin(theta3-
pi/2)*alpha3_sin;
 sin(theta3-pi/2), cos(theta3-pi/2)*alpha3_cos, -
1*alpha3_sin*cos(theta3-pi/2);
 0 , alpha3_sin , alpha3_cos ];

% using the found matricies to perform decoupling
% given T0t, so to find R_ot, we take the 3x3 minor:
R_03 = A_01*A_12*A_23;
R_0t = [Tot(1,1), Tot(1,2), Tot(1,3);
 Tot(2,1), Tot(2,2), Tot(2,3);
 Tot(3,1), Tot(3,2), Tot(3,3)] ;
% solving for R_3t
R_3t = R_03'*R_0t;
% extracting information from R_3t, and solving for desired angles
theta4_cos = R_3t(1,3);
theta4_sin = R_3t(2,3);
theta5_cos = R_3t(3,2);
theta5_sin = R_3t(3,1);
theta4 = atan2(theta4_sin,theta4_cos);
theta5 = atan2(theta5_sin,theta5_cos);
theta1 = rad2deg(theta1);
theta2 = rad2deg(theta2);
theta3 = rad2deg(theta3);
theta4 = rad2deg(theta4);
theta5 = rad2deg(theta5);
[theta1,theta2,theta3,theta4,theta5]
%% Theta to Encoder Values
EVo = [1500 1540 1500 1500 1500];
thetaMin = [-90 -45 -45 -45 -90];
thetaMax = [90 45 45 45 90];
EVmin = [2500 1115 1900 975 750];
EVmax = [550 1950 1075 1950 2500];
theta = [theta1 theta2 theta3 theta4 theta5];
for i=1:5
 EV(i) = EVo(i) + theta(i)*((EVmax(i) - EVmin(i))/(thetaMax(i) -
thetaMin(i)));
end