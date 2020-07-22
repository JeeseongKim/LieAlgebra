clear all;
close all;
clc;

%% StereoSLAM : generate_T from world to camera
% %panning 먼저 하고 tilting 한거임
% 
% %th1 = tilting [degree]
% %th2 = panning [degree]
% %d1 = camera offset y
% %d2 = camera offset z
% %d3 = camera offset x
% rx = 0;
% ry = 0;
% rz = 0;
% 
% th1 = 90; th2 = 90; %degree
% d1 = 0; d2 = 0; d3 = 0; 
% 
% cx = rx + d3*cos(rz) - d1*sin(rz);
% cy = ry + d3*sin(rz) - d1*cos(rz);
% cz = rz + th2*pi/180;
% 
% tw2c = [cx;   %world 기준 카메라 좌표
%         cy; 
%         d2;];
% 
% fCtemp = cos(cz + pi/2);
% fStemp = sin(cz + pi/2);
%     
% fCTilting = cos(th1*pi/180);
% fSTilting = sin(th1*pi/180);
% 
% Rw2c = [fCtemp                 fStemp                  0;
%         fSTilting * fStemp     -fSTilting * fCtemp     fCTilting;
%         fCTilting * fStemp     -fCTilting * fCtemp     -fSTilting;];
%     
% % tw2c = -Rw2c * tc2w; %tw2c 카메라 기준 world 좌표
% tc2w = -Rw2c * tw2c; %tw2c 카메라 기준 world 좌표 
% 
% Tw2c = [Rw2c   tw2c;  
%         0 0 0  1;];
%     
% test_x = [1;2;3;1;];
% test_x_w2c = Tw2c*test_x;
   
%% from world to robot

rot_x = 90*pi/180;
% rot_y = 90*pi/180;
rot_y = 0*pi/180;
% rot_z = 90*pi/180;
rot_z = 0*pi/180;

tr_x = 10;
tr_y = 10;
tr_z = 10;

%곱하는 순서 phi->theta->alpha = pitch->yaw->roll (고정축)

alpha = rot_x;
theta = rot_z;
phi = rot_y;

Rw2r = [cos(theta)*cos(phi)                                     -sin(theta)                 cos(theta)*sin(phi);
        cos(alpha)*sin(theta)*cos(phi) + sin(alpha)*sin(phi)    cos(alpha)*cos(theta)       cos(alpha)*sin(theta)*sin(phi)-sin(alpha)*cos(phi);
        sin(alpha)*sin(theta)*cos(phi)-cos(alpha)*sin(phi)      sin(alpha)*cos(theta)       sin(alpha)*sin(theta)*sin(phi) + cos(alpha)*cos(phi)];

Ty_phi = [cos(phi)  0   sin(phi)    0;
          0         1   0           tr_y;
          -sin(phi) 0   cos(phi)    0;
          0         0   0           1;];
      
Tz_theta = [cos(theta)  -sin(theta) 0   0;
            sin(theta)  cos(theta)  0   0;
            0           0           1   tr_z;
            0           0           0   1;];

Tx_alpha = [1   0           0           tr_x;
            0   cos(alpha)  -sin(alpha) 0;
            0   sin(alpha)  cos(alpha)  0;
            0   0           0           1;];

Tw2r = Tx_alpha * Tz_theta * Ty_phi;

% temp = [1;2;3;1];
% temp_ = Ty_phi*temp;
% temp__ = Tz_theta * temp_;
% temp___ = Tx_alpha*temp__;
% temp_result = Tw2r * temp;
% 
% tmp = [1;2;3;];
% tmp = Rw2r*tmp

%% from robot to camera (StereoSLAM 부분 사용) _ Euler
rx = 0;
ry = 0;
rz = 0; % 30degree in [radian]

th1 = 90; th2 = 90; %degree
d1 = 0; d2 = 0; d3 = 0; %camera offset

cx = rx + d3*cos(rz) - d1*sin(rz);
cy = ry + d3*sin(rz) - d1*cos(rz);
cz = rz + th2*pi/180;

tc2r = [cx; 
        cy; 
        d2;];

fCtemp = cos(cz + pi/2);
fStemp = sin(cz + pi/2);
    
fCTilting = cos(th1*pi/180);
fSTilting = sin(th1*pi/180);

Rr2c = [fCtemp                 fStemp                  0;
        fSTilting * fStemp     -fSTilting * fCtemp     fCTilting;
        fCTilting * fStemp     -fCTilting * fCtemp     -fSTilting;];
    
tr2c = -Rr2c * tc2r; %tw2c 카메라 기준 world 좌표 

Tr2c = [Rr2c   tr2c;
        0 0 0  1   ;];
%% from world to camera
%g(c,w) = g(c,r)g(r,w)
Tw2c = Tr2c * Tw2r;

%% test
tmp = [1;2;3;1];
result = Tw2c * tmp;


