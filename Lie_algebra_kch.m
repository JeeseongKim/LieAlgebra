clear all
clc

%% exponential operator
rot_x = 90*pi/180;
% rot_x = 0;
rot_y = -90*pi/180;
% rot_y = 0;
rot_z = 90*pi/180;
% rot_z = 0;

w = [rot_x; rot_y; rot_z];

w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
R = eye(3) + w_hat/norm(w,2)*sin(norm(w,2))+(w_hat*w_hat)/(norm(w,2))^2*(1-cos(norm(w,2)));

%% logarithm operator

w_hat2 = acos((trace(R)-1)/2);
w2 = w_hat2*([R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/(2*sin(w_hat2)));

%% euler angle
c1 = cos(rot_z);
s1 = sin(rot_z);
c2 = cos(rot_y);
s2 = sin(rot_y);
c3 = cos(rot_x);
s3 = sin(rot_x);

R2 = [c1*c2 (c1*s2*s3 - c3*s1) (s1*s3 + c1*c3*s2); c2*s1 (c1*c3 + s1*s2*s3) (c3*s1*s2 - c1*s3); -s2 c2*s3 c2*c3]';

w_hat3 = acos((trace(R2)-1)/2);
w3 = w_hat3*([R2(3,2)-R2(2,3);R2(1,3)-R2(3,1);R2(2,1)-R2(1,2)]/(2*sin(w_hat3)));