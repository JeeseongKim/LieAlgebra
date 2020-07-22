clear all;
close all;
clc;

%% Exponential so(3) -> SO(3) 

% r_x = 90*pi/180;
% r_y = -90*pi/180;
r_z = 90*pi/180;

r_x = 0*pi/180;
r_y = 0*pi/180;
% r_z = 0*pi/180;

w = [r_x;
     r_y;
     r_z;];

w_hat = [0        -w(3)   w(2);
         w(3)     0       -w(1);
         -w(2)    w(1)    0;];

w_norm = norm(w);

if (w_norm == 0)
    R = eye(3,3);
    
else
    w_w_norm = w_hat/w_norm;
    R = eye(3,3) + w_w_norm * sin(w_norm) + w_w_norm * w_w_norm *(1-cos(w_norm)); %this R = robot to world 

end

%% logarithm of SO(3) -> so(3)

norm_w = acos((trace(R)-1)/2);

log_R = (norm_w/(2*sin(norm_w)))*(R-R');

% log_w_w_norm = 1/(2*sin(norm_w)) * [R(3,2) - R(2,3);
%                                     R(1,3) - R(3,1);
%                                     R(2,1) - R(1,2);];
% if (norm_w == 0)
%     w_log = [0;0;0];
% else
%     w_log = norm_w * log_w_w_norm;
% end

if (norm_w==0)
    w_log = [0;0;0];
else
    w_3 = log_R(2,1); %w3
    w_1 = log_R(3,2); %w1
    w_2 = log_R(1,3); %w2
    w_log = [w_1; w_2; w_3]; 
end

%% se(3) -> SE(3)

tx = 1;
ty = 1;
tz = 1;

rx = 90*pi/180;
ry = -90*pi/180;
rz = 90*pi/180;

t = [tx;ty;tz];
w = [rx;ry;rz];
w_norm = norm(w);
w_skew = [0        -w(3)   w(2);
          w(3)     0       -w(1);
          -w(2)    w(1)    0;];
     
v_ = [t;w];

Av(1:3,1:3) = w_skew;
Av(1:3,4) = t;
Av(4,1:4) = [0 0 0 0];

V = eye(3,3) + (((1-cos(w_norm))*w_skew)/(w_norm*w_norm)) + (((w_norm - sin(w_norm))*w_skew*w_skew)/(w_norm*w_norm*w_norm));

if (w_norm == 0)
    R = eye(3,3);
else
    w_w_norm = w_hat/w_norm;
    R = eye(3,3) + w_w_norm * sin(w_norm) + w_w_norm * w_w_norm *(1-cos(w_norm)); %this R = robot to world 
end

expv(1:3,1:3) = R;
expv(1:3,4) = V*t;
expv(4,1:3)=[0 0 0];
expv(4,4) = 1;


%% SE(3) -> se(3)

inv_V =eye(3,3) - (0.5*w_skew) + (((1-((w_norm*cos(0.5*w_norm))/(2*sin(0.5*w_norm))))/(w_norm * w_norm))*(w_skew*w_skew));

norm_w = acos((trace(R)-1)/2);
log_R = (norm_w/(2*sin(norm_w)))*(R-R');

if (norm_w==0)
    w_ = [0;0;0];
else
    w_3 = log_R(2,1); %w3
    w_1 = log_R(3,2); %w1
    w_2 = log_R(1,3); %w2
    w_ = [w_1; w_2; w_3]; 
end

tprime = inv_V * t;














%% se(3) -> SE(3)
t = [1;0;0];

theta = norm(w);
A = sin(theta)/theta;
B = (1-cos(theta))/(theta*theta);
C = (1-A)/ (theta*theta);

R = eye(3,3) + A*w_hat + B*w_hat*w_hat;
V = eye(3,3) + B*w_hat + C*w_hat*w_hat;

%% SE(3) -> se(3)
% % inv_V_1 = eye(3,3) - 0.5*w_hat + (1/w_norm/w_norm)*(1-(A/2/B))*w_hat*w_hat;
% inv_V = eye(3,3) - 0.5*w_hat + ((1-((theta*cos(0.5*theta))/(2*sin(0.5*theta))))/w_norm/w_norm)*w_hat*w_hat;
% ans_u = inv_V * trans;
% 
% LieT(1:3, 1:3) = R;
% LieT(1:3, 4) = trans;
% LieT(4,1:4) = [0 0 0 1];
% 
% inv_LieT = inv(LieT);

%% confirm
% tmp_LieT(1:3, 1:3) = R';
% tmp_LieT(1:3, 4) = -R' * trans;
% tmp_LieT(4,1:4) = [0 0 0 1];

%%
translation_w = [1;0;0];
new_t = -R'*t;

tmp(1:3, 1:3) = R';
tmp(1:3, 4) = new_t;
tmp(4,1:4) = [0 0 0 1]; %world to robot

inv_V = eye(3,3) - 0.5*w_hat + ((1-((theta*cos(0.5*theta))/(2*sin(0.5*theta))))/w_norm/w_norm)*w_hat*w_hat;
ans_u = inv_V * new_t;

