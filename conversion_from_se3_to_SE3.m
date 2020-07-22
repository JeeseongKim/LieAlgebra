clear all;
close all;
clc;
%% compute se(3) given T

T = [0  -1  1  1;
     1  0  0   0;
     0  0  0   0;
     0  0  0   1;];
 
R = T(1:3,1:3);
thetha = acos((trace(R) - 1)/2);

% Handling zero rotation
if(thetha == 0)
     w = [0;0;0];
     v = T(1:3,4);
     ksi = [v w];
else
     w = thetha * (1/(2*sin(thetha))*[R(3,2) - R(2,3);R(1,3) - R(3,1);R(2,1) - R(1,2)]);
     wx = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0;];
     t = T(1:3,4);
     % Computing the linear velocity 3 x 1 vector (expression from ethan eade doc)
     v = (eye(3) - (1/2 * (wx)) + (((1/(thetha * thetha)) * (1 - ((thetha * sin(thetha)) / (2*(1 - cos(thetha)))))) * (wx * wx)))*t;
     ksi = [(v)' w'];
end


% Splitting the twist vector into angular and translational velocity
% vectors
twistVec = [1; 0; 0; 0*pi/180; 0*pi/180; 90*pi/180];

v = twistVec(1:3)';
w = twistVec(4:6)';

 % Constructing the rotational exponential coordinates matrix
wSkewed = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0;];

 % Handing zero rotation
if(1 && ~any(w))
    R = eye(3);
    t = v;
else
    % Rodrigues formula
    R = eye(3) + (wSkewed/norm(w)) * sin(norm(w)) + (((wSkewed)*(wSkewed))/(norm(w)*norm(w))) * (1 - cos(norm(w)));
    % Computing the 3 x 1 translational vector (expression from ethan eade doc)
    t = (eye(3) + (((1 - cos(norm(w))) / (norm(w)^2)) * wSkewed) + (((norm(w) - sin(norm(w))) / (norm(w)^3)) * (wSkewed * wSkewed))) * v;
end

 % Constructing the 4 x 4 transformation matrix
T = zeros(4);
T(4,4) = 1;

T(1:3,1:3) = R;
T(1:3,4) = t';



% function [ksi] = computese3(T)
%      R = T(1:3,1:3);
%      thetha = acos((trace(R) - 1)/2);
%      
%      % Handling zero rotation
%      if(thetha == 0)
%          w = [0;0;0];
%          v = T(1:3,4);
%          ksi = [v w];
%      else
%          w = thetha * (1/(2*sin(thetha))*[R(3,2) - R(2,3);R(1,3) - R(3,1);R(2,1) - R(1,2)]);
%          wx = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0;];
%          t = T(1:3,4);
%          % Computing the linear velocity 3 x 1 vector (expression from ethan eade doc)
%          v = (eye(3) - (1/2 * (wx)) + (((1/(thetha * thetha)) * (1 - ((thetha * sin(thetha)) / (2*(1 - cos(thetha)))))) * (wx * wx)))*t;
%          ksi = [(v)' w'];
%      end     
% end

% function [T] = computeSE3(twistVec)
%     % Splitting the twist vector into angular and translational velocity
%     % vectors
%      v = twistVec(1:3)';
%      w = twistVec(4:6)';
%      
%      % Constructing the rotational exponential coordinates matrix
%      wSkewed = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0;];
%      
%      % Handing zero rotation
%      if(1 && ~any(w))
%          R = eye(3);
%          t = v;
%      else
%          % Rodrigues formula
%          R = eye(3) + (wSkewed/norm(w)) * sin(norm(w)) + (((wSkewed)*(wSkewed))/(norm(w)*norm(w))) * (1 - cos(norm(w)));
% 
%          % Computing the 3 x 1 translational vector (expression from ethan eade doc)
%          t = (eye(3) + (((1 - cos(norm(w))) / (norm(w)^2)) * wSkewed) + (((norm(w) - sin(norm(w))) / (norm(w)^3)) * (wSkewed * wSkewed))) * v;
%      end
%      
%      % Constructing the 4 x 4 transformation matrix
%      T = zeros(4);
%      T(4,4) = 1;
%      
%      T(1:3,1:3) = R;
%      T(1:3,4) = t';
% 
% end