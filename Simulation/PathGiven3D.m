function [NewOffPth,NewPth]=PathGiven3D(Pth,xof,yof,zof,aof,xnoise,ynoise,znoise,obx,oby,dimxmin,dimxmax,dimymin,dimymax,spd,draw)
% Inputs
% Pth = 3xn matrix describing the true path
%   where
%   Pth(1,:) is the x values
%   Pth(2,:) is the y values
%   Pth(3,:) is the z values
%   Pth(4,:) is the q1 values
%   Pth(5,:) is the q2 values
%   Pth(6,:) is the q3 values
%   Pth(7,:) is the q4 values
%   Pth(8,:) is time
%   Note here q is quaternion representation
%
% xof = offset in the x direction
% yof = offset in the y direction
% zof = offest in the z direction
%
% aof = heading offset quaternion representation
%
% xnoise = gaussian noise in the x direction
% ynoise = gaussian noise in the y direction
% znoise = gaussian noise in the angle in degrees
%
% obx = dimension of object in the x dimension
% oby = dimension of object in the y dimension
% obz = dimension of object in the z dimension
%
% dimxmin = dimension of workspace in the min x dimension
% dimxmax = dimension of workspace in the max x dimension
% dimymin = dimension of workspace in the min y dimension
% dimymax = dimension of workspace in the max y dimension
% dimzmin = dimension of workspace in the min z dimension
% dimzmax = dimension of workspace in the max z dimension
%
% Outputs
% NewPth = Resulting Path
%
% Mili Shah
% March 2015
clf

% Create offset path
[m,nPts]=size(Pth);
NewOffPth = zeros(8,nPts);
NewPth = zeros(8,nPts);
Roff = q2rot(aof);

for i = 1:nPts
    if i == 1
        R = q2rot(Pth(4:7,i));
        NewPth(1:3,i) = Pth(1:3,i);
        NewOffPth(1:3,i) = R*Roff*[xof;yof;zof] + NewPth(1:3,i);
        Rn = Roff*R;
        NewOffPth(4:7,i) = rot2q(Rn)';
        old = Pth(1:3,i);
    else
        R = q2rot(Pth(4:7,i));
        new = Pth(1:3,i);
        NewPth(1:3,i) = Roff*(new-old)+NewPth(1:3,i-1);
        NewOffPth(1:3,i) = R*Roff*[xof;yof;zof] + NewPth(1:3,i);
        Rn = Roff*R;
        NewOffPth(4:7,i) = rot2q(Rn)';
        NewPth(4:7,i) = rot2q(Rn)';
        old = Pth(1:3,i);
    end
end

% Add noise to the new Path
NewOffPth(1,:) = NewOffPth(1,:) + xnoise*randn(1,nPts);
NewOffPth(2,:) = NewOffPth(2,:) + ynoise*randn(1,nPts);
NewOffPth(3,:) = NewOffPth(3,:) + znoise*randn(1,nPts);

% Plotting
plot(Pth(1,:),Pth(2,:),'k')
a = [dimxmin dimxmax dimymin dimymax];
axis(a)

Rob = [obx*[-1 1 1 -1 -1];oby*[-1 -1 1 1 -1];zeros(1,5)];

if draw == 1
    for i = 1:nPts
        % Draw groundtruth path
        plot(Pth(1,:),Pth(2,:),'g')
        hold on
        axis(a)
        
        % Draw center and offcenter on path
        plot(NewOffPth(1,1:i),NewOffPth(2,1:i),'--')
        plot(NewPth(1,1:i),NewPth(2,1:i),'ro-')
        plot(Pth(1,i),Pth(2,i),'go')
        plot(NewOffPth(1,i),NewOffPth(2,i),'o')
        plot(NewPth(1,i),NewPth(2,i),'ro')
        
        % Draw object
        ang = 2*acos(NewPth(7,i));
        R = q2rot(NewPth(4:7,i))*Rob;
        NewRob = R(1:2,:) + repmat(NewPth(1:2,i),1,5);
        plot(NewRob(1,:),NewRob(2,:),'k-')
        
        hold off
        if i > 1
            pause(spd*(Pth(8,i)-Pth(8,i-1)))
        end
    end
else
    close all
    plot(NewPth(1,:),NewPth(2,:),'r','LineWidth',1)
    hold on
    plot(NewOffPth(1,:),NewOffPth(2,:),'--','LineWidth',2)
    hold on
    plot(Pth(1,:),Pth(2,:),'g--','LineWidth',2)
    axis(a)
    axis equal
    legend('Groundtruth Path','Offset Path','Commanded Path')
end
end

function R = q2rot(q)
% Converting a quaternion q to 
% Rotation R from
%
% http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
%
% APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
% Working Paper, Stanford AI Lab, 19791
% by Eugene Salamin
%
% Mili Shah
% Aug 25, 2011

R = zeros(3,3);

R(1,1) = q(4)^2+q(1)^2-q(2)^2-q(3)^2;
R(2,2) = q(4)^2-q(1)^2+q(2)^2-q(3)^2;
R(3,3) = q(4)^2-q(1)^2-q(2)^2+q(3)^2;

R(1,2) = 2*(-q(4)*q(3)+q(1)*q(2));
R(2,1) = 2*(q(4)*q(3)+q(1)*q(2));

R(1,3) = 2*(q(4)*q(2)+q(1)*q(3));
R(3,1) = 2*(-q(4)*q(2)+q(1)*q(3));

R(2,3) = 2*(-q(4)*q(1)+q(2)*q(3));
R(3,2) = 2*(q(4)*q(1)+q(2)*q(3));
end

function q = rot2q(R)

% Converting a rotation matrix R to 
% quaternion q from
%
% http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
%
% APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
% Working Paper, Stanford AI Lab, 19791
% by Eugene Salamin
%
% Mili Shah
% Aug 25, 2011

q = zeros(4,1);

q(4) = 1/2*sqrt(1+R(1,1)+R(2,2)+R(3,3));
q(1) = 1/2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
q(2) = 1/2*sqrt(1-R(1,1)+R(2,2)-R(3,3));
q(3) = 1/2*sqrt(1-R(1,1)-R(2,2)+R(3,3));

[k, in] = max(q);

if in == 1
   q(2) = 1/4/q(1)*(R(1,2)+R(2,1));
   q(3) = 1/4/q(1)*(R(1,3)+R(3,1));
   q(4) = 1/4/q(1)*(R(3,2)-R(2,3));
end
if in == 2
    q(1) = 1/4/q(2)*(R(1,2)+R(2,1));
    q(3) = 1/4/q(2)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(2)*(R(1,3)-R(3,1));
end
if in == 3
    q(1) = 1/4/q(3)*(R(1,3)+R(3,1));
    q(2) = 1/4/q(3)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(3)*(R(2,1)-R(1,2));
end
if in == 4
    q(1) = 1/4/q(4)*(R(3,2)-R(2,3));
    q(2) = 1/4/q(4)*(R(1,3)-R(3,1));
    q(3) = 1/4/q(4)*(R(2,1)-R(1,2));
end
end

