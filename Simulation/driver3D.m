%Create Path
%Must be 8xn
%Row 1-3: 3D position
%Row 4-7: Quaternion rotation
%Row 8: Timestamp
%Some examples are below
nPts = 45;

%ACK Square
Pth = modSqr3D(nPts);

%Offsets
xof = 00; 
yof = 00;
zof = 00;

%Heading in Quaternion
%Following in degrees
rol = 0;
pit = 30;
yaw = 0;
Rx = [1 0 0; 0 cosd(rol) -sind(rol); 0 sind(rol) cosd(rol)];
Ry = [cosd(pit) 0 sind(pit); 0 1 0; -sind(pit) 0 cosd(pit)];
Rz = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
R = Rx*Ry*Rz
aof = rot2q(R)

%Gaussian Noise
xnoise = 0;
ynoise = 0;
znoise = 0;

%Size of the object
obx = 500;
oby = 500;

%Dimension of the workspace
dimxmin = 3000;
dimxmax = 9000;
dimymin = 11000;
dimymax = 16000;


%Speed Factor
%Less than 1 faster
%More than 1 slower
spd = .001;

%Draw
% 1 if you want to simulate the path
% 0 if you want just the final path
draw = 0;

%Create Path
figure(1)
Pth2=PathGiven3D(Pth,xof,yof,zof,aof,xnoise,ynoise,znoise,obx,oby,dimxmin,dimxmax,dimymin,dimymax,spd,draw);
a = [dimxmin dimxmax dimymin dimymax];
axis(a)

