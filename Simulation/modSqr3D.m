function Pth = modSqr3D(nPts)
close all
Pts = [...
    4251,4251,5050,5750,6450,7251,7251,7251,6450,5750,5050,4251,4251;
    13500,14200,15000,15000,15000,14200,13500,12800,12000,12000,12000,12800,13500;
    ];
Pth = [];
quarRot = zeros(4,nPts);
nVec = [0 0 1];
quad = -180;

for i = 1:12
    if Pts(1,i) == Pts(1,i+1) | Pts(2,i) == Pts(2,i+1)
        x = linspace(Pts(1,i),Pts(1,i+1),nPts);
        y = linspace(Pts(2,i),Pts(2,i+1),nPts);
        ang = quad*ones(1,nPts);
    else
        rx = Pts(1,i+1) - Pts(1,i);
        ry = Pts(2,i+1) - Pts(2,i);
        if rx>0
            if ry>0
                ang = linspace(180,90,nPts);
                x = rx*cosd(ang)+Pts(1,i+1);
                y = ry*sind(ang)+Pts(2,i);
                quad = 90;
            else
                ang = linspace(90,0,nPts);
                x = rx*cosd(ang)+Pts(1,i);
                y = -ry*sind(ang)+Pts(2,i+1);
                quad = 0;
            end
        else
            if ry<0
                ang = linspace(360,270,nPts);
                x = -rx*cosd(ang)+Pts(1,i+1);
                y = -ry*sind(ang)+Pts(2,i);
                quad = 270;
            else
                ang = linspace(270,180,nPts);
                x = -rx*cosd(ang)+Pts(1,i);
                y = ry*sind(ang)+Pts(2,i+1);
                quad = 180;
            end
        end
    end
    for j = 1:nPts
       quarRot(:,j) = [sind(ang(j)/2)*nVec cosd(ang(j)/2)];
    end
        Pth = [Pth [x;y;zeros(1,nPts);quarRot]];
    end
    [m,n]=size(Pth);
    Pth = [Pth;1:n];
end




