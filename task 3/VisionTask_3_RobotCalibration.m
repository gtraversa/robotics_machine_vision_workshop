clear all
close all
clc

squareSize = 22;
imRobotStation = imread('CheckerBoard.png');
% figure;imshow(imRobotStation);
% title( 'input Image');

load LabCameraParams.mat

[imagePoints, boardSize] = detectCheckerboardPoints(imRobotStation);

worldPoints = generateCheckerboardPoints(boardSize, squareSize);

[R,t] = extrinsics (imagePoints, worldPoints, cameraParams);

for i = 1:length(worldPoints)
    worldPointsMapped(i,:) = pointsToWorld(cameraParams,R,t,imagePoints(i,:));
end

% figure, plot (worldPoints(:,1),worldPoints(:,2),'x')
% hold on, plot (worldPointsMapped(:,1),worldPointsMapped(:,2),'x')

robotPoints = [-100,-56.1,-100.8,31.21,53.21,8.831,30.44; 200,199.2,156,153.7,153.3,132.1,109.7; -20,-19.92,-20.08,-19.85,-19.81,-19.93,-19.93];
Xrbt = robotPoints(1,:);
Yrbt = robotPoints(2,:);
Zrbt = robotPoints(3,:);

worldPointsNew = [[-0.0193689968800578],[43.9740624301806],[-0.00822645318829719],[131.912648401460],[153.905119850616],[109.869577214923],[131.841102714848];[-0.0945267215786416],[-0.125460302691064],[43.9362241203264],[43.7651709861235],[43.6465952634203],[65.7643278684802],[87.6940242678573]];
Xwld = worldPointsNew(1,:);
Ywld = worldPointsNew(2,:);

Matrix = ([sum(Xwld.^2), sum(Xwld.*Ywld), sum(Xwld); sum(Xwld.*Ywld), sum(Ywld.^2), sum(Ywld); sum(Xwld), sum(Ywld), 7])^(-1);

rho1 = Matrix * [sum(Xrbt.*Xwld); sum(Xrbt.* Ywld); sum(Xrbt)];
rho2 = Matrix * [sum(Yrbt.*Xwld); sum(Yrbt.* Ywld); sum(Yrbt)];
rho3 = Matrix * [sum(Zrbt.*Xwld); sum(Zrbt.* Ywld); sum(Zrbt)];

rho11 = rho1(1);
rho12 = rho1(2);
tx = rho1(3);

rho21 = rho2(1);
rho22 = rho2(2);
ty = rho2(3);

rho31 = rho3(1);
rho32 = rho3(2);
tz = rho3(3);

rho4 = cross([rho11;rho21;rho31], [rho12; rho22; rho32]);
rho13 = rho4(1);
rho23 = rho4(2);
rho33 = rho4(3);

 T_RobotWorld = [rho11, rho12, rho13, tx; rho21, rho22, rho23, ty; rho31, rho32, rho33, tz; 0, 0, 0, 1]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imageCylinder = [313.8439, 218.8197];
worldCylinder = pointsToWorld(cameraParams,R,t,imageCylinder);
worldCylinder = [worldCylinder 0]; % add Z-axis value
robotCylinder = T_RobotWorld * [worldCylinder 1]'

imageCube = [64.0168, 304.6212];
worldCube =  pointsToWorld(cameraParams,R,t,imageCube);
worldCube = [worldCube 0]; % add Z-axis value
robotCube = T_RobotWorld * [worldCube 1]'