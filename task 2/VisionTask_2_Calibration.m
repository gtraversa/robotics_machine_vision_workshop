clear all
close all
clc

numImages = 6;
squareSize = 22;

files = cell(1, numImages);
for i = 1:numImages
    
    files{i} = fullfile (sprintf('Image%d.png',i));

end

I = imread(files{1});
figure;imshow(I);
title('One of the calibration Images');

[imagePoints, boardSize] = detectCheckerboardPoints(files);

worldPoints = generateCheckerboardPoints(boardSize,squareSize);


imageSize = [size(I,1), size(I,2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize); 
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');

figure;
showExtrinsics(cameraParams, 'CameraCentric');

figure;
showExtrinsics(cameraParams, 'PatternCentric');

% Calibrate the camera again with code in task 1
% Dimensions of matrices changed to match input data

numberImages = numImages;

worldPoints = worldPoints';
X = worldPoints(1,:);
Y = worldPoints(2,:);

imagePoints = permute(imagePoints,[2,1,3]);

phi = zeros(108,9,6);
for N = 1:numberImages
    for i = 1:2:107
    j = (i+1)/2;
    phi(i,:,N) = [0, 0, 0, X(j), Y(j), 1, -imagePoints(2,j,N)* X(j),-imagePoints(2,j,N)* Y(j),-imagePoints(2,j,N)];
    phi(i+1,:,N)=[ X(j), Y(j),1, 0, 0, 0, -imagePoints(1,j,N)* X(j),-imagePoints(1,j,N)* Y(j),-imagePoints(1,j,N)];
    end
end
    
H = zeros(9,6);
for N = 1:numberImages
    [U,S,V] = svd(phi(:,:,N));
    H(:,N) = V(:,end);
end

v = zeros(10,6);
for N = 1:numberImages
    v(2*N-1,:) = [H(1,N)*H(2,N), H(4,N)*H(2,N)+H(1,N)*H(5,N), H(4,N)*H(5,N), H(7,N)*H(2,N)+H(1,N)*H(8,N),  H(7,N)*H(5,N)+H(4,N)*H(8,N), H(7,N)*H(8,N)];
    v(2*N,:) = [[H(1,N)*H(1,N), H(4,N)*H(1,N)+H(1,N)*H(4,N), H(4,N)*H(4,N), H(7,N)*H(1,N)+H(1,N)*H(7,N),  H(7,N)*H(4,N)+H(4,N)*H(7,N), H(7,N)*H(7,N)]-[H(2,N)*H(2,N), H(5,N)*H(2,N)+H(2,N)*H(5,N), H(5,N)*H(5,N), H(8,N)*H(2,N)+H(2,N)*H(8,N),  H(8,N)*H(5,N)+H(5,N)*H(8,N), H(8,N)*H(8,N)]];
end

[U,S,V] = svd(v);
B = V(:,end);

y_0 = ( B(2)*B(4)-B(1)*B(5) )/( B(1)*B(3)-B(2)^2);
lambda = B(6)-(B(4)^2+y_0*(B(2)*B(4)-B(1)*B(5)))/B(1);
alpha = sqrt(lambda/B(1));
beta = sqrt(lambda*B(1)/(B(1)*B(3)-B(2)^2));
skewness = -B(2)*alpha^2*beta/lambda;
x_0 = skewness*y_0/alpha - B(4)*alpha^2/lambda;
disp('alphax, skew, alphay, x0, y0, k1, k2');

disp(alpha)
disp(skewness)
disp(beta)
disp(x_0)
disp(y_0)

%intrinsic matrix
K = [alpha, skewness, x_0; 0, beta, y_0; 0, 0, 1];

R = zeros(3,4,6);
for N = 1:numberImages
    sigma = 1/ norm(inv(K)*[H(1,N);H(4,N);H(7,N)]);
    r1 = sigma* inv(K)*[H(1,N);H(4,N);H(7,N)];
    r2 = sigma* inv(K)*[H(2,N);H(5,N);H(8,N)];
    r3 = cross(r1,r2);
    Cpworg = sigma* inv(K) * [H(3,N);H(6,N);H(9,N)];
    R(:,:,N) = [r1, r2, r3, Cpworg];
end

disp(R)