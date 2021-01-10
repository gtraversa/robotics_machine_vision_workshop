
clear all
close all
clc

%%%%%%%%%%%%%%%%%
% Corner Points %
%%%%%%%%%%%%%%%%%

% Data from https://www.microsoft.com/en-us/research/project/a-flexible-new-technique-for-camera-calibration-2/?from=http%3A%2F%2Fresearch.microsoft.com%2F%7Ezhang%2Fcalib%2F

load Model.txt;

X = Model(:,1:2:end);
Y = Model(:,2:2:end);

X = X(:)';
Y = Y(:)';

numberImages = 5; 

load data1.txt;
load data2.txt;
load data3.txt;
load data4.txt;
load data5.txt;

x = data1(:,1:2:end);
y = data1(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,1) = [x;y];

x = data2(:,1:2:end);
y = data2(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,2) = [x;y];

x = data3(:,1:2:end);
y = data3(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,3) = [x;y];

x = data4(:,1:2:end);
y = data4(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,4) = [x;y];

x = data5(:,1:2:end);
y = data5(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,5) = [x;y];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continue with your own code %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%the phi matrix is calculatet to then extrapolate the intrinsic parameters by finding H
phi = zeros(512,9,5);
for N = 1:numberImages
    for i = 1:2:511
    j = (i+1)/2;
    phi(i,:,N) = [0, 0, 0, X(j), Y(j), 1, -imagePoints(2,j,N)* X(j),-imagePoints(2,j,N)* Y(j),-imagePoints(2,j,N)];
    phi(i+1,:,N)=[ X(j), Y(j),1, 0, 0, 0, -imagePoints(1,j,N)* X(j),-imagePoints(1,j,N)* Y(j),-imagePoints(1,j,N)];
    end
end
 

%H matrix found by performing singular value decompositon of phi matrix
H = zeros(9,5);
for N = 1:numberImages
    [U,S,V] = svd(phi(:,:,N));
    H(:,N) = V(:,end);
end

%Intrinsic parameters revocered from H matrix 
v = zeros(10,6);
for N = 1:numberImages
    v(2*N-1,:) = [H(1,N)*H(2,N), H(4,N)*H(2,N)+H(1,N)*H(5,N), H(4,N)*H(5,N), H(7,N)*H(2,N)+H(1,N)*H(8,N),  H(7,N)*H(5,N)+H(4,N)*H(8,N), H(7,N)*H(8,N)];
    v(2*N,:) = [[H(1,N)*H(1,N), H(4,N)*H(1,N)+H(1,N)*H(4,N), H(4,N)*H(4,N), H(7,N)*H(1,N)+H(1,N)*H(7,N),  H(7,N)*H(4,N)+H(4,N)*H(7,N), H(7,N)*H(7,N)]-[H(2,N)*H(2,N), H(5,N)*H(2,N)+H(2,N)*H(5,N), H(5,N)*H(5,N), H(8,N)*H(2,N)+H(2,N)*H(8,N),  H(8,N)*H(5,N)+H(5,N)*H(8,N), H(8,N)*H(8,N)]];
end

[U,S,V] = svd(v);
B = V(:,end);

%Intrinsic parameters calculated and stored in parameter matrix K
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


%For each image the extrinsic parameters R may be recovered
R = zeros(3,4,5);
for N = 1:numberImages
    sigma = 1/ norm(inv(K)*[H(1,N);H(4,N);H(7,N)]);
    r1 = sigma* inv(K)*[H(1,N);H(4,N);H(7,N)];
    r2 = sigma* inv(K)*[H(2,N);H(5,N);H(8,N)];
    r3 = cross(r1,r2);
    Cpworg = sigma* inv(K) * [H(3,N);H(6,N);H(9,N)];
    R(:,:,N) = [r1, r2, r3, Cpworg];
end

disp(R)