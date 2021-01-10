
clear all
close all
clc

%%%%%%%%%%%%%%%%%
% Corner Points %
%%%%%%%%%%%%%%%%%

% Data from https://www.microsoft.com/en-us/research/project/a-flexible-new-technique-for-camera-calibration-2/?from=http%3A%2F%2Fresearch.microsoft.com%2F%7Ezhang%2Fcalib%2F

% This excerpt of code extracts the corner points from the individual files
% given on the moodle page

%This loads Model.txt which includes the co-ordinates of the corner points
%on the physical print of the checkerboard 

load Model.txt; %loads file

%loads all X co-ordinates into a 64x4 matrix by taking all rows from the
%odd numbered columns, for Y-co-ordinates does the same but with the even
%numbered columns

X = Model(:,1:2:end);
Y = Model(:,2:2:end);

%takes the 64x4 matrices generated before and reads down down each column
%to load all co-ordinates into a single column. This is then transposed so
%the co-ordinates are in a single row

X = X(:)'; %puts x co-ordinates  of corner points all in one row
Y = Y(:)'; %puts y co-ordinates of corner points all in one row

numberImages = 5; %number of images existing of the checkerboard in diff 
%orientations to be used for calibration

%loads files containing pixel co-ordinates of corner points from the 5 
%images of the checkerboard

load data1.txt;
load data2.txt;
load data3.txt;
load data4.txt;
load data5.txt;

%similar to code for Model.txt, this takes the x-coordinates from the odd 
%numbered columns, y-coordinates from the even numbered columns and puts
%them into 64x4 matrices. Then it places all the x-coordinates in one row
%and all the y-coordinates in a single row.

%Then these corner points are placed into the 5-layered matrix called image
%points

x = data1(:,1:2:end);
y = data1(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,1) = [x;y]; %extracts corner points of image 1 into matrix

x = data2(:,1:2:end);
y = data2(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,2) = [x;y]; %extracts corner points of image 2 into matrix

x = data3(:,1:2:end);
y = data3(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,3) = [x;y]; %extracts corner points of image 3 into matrix

x = data4(:,1:2:end);
y = data4(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,4) = [x;y]; %extracts corner points of image 4 into matrix

x = data5(:,1:2:end);
y = data5(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,5) = [x;y]; %extracts corner points of image 5 into matrix

%imagePoints has a 3rd dimension has it has 5 layers (1 for each image's 
%corner points)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continue with your own code %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%Used to populate 5 separate phi matrices to then be used with SVD to 
%find the H matrix
%Loads corner points of physical printout from X and Y
%Loads pixel x-coordinates of corner points from images from 1st row of
%imagePoints
%Loads pixel y-coordinates of corner points from images from 2nd row of
%imagePoints

%For loop to generate 5 layers as there will be 5 layers of 512x9 matrices
%used to find the H matrix for each of the 5 images used 

%The inner loop is used to populate each 512x9 matrix to help find the H
%matrix for each image

phi = zeros(512,9,5);
for N = 1:numberImages
    for i = 1:2:511
    j = (i+1)/2;
    phi(i,:,N) = [0, 0, 0, X(j), Y(j), 1, -imagePoints(2,j,N)* X(j),-imagePoints(2,j,N)* Y(j),-imagePoints(2,j,N)];
    phi(i+1,:,N)=[ X(j), Y(j),1, 0, 0, 0, -imagePoints(1,j,N)* X(j),-imagePoints(1,j,N)* Y(j),-imagePoints(1,j,N)];
    end
end
    
%Using singular value decomposition to find the h-values as we can note
%that they are the null space of the phi matrix

%h-values will be the column of V corresponding to the lowest singular 
%value in S which will be the last column as the singular values arranged in 
%descending order. This finds H matrix.

%There is a loop so that H matrix is found for each of the 5 images

%The variable H contains the h-values for image 1 in column 1, image 2 in
%column 2 etc.

H = zeros(9,5);
for N = 1:numberImages
    [U,S,V] = svd(phi(:,:,N));
    H(:,N) = V(:,end); %populate columsn of H-matrix using last column of V
end

%Finding the v and B matrices to extract the intrinsic parameters from H
%Equation for v found in section 3.5 of workshop
%Extracting the correct h-values to match from each column
%Repeated 5 times as there will be 10 rows in the v matrix (5*2) since
%there are 5 images

v = zeros(10,6);
for N = 1:numberImages
    v(2*N-1,:) = [H(1,N)*H(2,N), H(4,N)*H(2,N)+H(1,N)*H(5,N), H(4,N)*H(5,N), H(7,N)*H(2,N)+H(1,N)*H(8,N),  H(7,N)*H(5,N)+H(4,N)*H(8,N), H(7,N)*H(8,N)];
    v(2*N,:) = [[H(1,N)*H(1,N), H(4,N)*H(1,N)+H(1,N)*H(4,N), H(4,N)*H(4,N), H(7,N)*H(1,N)+H(1,N)*H(7,N),  H(7,N)*H(4,N)+H(4,N)*H(7,N), H(7,N)*H(7,N)]-[H(2,N)*H(2,N), H(5,N)*H(2,N)+H(2,N)*H(5,N), H(5,N)*H(5,N), H(8,N)*H(2,N)+H(2,N)*H(8,N),  H(8,N)*H(5,N)+H(5,N)*H(8,N), H(8,N)*H(8,N)]];
end

%Right hand side for equation with the transposes of v multiplied by 
%b values is 0 so we can use SVD to extract the b-values
%As before, these will be in the last column of the V matrix as that column
%will have the smallest singular value of S

[U,S,V] = svd(v);
B = V(:,end); %finds the b-matrix which is then used to find intrinsic params.

%Using the b-values and the equations on page 14 of the workshop we can
%extract the needed b values and apply them to the equations to find our
%extrinsic parameters

y_0 = ( B(2)*B(4)-B(1)*B(5) )/( B(1)*B(3)-B(2)^2);
lambda = B(6)-(B(4)^2+y_0*(B(2)*B(4)-B(1)*B(5)))/B(1);
alpha = sqrt(lambda/B(1));
beta = sqrt(lambda*B(1)/(B(1)*B(3)-B(2)^2));
skewness = -B(2)*alpha^2*beta/lambda;
x_0 = skewness*y_0/alpha - B(4)*alpha^2/lambda;

%Used to display the 6 instrinsic parameters of our camera 
disp('alphax, skew, alphay, x0, y0, k1, k2');

disp(alpha)
disp(skewness)
disp(beta)
disp(x_0)
disp(y_0)

%intrinsic matrix K formed
K = [alpha, skewness, x_0; 0, beta, y_0; 0, 0, 1];

%When K is known, extrinsic parameters can be recovered but can only be
%done one image at a time as each image take differently

%Loop to find the extrinsic parameters for each image using the equations
%in Section 3.6 of the workshop.
%5 layers as each image will produce different extrinsic parameters
%Each layer contains a 3x4 matrix with the each column containing a
%separate extrinsic parameter

R = zeros(3,4,5);
for N = 1:numberImages
    sigma = 1/ norm(inv(K)*[H(1,N);H(4,N);H(7,N)]);
    r1 = sigma* inv(K)*[H(1,N);H(4,N);H(7,N)];
    r2 = sigma* inv(K)*[H(2,N);H(5,N);H(8,N)];
    r3 = cross(r1,r2);
    Cpworg = sigma* inv(K) * [H(3,N);H(6,N);H(9,N)];
    R(:,:,N) = [r1, r2, r3, Cpworg];
end

disp(R) %displays extrinsic parameters to compare to Zhang's results
    

        
        


