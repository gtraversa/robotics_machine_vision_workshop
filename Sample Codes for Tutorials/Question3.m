%%%%%%%%%%%%%%%%%%%%%%%%%%
% AIM: MOMENTS & INERTIA %
%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%%%%%%%%%%%%%%%%
% Import Image %
%%%%%%%%%%%%%%%%
 
% I = imread('WhiteCircle.tif'); 
% I = imread('WhiteEllipseA.tif'); 
% I = imread('WhiteEllipseB.tif'); 
% I = imread('WhiteEllipseC.tif'); 
% I = imread('WhiteHexagon.tif'); 
% I = imread('WhiteRectangle.tif'); 
% I = imread('WhiteRectangleB.tif'); 
% I = imread('WhiteSquare.tif'); 
I = imread('WhiteTriangle.tif'); 

% NOTE: I has 3 layers (RGB) even though it looks "black and white".

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transform into Greyscale %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IRed = double(I(:,:,1));         
IGreen = double(I(:,:,2));
IBlue = double(I(:,:,3));
IGrey = (IRed+IGreen+IBlue)/3;
I = uint8(IGrey);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Histogram and Original Image %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure, imhist(I)                   
figure, imshow(I)
title('Original')
I=double(I);
[m,n]=size(I);

%%%%%%%%%%%%%%%%
% Thresholding %
%%%%%%%%%%%%%%%%

Ithreshold = zeros(m,n);

for i = 1:m
    for j = 1:n
        if I(i,j) > 220              % INSTRUCTION: TUNE THIS NUMBER BASED ON HISTOGRAM
            Ithreshold(i,j) = 255;
        else
            Ithreshold(i,j) = 0;
        end
    end
end

Ithreshold = uint8(Ithreshold);
figure,imshow(Ithreshold)
title('Threshold')

%%%%%%%%%%%%%%%%%%%%
% Change to Binary %
%%%%%%%%%%%%%%%%%%%%

Ibw = imbinarize(Ithreshold);
figure,imshow(Ibw);
title('Binary')

%%%%%%%%%%%%%%%%%%%%%
% Find Bounding Box %
%%%%%%%%%%%%%%%%%%%%%

imin_old = m+1;
imax_old = 0;
jmin_old = n+1;
jmax_old = 0;
for i = 1:m % rows from up to down
    for j = 1:n % columns from left to right
        if Ibw(i,j) == 1
            if i < imin_old
                imin_old = i;
            end
            if i > imax_old
                imax_old = i;
            end
            if j < jmin_old
                jmin_old = j;
            end
            if j > jmax_old
                jmax_old = j;
            end
        end
    end
end
imin = imin_old     % first row
imax = imax_old     % last row
jmin = jmin_old     % first column
jmax = jmax_old     % last column

hold on, plot([jmin jmin jmax jmax jmin],[imin imax imax imin imin],'g')

%%%%%%%%%%%%%%%%%%%%%
% Find Middle Point %
%%%%%%%%%%%%%%%%%%%%%

xCenter = (jmax+jmin)/2
yCenter = (imax+imin)/2

hold on, plot(xCenter, yCenter, 'r+')

%%%%%%%%%%%%%%%%%
% Zeroth Moment %
%%%%%%%%%%%%%%%%%

Moment00 = 0;
for i = 1:m
    for j = 1:n
        if Ibw(i,j) == 1
            Moment00 = Moment00 + 1;
        end
    end
end
Moment00

%%%%%%%%%%%%%%%
% M10 and M01 %
%%%%%%%%%%%%%%%

Moment10 = 0;
Moment01 = 0;
Moment11 = 0;
Moment20 = 0;
Moment02 = 0;
for i = 1:m % rows from top to bottom
    for j = 1:n % columns from left to right
        Moment10 = Moment10 + j*Ibw(i,j);
        Moment01 = Moment01 + i*Ibw(i,j);
        Moment11 = Moment11 + j*i*Ibw(i,j);
        Moment20 = Moment20 + j^2*Ibw(i,j);
        Moment02 = Moment02 + i^2*Ibw(i,j);
    end
end
xCentroid = Moment10/Moment00
yCentroid = Moment01/Moment00
        
%%%%%%%%%%%%%%%%%%%
% Central Moments %
%%%%%%%%%%%%%%%%%%%

u00 = Moment00;
u01 = 0;
u10 = 0;
u11 = Moment11 - xCentroid*Moment01;
u20 = Moment20 - xCentroid*Moment10;
u02 = Moment02 - yCentroid*Moment01;

Inertia = [u20 u11;u11 u02];

[V,D] = eig(Inertia);
[d,ind] = sort(diag(D));    % Sort in ascending order
Ds = D(ind,ind);
Vs = V(:,ind);

Major = 2*sqrt(Ds(2,2)/Moment00)
Minor = 2*sqrt(Ds(1,1)/Moment00)

Angle = atan(Vs(2,2)/Vs(1,2))   % Note: Vs corresponding to largest eigenvalue

figure,imshow(Ibw);
Pt1 = [xCentroid yCentroid];
Pt2 = [xCentroid + Major*cos(Angle) yCentroid + Major*sin(Angle)];
hold on, plot([Pt1(1) Pt2(1)],[Pt1(2) Pt2(2)],'r')


%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Perimeter %
%%%%%%%%%%%%%%%%%%%%%%%

% Move from left to right, row by row.
% If current pixel is white
%    If left pixel is black and right pixel is white --> Take as boundary
%    Else if left pixel is white and right pixel is black --> Boundary
%    If top pixel is black and bottom pixel is white --> Take as boundary
%    Else if top pixel is white and bottom pixel is black --> Boundary
% Add the boundaries up

Perimeter = 0;
for i = 2:m-1 % rows from top to bottom
    for j = 2:n-1 % columns from left to right
        if Ibw(i,j) == 1
            if (Ibw(i,j-1) == 0) && (Ibw(i,j+1) == 1)
                Perimeter = Perimeter + 1;
            elseif (Ibw(i,j-1) == 1) && (Ibw(i,j+1) == 0)
                Perimeter = Perimeter + 1; 
            elseif (Ibw(i-1,j) == 0) && (Ibw(i+1,j) == 1)
                Perimeter = Perimeter + 1;
            elseif (Ibw(i-1,j) == 1) && (Ibw(i+1,j) == 0)
                Perimeter = Perimeter + 1; 
            end
        end
                
    end
end
Perimeter
BW2 = bwperim(Ibw,4);
PerimeterMatlab = sum(sum(BW2)) % Using Matlab's toolbox, just to confirm


%%%%%%%%%%%%%%%
% Circularity %
%%%%%%%%%%%%%%%

Circularity = 4*pi*Moment00/Perimeter^2






