%%%%%%%%%%%%%%%%
% AIM: CORNERS %
%%%%%%%%%%%%%%%%

clear all
close all
clc

%%%%%%%%%%%%%%%%
% Import Image %
%%%%%%%%%%%%%%%%
 
% I = imread('WhiteSquare.tif'); 
% CornerThreshold = 4;

% I = imread('WhiteRectangle.tif');
% CornerThreshold = 4;

% I = imread('WhiteTriangle.tif'); 
% CornerThreshold = 4;

I = imread('WhiteHexagon.tif'); 
CornerThreshold = 3;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Laplacian Edge Detection %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Note: Laplacian works well; Sobel and Prewitt less so

Iedgexy = zeros(m,n); 

for i = 2:m-1
    for j = 2:n-1
        Iedgexy(i,j) = (-1*Ibw(i-1,j-1)-1*Ibw(i-1,j)-1*Ibw(i-1,j+1)...
            -1*Ibw(i,j-1)+8*Ibw(i,j)-1*Ibw(i,j+1)...
            -1*Ibw(i+1,j-1)-1*Ibw(i+1,j)-1*Ibw(i+1,j+1));
    end
end

Iedgexy = abs(Iedgexy);
figure,imshow(Iedgexy)
title('Laplacian Edge Detection')

%%%%%%%%%%%%%%%%%
% Count Corners %
%%%%%%%%%%%%%%%%%

Corner = 0;

for i = 2:m-1
    for j = 2:n-1
        if Iedgexy(i,j-1) > CornerThreshold 
            continue
            % If pixel on the left is already a corner,
            % skip one for iteration to avoid counting "cluster" of corners more
            % than once. This method is ad-hoc and may need to change for
            % other images.
        end
        
        if Iedgexy(i,j) > CornerThreshold  
            Corner = Corner + 1;    
        end
    end
end
Corner







