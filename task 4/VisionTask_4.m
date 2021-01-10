clear all
close all
clc

I = imread('snap.bmp'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transform into Greyscale %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IRed = double(I(:,:,1));         
IGreen = double(I(:,:,2));
IBlue = double(I(:,:,3));
I_grey = (IRed+IGreen+IBlue)/3;
I_grey = uint8(I_grey);
I_cyl = I_grey;
I_cube = I_grey;

%%%%%%%%%%%%%%%%%%%%%%%%
% Find Yellow Cylinder %
%%%%%%%%%%%%%%%%%%%%%%%%

[m,n] = size(IRed);

for i = 1:m
    for j = 1:n 
        if (IRed(i,j) > 250) && (IGreen(i,j) > 250) && (IBlue(i,j) < 50)
            % Use < or > instead of == so that algorithm is more robust
            % towards redness of apple and lighting conditions
            I_cyl(i,j) = 255;
        else
            I_cyl(i,j) = 0; 
        end
    end
end
%  figure, imshow(I_cyl)
%  title('Cylinder');
 
 %%%%%%%%%%%%%%%%%%%%
% Change to Binary %
%%%%%%%%%%%%%%%%%%%%

Ibw_cyl = imbinarize(I_cyl);
figure,imshow(Ibw_cyl);
title('Binary-cylinder')


%%%%%%%%%%%%%%%%%
% Zeroth Moment %
%%%%%%%%%%%%%%%%%

Moment00 = 0;
for i = 1:m
    for j = 1:n
        if Ibw_cyl(i,j) == 1
            Moment00 = Moment00 + 1;
        end
    end
end

 
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
        Moment10 = Moment10 + j*Ibw_cyl(i,j);
        Moment01 = Moment01 + i*Ibw_cyl(i,j);
        Moment11 = Moment11 + j*i*Ibw_cyl(i,j);
        Moment20 = Moment20 + j^2*Ibw_cyl(i,j);
        Moment02 = Moment02 + i^2*Ibw_cyl(i,j);
    end
end
xCentroid_cyl = Moment10/Moment00
yCentroid_cyl = Moment01/Moment00

%%%%%%%%% For cube
 figure, imhist(I_grey);

for i = 1:m
    for j = 1:n 
        if ((IRed(i,j) < 50) && (IGreen(i,j) > 250) && (IBlue(i,j) > 250)) 
            I_cube(i,j) = 255;
        else
            I_cube(i,j) = 0; 
        end
    end
end
%  figure, imshow(I_cube)
%  title('Cube');
 
 %%%%%%%%%%%%%%%%%%%%
% Change to Binary for the Cube %
%%%%%%%%%%%%%%%%%%%%

Ibw_cube = imbinarize(I_cube);
figure,imshow(Ibw_cube);
title('Binary-cube')

%%%%%%%%%%%%%%%%%
% Zeroth Moment %
%%%%%%%%%%%%%%%%%

Moment00 = 0;
for i = 1:m
    for j = 1:n
        if Ibw_cube(i,j) == 1
            Moment00 = Moment00 + 1;
        end
    end
end

 
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
        Moment10 = Moment10 + j*Ibw_cube(i,j);
        Moment01 = Moment01 + i*Ibw_cube(i,j);
        Moment11 = Moment11 + j*i*Ibw_cube(i,j);
        Moment20 = Moment20 + j^2*Ibw_cube(i,j);
        Moment02 = Moment02 + i^2*Ibw_cube(i,j);
    end
end
xCentroid_cube = Moment10/Moment00
yCentroid_cube = Moment01/Moment00

Pt2 = [xCentroid_cube yCentroid_cube];
Pt1 = [xCentroid_cyl yCentroid_cyl];

figure, imshow(I_grey);
hold on, plot([Pt1(1) Pt2(1)],[Pt1(2) Pt2(2)],'r+');

% Calculate Perimeter %
%%%%%%%%%%%%%%%%%%%%%%%

% Move from left to right, row by row.
% If current pixel is white
%    If left pixel is black and right pixel is white --> Take as boundary
%    Else if left pixel is white and right pixel is black --> Boundary
%    If top pixel is black and bottom pixel is white --> Take as boundary
%    Else if top pixel is white and bottom pixel is black --> Boundary
% Add the boundaries up

% Perimeter = 0;
% for i = 2:m-1 % rows from top to bottom
%     for j = 2:n-1 % columns from left to right
%         if Ibw_cube(i,j) == 1
%             if (Ibw_cube(i,j-1) == 0) && (Ibw_cube(i,j+1) == 1)
%                 Perimeter = Perimeter + 1;
%             elseif (Ibw_cube(i,j-1) == 1) && (Ibw_cube(i,j+1) == 0)
%                 Perimeter = Perimeter + 1; 
%             elseif (Ibw_cube(i-1,j) == 0) && (Ibw_cube(i+1,j) == 1)
%                 Perimeter = Perimeter + 1;
%             elseif (Ibw_cube(i-1,j) == 1) && (Ibw_cube(i+1,j) == 0)
%                 Perimeter = Perimeter + 1; 
%             end
%         end
%                 
%     end
% end
% Perimeter
% BW2 = bwperim(Ibw_cube,4);
% PerimeterMatlab = sum(sum(BW2)) % Using Matlab's toolbox, just to confirm


%%%%%%%%%%%%%%%
% Circularity %
%%%%%%%%%%%%%%%

% Circularity = 4*pi*Moment00/Perimeter^2
% 
% %%%%%%%%%%%%%%%%%%%
% % Central Moments %
% %%%%%%%%%%%%%%%%%%%
% 
% u00 = Moment00;
% u01 = 0;
% u10 = 0;
% u11 = Moment11 - xCentroid_cube*Moment01;
% u20 = Moment20 - xCentroid_cube*Moment10;
% u02 = Moment02 - yCentroid_cube*Moment01;
% 
% Inertia = [u20 u11;u11 u02];
% 
% [V,D] = eig(Inertia);
% [d,ind] = sort(diag(D));    % Sort in ascending order
% Ds = D(ind,ind);
% Vs = V(:,ind);
% 
% Major = 2*sqrt(Ds(2,2)/Moment00)
% Minor = 2*sqrt(Ds(1,1)/Moment00)
% 
% Angle = atan(Vs(2,2)/Vs(1,2))   % Note: Vs corresponding to largest eigenvalue
% 
% figure,imshow(I_grey);
% Pt1 = [xCentroid_cube yCentroid_cube];
% Pt2 = [xCentroid_cube + Major*cos(Angle) yCentroid_cube + Major*sin(Angle)];
% hold on, plot([Pt1(1) Pt2(1)],[Pt1(2) Pt2(2)],'r')

% IedgexyCube = zeros(m,n); 
% 
% for i = 2:m-1
%     for j = 2:n-1
%         IedgexyCube(i,j) = (-1*Ibw_cube(i-1,j-1)-1*Ibw_cube(i-1,j)-1*Ibw_cube(i-1,j+1)...
%             -1*Ibw_cube(i,j-1)+8*Ibw_cube(i,j)-1*Ibw_cube(i,j+1)...
%             -1*Ibw_cube(i+1,j-1)-1*Ibw_cube(i+1,j)-1*Ibw_cube(i+1,j+1));
%     end
% end
% 
% IedgexyCube = abs(IedgexyCube);
% figure(15),imshow(IedgexyCube)
% title('Laplacian Edge Detection Box')
% 
% IedgexyCyl = zeros(m,n); 
% 
% for i = 2:m-1
%     for j = 2:n-1
%         IedgexyCyl(i,j) = (-1*Ibw_cyl(i-1,j-1)-1*Ibw_cyl(i-1,j)-1*Ibw_cyl(i-1,j+1)...
%             -1*Ibw_cyl(i,j-1)+8*Ibw_cyl(i,j)-1*Ibw_cyl(i,j+1)...
%             -1*Ibw_cyl(i+1,j-1)-1*Ibw_cyl(i+1,j)-1*Ibw_cyl(i+1,j+1));
%     end
% end
% 
% IedgexyCyl = abs(IedgexyCyl);
% figure(20),imshow(IedgexyCyl)
% title('Laplacian Edge Detection Cylinder')
% 
% %%%%%%%%%%%%%%%%%
% % Count Corners %
% %%%%%%%%%%%%%%%%%
% 
% CornerBox = 0;
% CornerBoxThreshold = 5;
% 
% for i = 2:m-1
%     for j = 2:n-1
%         if IedgexyCube(i,j-1) > CornerBoxThreshold 
%             continue
%             % If pixel on the left is already a corner,
%             % skip one for iteration to avoid counting "cluster" of corners more
%             % than once. This method is ad-hoc and may need to change for
%             % other images.
%         end
%         
%         if IedgexyCube(i,j) > CornerBoxThreshold  
%             CornerBox = CornerBox + 1;    
%         end
%     end
% end
% CornerBox % returns a value of 6 becasue of the internal corners given by the cross pattern
% 
% 
% CornerCyl = 0;
% CornerCylThreshold = 5;
% 
% for i = 2:m-1
%     for j = 2:n-1
%         if IedgexyCyl(i,j-1) > CornerCylThreshold 
%             continue
%             % If pixel on the left is already a corner,
%             % skip one for iteration to avoid counting "cluster" of corners more
%             % than once. This method is ad-hoc and may need to change for
%             % other images.
%         end
%         
%         if IedgexyCyl(i,j) > CornerCylThreshold  
%             CornerCyl = CornerCyl + 1;    
%         end
%     end
% end
% CornerCyl
