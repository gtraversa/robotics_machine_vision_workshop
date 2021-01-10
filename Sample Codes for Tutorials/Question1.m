%%%%%%%%%%%%%%%%%%%%
% AIM: FIND APPLES %
%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%%%%%%%%%%%%%%%%
% Import Image %
%%%%%%%%%%%%%%%%

I = imread('appletree.png');
figure, imshow(I)
title('appletree')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get intensities of RGB layers %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IRed = double(I(:,:,1));
IGreen = double(I(:,:,2));
IBlue = double(I(:,:,3));

%%%%%%%%%%%%%%%%%%%
% Find red apples %
%%%%%%%%%%%%%%%%%%%

[n,m] = size(IRed);
AppleCount = 0;
for i = 1:n 
    for j = 1:m 
        if (IRed(i,j) > 225) && (IGreen(i,j) < 100) && (IBlue(i,j) < 100)
            % Use < or > instead of == so that algorithm is more robust
            % towards redness of apple and lighting conditions
            AppleCount = AppleCount + 1;
            fprintf('apple postion row %d, column %d.\n',i,j);   
        end
    end
end
fprintf('Total number of apples is %d.\n',AppleCount);
