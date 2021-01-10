clear all
close all
clc

numImages = 6;
camList = webcamlist;
cam = webcam(1);
preview(cam);

for idx = 1:numImages
    
    pause(10.0);
    img(:,:,:,idx) = snapshot(cam);
    figure,image(img(:,:,:,idx));
    
    fname = sprintf('Image%d.png',idx);
    imwrite(img(:,:,:,idx),fname);
end

clear cam