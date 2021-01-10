clear all
close all
clc

camlist = webcamlist;
cam = webcam(1);
preview(cam);

pause(15.0);
img = snapshot(cam);
figure,image(img);

imwrite(img,'CheckerBoard.png');
clear cam