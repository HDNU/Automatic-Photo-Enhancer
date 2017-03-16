clear all
close all
a= imread('images.jpg');
b = rgb2ycbcr(a);
imshow(a);
figure
imshow(b);
figure
imshow(b(:,:,1));
figure
imshow(b(:,:,2));
figure
imshow(b(:,:,3));

num=10;
b(:,:,1) =medfilt2(b(:,:,1),[num num]);
%b(:,:,3) =medfilt2(b(:,:,3),[num num]);
c= ycbcr2rgb(b);
figure
imshow(c)