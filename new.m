clear all 
close all

a = imread('image1.jpg');
%C = get(hObject,'Value')/10;
C=255;
filtered1 = a(:,:,1);
filtered2 = a(:,:,2);
filtered3 = a(:,:,3);

F =(259*(C+255))/(255*(259-C));

filtered1 =uint8(F*(filtered1-128)+128);
filtered2 =uint8(F*(filtered2-128)+128);
filtered3 =uint8(F*(filtered3-128)+128);

filtered = a;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
imshow(filtered)
figure
imhist(filtered(:,:,1));