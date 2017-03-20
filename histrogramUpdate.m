function histrogramUpdate(handles, currentEditedImage)
intensityImage=rgb2gray(currentEditedImage);
%Split into RGB Channels
Red = currentEditedImage(:,:,1);
Green =currentEditedImage(:,:,2);
Blue = currentEditedImage(:,:,3);


%Get histValues for each channel
[yRed, x] = imhist(Red);
[yGreen, x] = imhist(Green);
[yBlue, x] = imhist(Blue);
[intensity,x]=imhist( intensityImage);
%Plot them together in one plot
axes(handles.histrogramAreaAxes);
a=area([x,x,x,x],[yRed,yGreen,yBlue,intensity]);
set(a(1),'FaceColor',[1 .1 0]);
set(a(2),'FaceColor',[0 1 0]);
set(a(3),'FaceColor',[0 1 1]);
set(a(4),'FaceColor',[0.8 0.8 0.8]);
set(gca,'Color',[0 0 0]);

end