function histrogramUpdate(handles, currentEditedImage)

    %Split into RGB Channels
    Red = currentEditedImage(:,:,1);
    Green =currentEditedImage(:,:,2);
    Blue = currentEditedImage(:,:,3);

    %Get histValues for each channel
    [yRed, x] = imhist(Red);
    [yGreen, x] = imhist(Green);
    [yBlue, x] = imhist(Blue);

    %Plot them together in one plot
    axes(handles.histrogramAreaAxes);
    a=area([x,x,x],[yRed,yGreen,yBlue]);
    set(a(1),'FaceColor',[1 0 0]);
    set(a(2),'FaceColor',[0 1 0]);
    set(a(3),'FaceColor',[0 0 1]);
   
end