function varargout = apes(varargin)
% APES MATLAB code for apes.fig
%      APES, by itself, creates a new APES or raises the existing
%      singleton*.
%
%      H = APES returns the handle to a new APES or the handle to
%      the existing singleton*.
%
%      APES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in APES.M with the given input arguments.
%
%      APES('Property','Value',...) creates a new APES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before apes_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to apes_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help apes


% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @apes_OpeningFcn, ...
    'gui_OutputFcn',  @apes_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before apes is made visible.
function apes_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to apes (see VARARGIN)
I = imread('default.jpg');
axes(handles.axesImage);
imshow(I);
global vignetteAmount vignetteMidpoint vignetteFeather;
vignetteAmount = 0;
vignetteMidpoint = 0.5;
vignetteFeather = 0.5;

% Choose default command line output for apes
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes apes wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = apes_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in selectPicturePushbutton.
function selectPicturePushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to selectPicturePushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage originalImage hueImage satImage colour_slide luminance_slide fuzzyImage vignetteImage;
global path sze;
colour_slide =0;
luminance_slide=0;
[path, Cancel] = imgetfile();

if Cancel
    msgbox(sprintf('Error'),'Error','Error');
    return
end
currentEditedImage = imread(path);
fuzzyImage = currentEditedImage;
currentEditedImage = im2double(currentEditedImage);
originalImage = currentEditedImage;
vignetteImage = currentEditedImage;
sze = size(currentEditedImage);
if (length(sze)==2)
    sze=[sze,1];
end
sze = sze(3);
axes(handles.axesImage);
imshow(currentEditedImage);

size2 = size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(1,3)==3)
    hueImage=rgb2hsv(currentEditedImage);
    step=round(size2(1,2)/200);
    i=0;
    for l=1:step:200*step
        
        hueImage(:,l,1)=i/200;
        i=i+1;
    end
    
    hueImage=hueImage(1:20,1:step:200*step,:) ;
    
    % im2(:,:,2)=0.75;
    % im2(:,:,3)=0.75;
    axes(handles.Hue);
    imshow(hueImage);
    
    satImage=rgb2hsv(currentEditedImage);
    i=0;
    for l=1:step:200*step
        satImage(1,l,2)=i/200;
        i=i+1;
    end
    
    satImage= satImage(1:20,1:step:200*step,:) ;
    
    % im2(:,:,2)=0.75;
    % im2(:,:,3)=0.75;
    axes(handles.Saturation);
    imshow( satImage);
    
    histrogramUpdate(handles, currentEditedImage);
end

set(handles.vignetteAmountSlider, 'value', 0.5);
set(handles.vignetteMidpointSlider, 'value', 0.5);
set(handles.vignetteFeatherSlider, 'value', 0.5);
set(handles.vaText, 'String', 0);
set(handles.vmText, 'String', 50);
set(handles.vfText, 'String', 50);
set(handles.gaussianSlider, 'value', 0.0);
set(handles.wienerSlider, 'value', 0.0);
set(handles.medianSlider, 'value', 0.0);
set(handles.fuzzySlider, 'value', 0.0);
set(handles.colourNoiseSlider, 'value', 0.0);
set(handles.luminanaceNoiseSlider, 'value', 0.0);
set(handles.blurRemovalSlider, 'value', 0.0);
set(handles.sharpenSlider, 'value', 0.0);
set(handles.shadowRecoSlider, 'value', 0.0);



% --- Executes during object creation, after setting all properties.
function text8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function text10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns calle


% --- Executes on button press in histogramsPushbutton.
function histogramsPushbutton_Callback(hObject, eventdata, handles)
% h = uicontrol('Style','text','String','Hello world','Position',[200 420 100 20]);
global currentEditedImage;
% MultiSlider

histrogramUpdate(handles, currentEditedImage);


% hObject    handle to histogramsPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on slider movement.
function expostureAdjustSlider_Callback(hObject, eventdata, handles)
% hObject    handle to expostureAdjustSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');

filtered = currentEditedImage;
filtered=filtered*exp(-0.0205)*2^((0.4385/log(2))*val);

set(handles.text44,'string',val);

axes(handles.axesImage);
imshow(filtered);

histrogramUpdate(handles, filtered);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function expostureAdjustSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to expostureAdjustSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function contrastAdjustSlider_Callback(hObject, eventdata, handles)
% hObject    handle to contrastAdjustSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  currentEditedImage sze;
C = (get(hObject,'Value'));

filtered = currentEditedImage;
F =2^(C+1)-1;

filtered(:,:,1) =F*(filtered(:,:,1)-0.5)+0.5;

if(sze==3)
    
    filtered(:,:,2) =F*(filtered(:,:,2)-0.5)+0.5;
    filtered(:,:,3) =F*(filtered(:,:,3)-0.5)+0.5;

end

axes(handles.axesImage);
imshow(filtered);

histrogramUpdate(handles, currentEditedImage);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function contrastAdjustSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to contrastAdjustSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in cropPushbutton.
function cropPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to cropPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage satImage hueImage;
axes(handles.Saturation);
imshow(satImage);
axes(handles.Hue);
imshow(hueImage);
axes(handles.axesImage);
imshow(currentEditedImage);
croppedImage = imcrop(currentEditedImage);
currentEditedImage = croppedImage;
axes(handles.axesImage);
imshow(currentEditedImage);

histrogramUpdate(handles, currentEditedImage);


% --- Executes on button press in saveImagePushbutton.
function saveImagePushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to saveImagePushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[FileName,saves] = uiputfile('*jpg', 'Save As');
if (saves==0)
    msgbox(sprintf('Specify The File Name'),'Error','Error');
    return
end

frame = getframe(handles.axesImage);
im = frame2im(frame);
saves =strcat(saves,FileName);
saves =strcat(saves,'.jpg');
imwrite(im, saves,'jpg')

% --- Executes during object creation, after setting all properties.
function selectPicturePushbutton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to selectPicturePushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on slider movement.



% --- Executes on slider movement.
function vignetteAmountSlider_Callback(hObject, eventdata, handles)
% hObject    handle to vignetteAmountSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global currentEditedImage vignetteAmount vignetteMidpoint vignetteImage vignetteFeather;
set(handles.vaText, 'String', floor(100*(2*get(hObject,'Value')-1)));
vignetteAmount = 2*get(hObject,'Value')-1;
[nr, nc, nChannels]=size(currentEditedImage);
cx=ceil(nc/2);
cy=ceil(nr/2);

radius = vignetteMidpoint*max(cx,cy);
maxDistance = sqrt(cx.^2 + cy.^2)-radius;

vignetteImage=currentEditedImage;
for k = 1:nChannels
    for i=1:nr
        for j=1:nc
            distanceFromCenter = sqrt((abs(i-cy).^2)+(abs(j-cx).^2));
            if(distanceFromCenter>radius)
                scale= abs(vignetteAmount*(distanceFromCenter-radius)/maxDistance);
                if (vignetteAmount<0)
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)*(1-scale)-vignetteFeather*2*scale;
                else
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)/(1-scale)+vignetteFeather*2*scale;
                end
            end
        end
    end
end
% currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(vignetteImage);
histrogramUpdate(handles, vignetteImage);


% --- Executes during object creation, after setting all properties.
function vignetteAmountSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vignetteAmountSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function vignetteMidpointSlider_Callback(hObject, eventdata, handles)
% hObject    handle to vignetteMidpointSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global currentEditedImage vignetteAmount vignetteMidpoint vignetteImage vignetteFeather;
set(handles.vmText, 'String', floor(100*get(hObject,'Value')));
vignetteMidpoint = get(hObject,'Value');
[nr, nc, nChannels]=size(currentEditedImage);
cx=ceil(nc/2);
cy=ceil(nr/2);

radius = vignetteMidpoint*max(cx,cy);
maxDistance = sqrt(cx.^2 + cy.^2)-radius;

vignetteImage=currentEditedImage;
for k = 1:nChannels
    for i=1:nr
        for j=1:nc
            distanceFromCenter = sqrt((abs(i-cy).^2)+(abs(j-cx).^2));
            if(distanceFromCenter>radius)
                scale= abs(vignetteAmount*(distanceFromCenter-radius)/maxDistance);
                if (vignetteAmount<0)
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)*(1-scale)-vignetteFeather*2*scale;
                else
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)/(1-scale)+vignetteFeather*2*scale;
                end
            end
        end
    end
end
% currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(vignetteImage);
histrogramUpdate(handles, vignetteImage);


% --- Executes during object creation, after setting all properties.
function vignetteMidpointSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vignetteMidpointSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes on slider movement.
function vignetteFeatherSlider_Callback(hObject, eventdata, handles)
% hObject    handle to vignetteFeatherSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global currentEditedImage vignetteAmount vignetteMidpoint vignetteImage vignetteFeather;
set(handles.vfText, 'String', floor(100*get(hObject,'Value')));
vignetteFeather = -1*get(hObject,'Value')+1;
[nr, nc, nChannels]=size(currentEditedImage);
cx=ceil(nc/2);
cy=ceil(nr/2);

radius = vignetteMidpoint*max(cx,cy);
maxDistance = sqrt(cx.^2 + cy.^2)-radius;

vignetteImage=currentEditedImage;
for k = 1:nChannels
    for i=1:nr
        for j=1:nc
            distanceFromCenter = sqrt((abs(i-cy).^2)+(abs(j-cx).^2));
            if(distanceFromCenter>radius)
                scale= abs(vignetteAmount*(distanceFromCenter-radius)/maxDistance);
                if (vignetteAmount<0)
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)*(1-scale)-vignetteFeather*2*scale;
                else
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)/(1-scale)+vignetteFeather*2*scale;
                end
            end
        end
    end
end
% currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(vignetteImage);
histrogramUpdate(handles, vignetteImage);


% --- Executes during object creation, after setting all properties.
function vignetteFeatherSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vignetteFeatherSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in vignetteDonePushbutton.
function vignetteDonePushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to vignetteDonePushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage vignetteImage;
currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(currentEditedImage);
histrogramUpdate(handles, currentEditedImage);
set(handles.vignetteAmountSlider, 'value', 0.5);
set(handles.vignetteMidpointSlider, 'value', 0.5);
set(handles.vignetteFeatherSlider, 'value', 0.5);
set(handles.vaText, 'String', 0);
set(handles.vmText, 'String', 50);
set(handles.vfText, 'String', 50);



% --- Executes on button press in rivert2OriginalPushbutton.
function rivert2OriginalPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to rivert2OriginalPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global originalImage currentEditedImage vignetteImage;
currentEditedImage = originalImage;
vignetteImage = currentEditedImage;
axes(handles.axesImage);
imshow(currentEditedImage);
currentEditedImage = im2double(currentEditedImage);
histrogramUpdate(handles, currentEditedImage);

% set default in color profile option
clearPushButton(handles);
set(handles.Default,'Value',1);

set(handles.vignetteAmountSlider, 'value', 0.5);
set(handles.vignetteMidpointSlider, 'value', 0.5);
set(handles.vignetteFeatherSlider, 'value', 0.5);
set(handles.vaText, 'String', 0);
set(handles.vmText, 'String', 50);
set(handles.vfText, 'String', 50);
set(handles.gaussianSlider, 'value', 0.0);
set(handles.wienerSlider, 'value', 0.0);
set(handles.medianSlider, 'value', 0.0);
set(handles.fuzzySlider, 'value', 0.0);
set(handles.colourNoiseSlider, 'value', 0.0);
set(handles.luminanaceNoiseSlider, 'value', 0.0);
set(handles.blurRemovalSlider, 'value', 0.0);
set(handles.sharpenSlider, 'value', 0.0);
set(handles.shadowRecoSlider, 'value', 0.0);



% --- Executes on button press in undoLastEditPushbutton.
function undoLastEditPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to undoLastEditPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage;
histrogramUpdate(handles, currentEditedImage);



% --- Executes on button press in setHuePusshbutton.
function setHuePusshbutton_Callback(hObject, eventdata, handles)
% hObject    handle to setHuePusshbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global hueImage hue_min_val hue_max_val currentEditedImage ;
size2=size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(3)==3)
%     axes(handles.axesImage);
%     imshow(currentEditedImage);
h = uicontrol('Style','text','String','Select the Mapping Region','Position',[220 120 200 40]);

     axes(handles.histrogramAreaAxes);
    imshow(hueImage);
    croppedImage = imcrop(hueImage);
    
    sze = size(croppedImage);
    hue_min_val = croppedImage(1,1,1);
    hue_max_val =croppedImage(1,sze(2),1);
    
    ImageHSV = rgb2hsv(currentEditedImage);
    ImageHSV1(:,:,1)=hue_min_val + (ImageHSV(:,:,1))*(hue_max_val-hue_min_val);
    ImageHSV1(:,:,2)=ImageHSV(:,:,2);
    ImageHSV1(:,:,3)=ImageHSV(:,:,3);
    
   ImageHSV1=hsv2rgb(ImageHSV1);
    axes(handles.axesImage);
    imshow(ImageHSV1);
     delete(h);
     currentEditedImage = ImageHSV1;
         histrogramUpdate(handles, currentEditedImage);

else
    msgbox(sprintf('Input color Image'),'Error','Error');
end

% --- Executes on button press in setSatPushbutton.
function setSatPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to setSatPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global satImage sat_min_val sat_max_val currentEditedImage ;
size2=size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(3)==3)
    h = uicontrol('Style','text','String','Select the Mapping Region','Position',[220 120 200 40]);

     axes(handles.histrogramAreaAxes);
    imshow(currentEditedImage);
    croppedImage = imcrop(satImage);
    
    sze = size(croppedImage);
    sat_min_val = croppedImage(1,1,2);
    sat_max_val =croppedImage(1,sze(2),2);
    
    ImageHSV = rgb2hsv(currentEditedImage);
    ImageHSV1(:,:,2)=sat_min_val + (ImageHSV(:,:,2))*(sat_max_val-sat_min_val);
    ImageHSV1(:,:,1)=ImageHSV(:,:,1);
    ImageHSV1(:,:,3)=ImageHSV(:,:,3);
    
   ImageHSV1=hsv2rgb(ImageHSV1);
    axes(handles.axesImage);
    imshow(ImageHSV1);
    delete(h);
    currentEditedImage = ImageHSV1;
    histrogramUpdate(handles, currentEditedImage);


else
    msgbox(sprintf('Input color Image'),'Error','Error');
end




% --- Executes on button press in equalizationPushbutton.
function equalizationPushbutton_Callback(hObject, eventdata, handles)
global currentEditedImage;
% Im3=rgb2gray(currentEditedImage);
% Im3=histeq(Im3,64);
% currentEditedImage = Im3;
% axes(handles.axesImage);
% imshow(currentEditedImage);
size2=size(currentEditedImage);
size2(3)=1;
xq=0:1/255:1;
axes(handles.histrogramAreaAxes);
x1=[0;1];
y1=[0;1];
vq1 = interp1(x1,y1,xq,'linear');
plot (xq,vq1);
for i=1:2
    [x y]=ginput(1);
    x1=[x1;x];
    y1=[y1;y];
    vq1 = interp1(x1,y1,xq,'splaine');
    equalizedImage=zeros(size2(1),size2(2));
    plot (xq,vq1);
    vq1=uint8(vq1*256);
    grayImage=(uint8(currentEditedImage*255));
    
    for i=1:size2(1)
        for j=1:size2(2)
            for k=1:3
                equalizedImage(i,j,k)=vq1(1,grayImage(i,j,k)+1);
            end
        end
    end
    equalizedImage=equalizedImage/255;
    axes(handles.axesImage);
    imshow(equalizedImage);
    
end
currentEditedImage=equalizedImage;

function Lab_to_sRGB_Callback(hObject, eventdata, handles)
% hObject    handle to Lab_to_sRGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Lab_to_sRGB
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.Lab_to_sRGB,'Value',1);
    try
        hcsc.Conversion = 'L*a*b* to sRGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
        
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
    end
    
    
else
    
end


% --- Executes on button press in sRGB_to_Lab.
function sRGB_to_Lab_Callback(hObject, eventdata, handles)
% hObject    handle to sRGB_to_Lab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sRGB_to_Lab
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.sRGB_to_Lab,'Value',1);
    try
        hcsc.Conversion = 'sRGB to L*a*b*';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


% --- Executes on button press in sRGB_to_XYZ.
function sRGB_to_XYZ_Callback(hObject, eventdata, handles)
% hObject    handle to sRGB_to_XYZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sRGB_to_XYZ
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.sRGB_to_XYZ,'Value',1);
    try
        hcsc.Conversion = 'sRGB to XYZ';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


% --- Executes on button press in XYZ_to_sRGB.
function XYZ_to_sRGB_Callback(hObject, eventdata, handles)
% hObject    handle to XYZ_to_sRGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of XYZ_to_sRGB
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.XYZ_to_sRGB,'Value',1);
    try
        hcsc.Conversion = 'XYZ to sRGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


% --- Executes on button press in Default.
function Default_Callback(hObject, eventdata, handles)
% hObject    handle to Default (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Default
global originalImage;
axes(handles.axesImage);
imshow(originalImage);
clearPushButton(handles);
set(handles.Default,'Value',1);
histrogramUpdate(handles, originalImage);


% --- Executes on button press in RGB_to_intensity.
function RGB_to_intensity_Callback(hObject, eventdata, handles)
% hObject    handle to RGB_to_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RGB_to_intensity
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_intensity,'Value',1);
    
    try
        hcsc.Conversion = 'RGB to intensity';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
%         histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end



% --- Executes on button press in YCbCr_to_RGB.
function YCbCr_to_RGB_Callback(hObject, eventdata, handles)
% hObject    handle to YCbCr_to_RGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of YCbCr_to_RGB
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.YCbCr_to_RGB,'Value',1);
    try
        hcsc.Conversion = 'YCbCr to RGB]';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


% --- Executes on button press in RGB_to_YCbCr.
function RGB_to_YCbCr_Callback(hObject, eventdata, handles)
% hObject    handle to RGB_to_YCbCr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RGB_to_YCbCr
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_YCbCr,'Value',1);
    try
        hcsc.Conversion = 'RGB to YCbCr';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


% --- Executes on button press in RGB_to_HSV.
function RGB_to_HSV_Callback(hObject, eventdata, handles)
% hObject    handle to RGB_to_HSV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RGB_to_HSV
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;
if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_HSV,'Value',1);
    try
        hcsc.Conversion = 'RGB to HSV';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    axes(handles.axesImage);
    imshow(currentEditedImage);
    
end


% --- Executes on button press in HSV_to_RGB.
function HSV_to_RGB_Callback(hObject, eventdata, handles)
% hObject    handle to HSV_to_RGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of HSV_to_RGB
global currentEditedImage;
global originalImage;
i1 = originalImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.HSV_to_RGB,'Value',1);
    try
        hcsc.Conversion = 'HSV to RGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
        currentEditedImage=i2;
        histrogramUpdate(handles, currentEditedImage);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


%-----------------metadataPushbutton calculation-------------------------------------

% --- Executes on button press in metadataPushbutton.
function metadataPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to metadataPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global path;
imageinfo(path);
% hObject    handle to equalizationPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on slider movement.
function shadowRecoSlider_Callback(hObject, eventdata, handles)
% hObject    handle to shadowRecoSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% hObject    handle to ShadowHighlightRecovery (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Compute normalized log of HDR data
coef  = get(hObject,'Value');
global currentEditedImage;
size2=size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(3)==3)
    red = currentEditedImage (:,:,1);
    green = currentEditedImage (:,:,2);
    blue = currentEditedImage (:,:,3);
    
    Rthreshold = graythresh(red);
    Gthreshold = graythresh(green);
    Bthreshold = graythresh(blue);
    
    bred = im2bw(red, Rthreshold*coef);
    bgreen = im2bw(green, Gthreshold*coef);
    bblue = im2bw(blue, Bthreshold*coef);
    
    bred = bwareaopen(bred,250);
    bgreen = bwareaopen(bgreen,250);
    bblue = bwareaopen(bblue,250);
    
    SE = strel('disk',10);
    redScale = imclose(bred,SE);
    greenScale = imclose(bgreen,SE);
    blueScale = imclose(bblue,SE);
    
    ScaleImage(:,:,1)=redScale;
    ScaleImage(:,:,2)=greenScale;
    ScaleImage(:,:,3)=blueScale;
    ScaleImage=ScaleImage+0.3;
    
    FinalImage= currentEditedImage .*ScaleImage;
    
    axes(handles.axesImage);
    imshow(FinalImage);
else
    red = currentEditedImage (:,:,1);
    
    Rthreshold = graythresh(red);
    
    
    bred = im2bw(red, Rthreshold*coef);
    
    bred = bwareaopen(bred,250);
    
    
    SE = strel('disk',10);
    redScale = imclose(bred,SE);
    
    
    ScaleImage(:,:,1)=redScale;
    
    ScaleImage=ScaleImage+0.3;
    
    FinalImage= currentEditedImage.*ScaleImage;
    
    axes(handles.axesImage);
    imshow(FinalImage);
    
    
end
 histrogramUpdate(handles, FinalImage);

% --- Executes during object creation, after setting all properties.
function shadowRecoSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shadowRecoSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on slider movement.
function gaussianSlider_Callback(hObject, eventdata, handles)
% hObject    handle to gaussianSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered = currentEditedImage;
if(val>0)
    filtered1 = currentEditedImage(:,:,1);
    sigma =(val)+1;
    ker = ceil(3*sigma);
    Gauss =fspecial('gaussian',[ker ker],sigma);
    filtered1 =imfilter(filtered1,Gauss,'same');
    
    
    filtered(:,:,1) =filtered1;
    
    if(sze==3)
        filtered2 = currentEditedImage(:,:,2);
        filtered3 = currentEditedImage(:,:,3);
        
        filtered2 =imfilter(filtered2,Gauss,'same');
        filtered3 =imfilter(filtered3,Gauss,'same');
        
        filtered(:,:,2) =filtered2;
        filtered(:,:,3) =filtered3;
    end
end
axes(handles.axesImage);
imshow(filtered)
if(sze==3)
    histrogramUpdate(handles, filtered);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function gaussianSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gaussianSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function wienerSlider_Callback(hObject, eventdata, handles)
% hObject    handle to wienerSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  originalImage sze;

Kernel_Size = ceil(get(hObject,'Value')*10+1);
filtered = originalImage;
if(Kernel_Size>1)
    filtered1 = wiener2(originalImage(:,:,1),[Kernel_Size Kernel_Size]);
    if(sze==1)
        filtered =filtered1;
    end
    if(sze==3)
        filtered2 = wiener2(originalImage(:,:,2),[Kernel_Size Kernel_Size]);
        filtered3 = wiener2(originalImage(:,:,3),[Kernel_Size Kernel_Size]);
        filtered =cat(3,filtered1,filtered2,filtered3);
    end
    
end
axes(handles.axesImage);
imshow(filtered);
if(sze==3)
    histrogramUpdate(handles, filtered);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function wienerSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wienerSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function medianSlider_Callback(hObject, eventdata, handles)
% hObject    handle to medianSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered = currentEditedImage;
if(val>0)
    filtered1 = currentEditedImage(:,:,1);
    kernel_size =int64(val*10)+1
    filtered1 =medfilt2(filtered1,[kernel_size kernel_size]);
    
    
    filtered =filtered1;
    if(sze==3)
        filtered2 = currentEditedImage(:,:,2);
        filtered3 = currentEditedImage(:,:,3);
        
        filtered2 =medfilt2(filtered2,[kernel_size kernel_size]);
        filtered3 =medfilt2(filtered3,[kernel_size kernel_size]);
        
        filtered =cat(3,filtered1,filtered2,filtered3);
    end
end
axes(handles.axesImage);
imshow(filtered)
if(sze==3)
    histrogramUpdate(handles, filtered);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function medianSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to medianSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function fuzzySlider_Callback(hObject, eventdata, handles)
% hObject    handle to fuzzySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sze fuzzyImage
h = ceil(get(hObject,'Value')*10);%user input
filtered  = zeros(size(fuzzyImage));
if(h>0)
    condition = mod(h,2);%check whether kernel size is even
    if(condition==0)
        h=h+1;
    end
    
    
    for num=1:1:sze
        pad=(h-1)/2 ;   % padding value for window: f=1; f=2; f=3; f=4; f=5;
        
        block = zeros(h*h,1);
        Image = double(padarray(fuzzyImage(:,:,num),[pad pad],'symmetric'));
        [m n ~] = size(fuzzyImage(:,:,num));
        for i=1+pad:1:m-pad
            for j=1+pad:1:n-pad
                x=reshape(Image(i-pad:i+pad, j-pad:j+pad),[],1);
                block_min = min(x);
                block_avg = mean(x);
                block_max = max(x);
                
                block(:,:) = 0;
                if (block_avg-block_min==0)||(block_max-block_avg==0)
                    block(:,:) = 1;
                else
                    ind1 = find((x>=block_min)&(x<=block_avg));
                    block(ind1) = 1-(block_avg-x(ind1))/(block_avg-block_min);
                    
                    ind2 = find((x>=block_avg)&(x<=block_max));
                    block(ind2) = 1-(x(ind2)-block_avg)/(block_max-block_avg);
                end
                
                filtered(i-pad,j-pad,num) = sum(sum(block.*x))/sum(sum(block));
                clear xmax xmin xmav ind1 ind2;
            end
        end
        
    end
    filtered = filtered(1:m-2*pad,1:n-2*pad,:);
end
filtered =uint8(filtered);
pad
axes(handles.axesImage);
imshow(filtered)
if(sze==3)
    histrogramUpdate(handles, filtered);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function fuzzySlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fuzzySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function colourNoiseSlider_Callback(hObject, eventdata, handles)
% hObject    handle to colourNoiseSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global originalImage sze  colour_slide luminance_slide;
axes(handles.axesImage);
imshow(originalImage)
colour_slide = get(hObject,'Value');

    if(sze==3)
        YCrCb_Image = rgb2ycbcr(originalImage);
        
        kernel_Size =ceil(colour_slide *30);
        condition = mod(kernel_Size,2);%check whether kernel size is even
        if(condition==0)
        kernel_Size=kernel_Size+1;
        end
          
        sigma =(luminance_slide)+1;
       
        ker_size =ceil(3*sigma);
        condition =mod(ker_size,2);
        if(condition==0)
            ker_size=ker_size+1;
        end
        filtered = YCrCb_Image;
        ker =max(kernel_Size,ker_size);
        pad=(ker-1)/2    % padding value for window
        
        filtered1 = padarray(YCrCb_Image(:,:,1),[pad pad],'symmetric');
        filtered2 = padarray(YCrCb_Image(:,:,2),[pad pad],'symmetric');
        filtered3 = padarray(YCrCb_Image(:,:,3),[pad pad],'symmetric');
        [row col] =size(filtered1);
        if(luminance_slide>0)
       
        Gauss =fspecial('gaussian',[ker_size ker_size],sigma);
        filtered1 =imfilter(filtered1,Gauss,'same');
       
        end
        
        if(colour_slide>0)
        
        filtered2 =medfilt2(filtered2,[kernel_Size kernel_Size]); 
        
      
        filtered3 =medfilt2(filtered3,[kernel_Size kernel_Size]);
        
        end
        filtered1 = filtered1(1+pad:row-pad,1+pad:col-pad);
        filtered2 = filtered2(1+pad:row-pad,1+pad:col-pad);
        filtered3 = filtered3(1+pad:row-pad,1+pad:col-pad);
        filtered=cat(3,filtered1,filtered2,filtered3);
        final = ycbcr2rgb(filtered);
        axes(handles.axesImage);
        imshow(final)
        histrogramUpdate(handles, final);
    end

if (sze<3)
    msgbox(sprintf('This function is applicable only for coloured images'),'Error','Error');
    return
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function colourNoiseSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to colourNoiseSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function luminanaceNoiseSlider_Callback(hObject, eventdata, handles)
% hObject    handle to luminanaceNoiseSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global originalImage sze  colour_slide luminance_slide;
axes(handles.axesImage);
imshow(originalImage)
luminance_slide = get(hObject,'Value')*10;

    if(sze==3)
        YCrCb_Image = rgb2ycbcr(originalImage);
        
        kernel_Size =ceil(colour_slide*10)+1;
        sigma =(luminance_slide)+1;
        ker_size =ceil(3*sigma);
        condition =mod(kernel_Size,2);
        if(condition==0)
            kernel_Size = kernel_Size+1;
        end
        condition = mod(ker_size,2);
        if(condition==0)
            ker_size=ker_size+1;
        end
        
        filtered = YCrCb_Image;
        ker =max(kernel_Size,ker_size);
      
        pad=(ker-1)/2   ; % padding value for window
        filtered1 = padarray(YCrCb_Image(:,:,1),[pad pad],'symmetric');
        filtered2 = padarray(YCrCb_Image(:,:,2),[pad pad],'symmetric');
        filtered3 = padarray(YCrCb_Image(:,:,3),[pad pad],'symmetric');
        [row col] =size(filtered1);
        if(luminance_slide>0)
       
        Gauss =fspecial('gaussian',[ker_size ker_size],sigma);
        filtered1 =imfilter(filtered1,Gauss,'same');
       
        end
        
        if(colour_slide>0)
        
        filtered2 =medfilt2(filtered2,[kernel_Size kernel_Size]); 
        
      
        filtered3 =medfilt2(filtered3,[kernel_Size kernel_Size]);
        
        end
        filtered1 = filtered1(1+pad:row-pad,1+pad:col-pad);
        filtered2 = filtered2(1+pad:row-pad,1+pad:col-pad);
        filtered3 = filtered3(1+pad:row-pad,1+pad:col-pad);
        filtered=cat(3,filtered1,filtered2,filtered3);
        final = ycbcr2rgb(filtered);
        axes(handles.axesImage);
        imshow(final)
       
        
        histrogramUpdate(handles, final);
        
    end

if (sze<3)
    msgbox(sprintf('This function is applicable only for coloured images'),'Error','Error');
    return
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function luminanaceNoiseSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to luminanaceNoiseSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function blurRemovalSlider_Callback(hObject, eventdata, handles)
% hObject    handle to blurRemovalSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
estimated_noise  = get(hObject,'Value')/50;

LEN = 21;
THETA = 11;
filter = fspecial('motion', LEN, THETA);

weiner = deconvwnr(currentEditedImage, filter, estimated_noise);
axes(handles.axesImage);
imshow(weiner)
if(sze==3)
    histrogramUpdate(handles, weiner);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function blurRemovalSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blurRemovalSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sharpenSlider_Callback(hObject, eventdata, handles)
% hObject    handle to sharpenSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
sharpen = get(hObject,'Value');
filtered = currentEditedImage;
if(sharpen>0)
    filtered1 = currentEditedImage(:,:,1);
    sigma =(sharpen)+1;
    
    ker_size = ceil(3*sigma);
    
    Gauss =fspecial('gaussian',[ker_size ker_size],sigma);
    filtered1 =imfilter(filtered1,Gauss,'same');
    filtered1 = (1+sharpen)*(currentEditedImage(:,:,1))- sharpen*(filtered1);
    
    
    filtered(:,:,1) =filtered1;
    if(sze==3)
        filtered2 = currentEditedImage(:,:,2);
        filtered3 = currentEditedImage(:,:,3);
        
        filtered2 =imfilter(filtered2,Gauss,'same');
        filtered3 =imfilter(filtered3,Gauss,'same');
        
        filtered2 = (1+sharpen)*(currentEditedImage(:,:,2))- sharpen*(filtered2);
        filtered3 = (1+sharpen)*(currentEditedImage(:,:,3))- sharpen*(filtered3);
        
        filtered(:,:,2) =filtered2;
        filtered(:,:,3) =filtered3;
    end
    axes(handles.axesImage);
end
imshow(filtered)
if(sze==3)
    histrogramUpdate(handles, filtered);
end
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function sharpenSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sharpenSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on slider movement.
function slider46_Callback(hObject, eventdata, handles)
% hObject    handle to slider46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global currentEditedImage sze;

if(sze==3)
 set_temp = get(hObject,'Value')/500;
 filtered=currentEditedImage;
 filtered(:,:,1)=currentEditedImage(:,:,1)+set_temp;
 filtered(:,:,3)=currentEditedImage(:,:,3)-set_temp;
 
 axes(handles.axesImage);
 imshow(filtered)
 
 
 histrogramUpdate(handles, filtered);
 end

% --- Executes during object creation, after setting all properties.
function slider46_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function cropPushbutton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cropPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on mouse press over axes background.
function axesImage_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axesImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;

axes(handles.axesImage);
[x,y]=ginput(1);
RGB=currentEditedImage(floor(y),floor(x),:);
r=RGB(1);
g=RGB(2);
b=RGB(3);
m=max([r,g,b]);
r=r/m;
g=g/m;
b=b/m;

filtered = currentEditedImage;
%%%%%%Diagonal matrix multiplication%%%%%%
filtered(:,:,1)=filtered(:,:,1)/r;
filtered(:,:,2)=filtered(:,:,2)/g;
filtered(:,:,3)=filtered(:,:,3)/b;

imshow(filtered)
histrogramUpdate(handles, filtered);


% --- Executes on button press in autoAdjust.
function autoAdjust_Callback(hObject, eventdata, handles)
% hObject    handle to autoAdjust (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage;
currentEditedImage=auto_tone_adjust(currentEditedImage);
axes(handles.axesImage);
imshow(currentEditedImage);
histrogramUpdate(handles, currentEditedImage);





