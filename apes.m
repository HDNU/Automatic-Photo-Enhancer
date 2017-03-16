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
 global vignetteAmount vignetteMidpoint;
 vignetteAmount = 0.5;
 vignetteMidpoint = 0.5;

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


% --- Executes on button press in selectPicture.
function selectPicture_Callback(hObject, eventdata, handles)
% hObject    handle to selectPicture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global path sze;
[path, Cancel] = imgetfile();
global currentEditedImage originalImage hueImage satImage;
if Cancel
    msgbox(sprintf('Error'),'Error','Error');
    return
end
currentEditedImage = imread(path);
originalImage = currentEditedImage;
currentEditedImage = im2double(currentEditedImage);
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
end






% --- Executes on button press in Histograms.
function Histograms_Callback(hObject, eventdata, handles)
% h = uicontrol('Style','text','String','Hello world','Position',[200 420 100 20]);
global currentEditedImage;
% MultiSlider

    %Split into RGB Channels
    Red = currentEditedImage(:,:,1);
    Green =currentEditedImage(:,:,2);
    Blue = currentEditedImage(:,:,3);

    %Get histValues for each channel
    [yRed, x] = imhist(Red);
    [yGreen, x] = imhist(Green);
    [yBlue, x] = imhist(Blue);

    %Plot them together in one plot
    plot(x, yRed, 'Red', x, yGreen, 'Green', x, yBlue, 'Blue');
% hObject    handle to Histograms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% img reading again???
a = get(hObject,'Value');
filename=handles.filename;
currentEditedImage =imread(filename);
Im2=rgb2gray(currentEditedImage);
Im2=Im2*a;
% ImMax=double(max(max(Im2)));
% ImMin=double(min(min(Im2)));
% Im2 = ((Im2-ImMin)/(ImMax-ImMin))*a*255;
axes(handles.axesImage);
imshow(Im2);
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end








% --------------------------------------------------------------------

% hObject    handle to uitoggletool2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on slider1 and none of its controls.

% hObject    handle to slider1 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)








% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%h = uicontrol('Style','text','String','','Position',[200 20 200 420]);

global path;
a=imfinfo(path);
a=struct2cell(a);
g=size(a);
g(1,1);
d='';
Datapack={'Filename','FileModDate','FileSize','FileFormat','version','Width','Height','Depth','ColorType'};
for k=1:8
word = a(k);
word = word{1};

if(isnumeric(word))
    word = int2str(word);
end
qe=strcat(Datapack(k),':',word);
d=[d  char(10)'  qe];
end

h = uicontrol('Style','text','String',d,'Position',[650 10 200 470]);
handles.h=h;
guidata(hObject,handles);
handles.jil=1;
guidata(hObject,handles);


% --- Executes on button press in sRGB_to_Lab.

% hObject    handle to sRGB_to_Lab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sRGB_to_Lab


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;

img1 = currentEditedImage(:,:,1);
img1 =imadjust(img1);
img1 = histeq(img1);
img1 = adapthisteq(img1);

img4 =currentEditedImage;
img4(:,:,1) =img1;

if(sze==3)
img2 = currentEditedImage(:,:,2);
img3 = currentEditedImage(:,:,3);

img2 =imadjust(img2);
img2 = histeq(img2);
img2 = adapthisteq(img2);

img3 =imadjust(img3);
img3 = histeq(img3);
img3 = adapthisteq(img3);

img4(:,:,2) =img2;
img4(:,:,3) =img3;

end
currentEditedImage = img4;
axes(handles.axesImage);
imshow(currentEditedImage);




% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered1 = currentEditedImage(:,:,1);
sigma =(val)+1;
Gauss =fspecial('gaussian',[5 5],sigma);
filtered1 =imfilter(filtered1,Gauss,'same');

filtered = currentEditedImage;
filtered(:,:,1) =filtered1;

if(sze==3)
filtered2 = currentEditedImage(:,:,2);
filtered3 = currentEditedImage(:,:,3);

filtered2 =imfilter(filtered2,Gauss,'same');
filtered3 =imfilter(filtered3,Gauss,'same');

filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
end
currentEditedImage = filtered;
axes(handles.axesImage);
imshow(currentEditedImage);

axes(handles.axes2);
imhist(filtered(:,:,1));
set(handles.text10, 'String', 'Histogram');
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered1 = currentEditedImage(:,:,1);
num =int64(val*10)+1;
filtered1 =medfilt2(filtered1,[num num]);

filtered = currentEditedImage;
filtered(:,:,1) =filtered1;
if(sze==3)
filtered2 = currentEditedImage(:,:,2);
filtered3 = currentEditedImage(:,:,3);

filtered2 =medfilt2(filtered2,[num num]);
filtered3 =medfilt2(filtered3,[num num]);

filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
end
currentEditedImage = filtered;
axes(handles.axesImage);
imshow(currentEditedImage);

axes(handles.axes2);
imhist(filtered(:,:,1));
set(handles.text10, 'String', 'Histogram');
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered1 = currentEditedImage(:,:,1);
sigma =(val)+1;
Gauss =fspecial('gaussian',[5 5],sigma);
filtered1 =imfilter(filtered1,Gauss,'same');
filtered1 = (1+val)*(currentEditedImage(:,:,1))- val*(filtered1);

filtered = currentEditedImage;
filtered(:,:,1) =filtered1;
if(sze==3)
filtered2 = currentEditedImage(:,:,2);
filtered3 = currentEditedImage(:,:,3);

filtered2 =imfilter(filtered2,Gauss,'same');
filtered3 =imfilter(filtered3,Gauss,'same');

filtered2 = (1+val)*(currentEditedImage(:,:,2))- val*(filtered2);
filtered3 = (1+val)*(currentEditedImage(:,:,3))- val*(filtered3);

filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
end
currentEditedImage = filtered;
axes(handles.axesImage);
imshow(currentEditedImage);

axes(handles.axes2);
imhist(filtered(:,:,1));
set(handles.text10, 'String', 'Histogram');
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider12_Callback(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage sze;
val = get(hObject,'Value');
filtered1 = currentEditedImage(:,:,1);
gamma = 0.9 + val/5;

c = 1/(1.0^gamma);

filtered1 =255.0*c*((filtered1/255.0).^gamma);

filtered = currentEditedImage;
filtered(:,:,1) =filtered1;
if(sze==3)
filtered2 = currentEditedImage(:,:,2);
filtered3 = currentEditedImage(:,:,3);

filtered2 =255.0*c*((filtered2/255.0).^gamma);
filtered3 =255.0*c*((filtered3/255.0).^gamma);


filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
end
currentEditedImage = filtered;
axes(handles.axesImage);
imshow(currentEditedImage);

axes(handles.axes2);
[yRed, x] = imhist(filtered(:,:,1));
[yGreen, x] = imhist(filtered(:,:,2));
[yBlue, x] = imhist(filtered(:,:,3));
%Plot them together in one plot
plot(x, yRed, 'Red', x, yGreen, 'Green', x, yBlue, 'Blue');
set(handles.text10, 'String', 'Histogram');
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  currentEditedImage sze;
C = (get(hObject,'Value')*256)-128;


filtered1 = currentEditedImage(:,:,1);
F =(259*(C+255))/(255*(259-C));

filtered1 =uint8(F*(filtered1-128)+128);
filtered = currentEditedImage;
filtered(:,:,1) =filtered1;
if(sze==3)
filtered2 = currentEditedImage(:,:,2);
filtered3 = currentEditedImage(:,:,3);


filtered2 =uint8(F*(filtered2-128)+128);
filtered3 =uint8(F*(filtered3-128)+128);


filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
end

currentEditedImage = filtered;
axes(handles.axesImage);
imshow(currentEditedImage);

axes(handles.axes2);
[yRed, x] = imhist(filtered(:,:,1));
[yGreen, x] = imhist(filtered(:,:,2));
[yBlue, x] = imhist(filtered(:,:,3));
%Plot them together in one plot
plot(x, yRed, 'Red', x, yGreen, 'Green', x, yBlue, 'Blue');
set(handles.text10, 'String', 'Histogram');
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in CropPushbutton.
function CropPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to CropPushbutton (see GCBO)
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
axes(handles.axes2);
imshow(currentEditedImage);



% --- Executes during object creation, after setting all properties.
function text8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function text10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
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
function selectPicture_CreateFcn(hObject, eventdata, handles)
% hObject    handle to selectPicture (see GCBO)
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
global currentEditedImage vignetteAmount vignetteMidpoint;
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
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)*(1-scale)-5*scale;
                else
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)/(1-scale)+ 5*scale;
                end
            end
        end
    end
end
currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(currentEditedImage);


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
global currentEditedImage vignetteAmount vignetteMidpoint;
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
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)*(1-scale)-5*scale;
                else
                    vignetteImage(i,j,k)= currentEditedImage(i,j,k)/(1-scale)+5*scale;
                end
            end
        end
    end
end
currentEditedImage = vignetteImage;
axes(handles.axesImage);
imshow(currentEditedImage);


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
function slider16_Callback(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentEditedImage;
estimated_noise  = get(hObject,'Value')/50;

LEN = 21;
THETA = 11;
PSF = fspecial('motion', LEN, THETA);

wnr3 = deconvwnr(currentEditedImage, PSF, estimated_noise);
currentEditedImage = wnr3;
axes(handles.axesImage);
imshow(currentEditedImage);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Rivert2OriginalPushbutton.
function Rivert2OriginalPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Rivert2OriginalPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global originalImage currentEditedImage;
currentEditedImage = originalImage;
axes(handles.axesImage);
imshow(currentEditedImage);


% --- Executes on button press in UndoLastEditPushbutton.
function UndoLastEditPushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to UndoLastEditPushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.



% --- Executes on slider movement.


% --- Executes on button press in SetHue.
function SetHue_Callback(hObject, eventdata, handles)
% hObject    handle to SetHue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global hueImage hue_min_val hue_max_val currentEditedImage ;
size2=size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(3)==3)
    
axes(handles.Hue);
imshow(hueImage);
croppedImage = imcrop(hueImage);

sze = size(croppedImage);
hue_min_val = croppedImage(1,1,1);
hue_max_val =croppedImage(1,sze(2),1);

ImageHSV = rgb2hsv(currentEditedImage);
ImageHSV1(:,:,1)=hue_min_val + (ImageHSV(:,:,1))*(hue_max_val-hue_min_val);
ImageHSV1(:,:,2)=ImageHSV(:,:,2);
ImageHSV1(:,:,3)=ImageHSV(:,:,3);

axes(handles.axes2);
imshow(croppedImage);
axes(handles.axesImage);
imshow(ImageHSV1);
else
 msgbox(sprintf('Input color Image'),'Error','Error');
end

% --- Executes on button press in Sat.
function Sat_Callback(hObject, eventdata, handles)
% hObject    handle to Sat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global satImage sat_min_val sat_max_val currentEditedImage ;
size2=size(currentEditedImage);
if (length(size2)==2)
    size2=[size2,1];
end
if (size2(3)==3)
axes(handles.Saturation);
imshow(satImage);
croppedImage = imcrop(satImage);

sze = size(croppedImage);
sat_min_val = croppedImage(1,1,2);
sat_max_val =croppedImage(1,sze(2),2);

ImageHSV = rgb2hsv(currentEditedImage);
ImageHSV1(:,:,2)=sat_min_val + (ImageHSV(:,:,2))*(sat_max_val-sat_min_val);
ImageHSV1(:,:,1)=ImageHSV(:,:,1);
ImageHSV1(:,:,3)=ImageHSV(:,:,3);

axes(handles.axes2);
imshow(croppedImage);
axes(handles.axesImage);
imshow(ImageHSV1);
else
 msgbox(sprintf('Input color Image'),'Error','Error');
end

% --- Executes on button press in ShadowHighlightRecovery.
function ShadowHighlightRecovery_Callback(hObject, eventdata, handles)
% hObject    handle to ShadowHighlightRecovery (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Compute normalized log of HDR data
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

bred = im2bw(red, Rthreshold/2);
bgreen = im2bw(green, Gthreshold/2);
bblue = im2bw(blue, Bthreshold/2);

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


bred = im2bw(red, Rthreshold/2);

bred = bwareaopen(bred,250);


SE = strel('disk',10);
redScale = imclose(bred,SE);


ScaleImage(:,:,1)=redScale;

ScaleImage=ScaleImage+0.3;

FinalImage= currentEditedImage.*ScaleImage;

axes(handles.axesImage);
imshow(FinalImage);
end


% --- Executes on button press in Histeq1.
function Histeq1_Callback(hObject, eventdata, handles)
global currentEditedImage;
Im3=rgb2gray(currentEditedImage);
Im3=histeq(Im3,64);
currentEditedImage = Im3;
axes(handles.axesImage);
imshow(currentEditedImage);


function Lab_to_sRGB_Callback(hObject, eventdata, handles)
% hObject    handle to Lab_to_sRGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Lab_to_sRGB
global currentEditedImage;
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.Lab_to_sRGB,'Value',1);
   
        hcsc.Conversion = 'L*a*b* to sRGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
    
        
        
    
else
    
end


% --- Executes on button press in sRGB_to_Lab.
function sRGB_to_Lab_Callback(hObject, eventdata, handles)
% hObject    handle to sRGB_to_Lab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sRGB_to_Lab
global currentEditedImage;
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.sRGB_to_Lab,'Value',1);
    try
        hcsc.Conversion = 'sRGB to L*a*b*';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.sRGB_to_XYZ,'Value',1);
    try
        hcsc.Conversion = 'sRGB to XYZ';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.XYZ_to_sRGB,'Value',1);
    try
        hcsc.Conversion = 'XYZ to sRGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
global currentEditedImage;
axes(handles.axesImage);
imshow(currentEditedImage);
clearPushButton(handles);
set(handles.Default,'Value',1);


% --- Executes on button press in RGB_to_intensity.
function RGB_to_intensity_Callback(hObject, eventdata, handles)
% hObject    handle to RGB_to_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RGB_to_intensity
global currentEditedImage;
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_intensity,'Value',1);
    
    try
        hcsc.Conversion = 'RGB to intensity';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.YCbCr_to_RGB,'Value',1);
    try
        hcsc.Conversion = 'YCbCr to RGB]';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_YCbCr,'Value',1);
    try
        hcsc.Conversion = 'RGB to YCbCr';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;
if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.RGB_to_HSV,'Value',1);
    try
        hcsc.Conversion = 'RGB to HSV';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
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
i1 = currentEditedImage;
hcsc = vision.ColorSpaceConverter;

if get(hObject,'Value')
    clearPushButton(handles);
    set(handles.HSV_to_RGB,'Value',1);
    try
        hcsc.Conversion = 'HSV to RGB';
        i2 = step(hcsc, i1);
        axes(handles.axesImage);
        imshow(i2);
    catch
        uiwait(msgbox('This conversion is not valid','Error'));
        
    end
else
    
end


%-----------------metaData calculation-------------------------------------

% --- Executes on button press in metaData.
function metaData_Callback(hObject, eventdata, handles)
% hObject    handle to metaData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global path;
imageinfo(path);
% hObject    handle to Histeq1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in metaData.
