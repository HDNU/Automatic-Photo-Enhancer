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
%  I = imread('image1.jpg');
%  axes(handles.axesImage);
%  imshow(I);
global path;
[path, Cancel] = imgetfile();
global Im1 duplicate;
if Cancel
    msgbox(sprintf('Error'),'Error','Error');
    return
end
Im1 = imread(path);
duplicate =Im1;
Im1 = im2double(Im1);

axes(handles.axesImage)
imshow(Im1)
 
 function pushbutton2_Callback(hObject, eventdata, handles)
 global Im1;
Im2 =Im1;
Im2(:,:,2)=0;
Im2(:,:,3)=0;

imshow(Im2);
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function pushbutton3_Callback(hObject, eventdata, handles)
global Im1;
Im3=rgb2gray(Im1);
Im3=histeq(Im3,64);
axes(handles.axesImage);
imshow(Im3);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global Im1;
Im4=rgb2gray(Im1);
Im4=imsharpen(Im4);
axes(handles.axesImage);
imshow(Im4);
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% h = uicontrol('Style','text','String','Hello world','Position',[200 420 100 20]);
global Im1;
% MultiSlider

    %Split into RGB Channels
    Red = Im1(:,:,1);
    Green =Im1(:,:,2);
    Blue = Im1(:,:,3);

    %Get histValues for each channel
    [yRed, x] = imhist(Red);
    [yGreen, x] = imhist(Green);
    [yBlue, x] = imhist(Blue);

    %Plot them together in one plot
    plot(x, yRed, 'Red', x, yGreen, 'Green', x, yBlue, 'Blue');
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
a = get(hObject,'Value');
filename=handles.filename;
Im1 =imread(filename);
Im2=rgb2gray(Im1);
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



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
%string1 = sprintf('Dynamic');

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






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


% --- Executes on button press in radiobutton2.

% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Im1;

img1 = Im1(:,:,1);
img2 = Im1(:,:,2);
img3 = Im1(:,:,3);

img1 =imadjust(img1);
img1 = histeq(img1);
img1 = adapthisteq(img1);


img2 =imadjust(img2);
img2 = histeq(img2);
img2 = adapthisteq(img2);

img3 =imadjust(img3);
img3 = histeq(img3);
img3 = adapthisteq(img3);

img4 =Im1;
img4(:,:,1) =img1;
img4(:,:,2) =img2;
img4(:,:,3) =img3;
axes(handles.axesImage);
imshow(img4);





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
global Im1;
val = get(hObject,'Value');
filtered1 = Im1(:,:,1);
filtered2 = Im1(:,:,2);
filtered3 = Im1(:,:,3);
sigma =(val)+1;
Gauss =fspecial('gaussian',[5 5],sigma);
filtered1 =imfilter(filtered1,Gauss,'same');
filtered2 =imfilter(filtered2,Gauss,'same');
filtered3 =imfilter(filtered3,Gauss,'same');
filtered = Im1;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
axes(handles.axesImage);
imshow(filtered)
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
global Im1;
val = get(hObject,'Value');
filtered1 = Im1(:,:,1);
filtered2 = Im1(:,:,2);
filtered3 = Im1(:,:,3);
num =int64(val*10)+1;
filtered1 =medfilt2(filtered1,[num num]);
filtered2 =medfilt2(filtered2,[num num]);
filtered3 =medfilt2(filtered3,[num num]);
filtered = Im1;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
axes(handles.axesImage);
imshow(filtered)
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
global Im1;
val = get(hObject,'Value');
filtered1 = Im1(:,:,1);
filtered2 = Im1(:,:,2);
filtered3 = Im1(:,:,3);
sigma =(val)+1;
Gauss =fspecial('gaussian',[5 5],sigma);
filtered1 =imfilter(filtered1,Gauss,'same');
filtered2 =imfilter(filtered2,Gauss,'same');
filtered3 =imfilter(filtered3,Gauss,'same');
filtered1 = (1+val)*(Im1(:,:,1))- val*(filtered1);
filtered2 = (1+val)*(Im1(:,:,2))- val*(filtered2);
filtered3 = (1+val)*(Im1(:,:,3))- val*(filtered3);
filtered = Im1;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
axes(handles.axesImage);
imshow(filtered)
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
global Im1;
val = get(hObject,'Value');
filtered1 = Im1(:,:,1);
filtered2 = Im1(:,:,2);
filtered3 = Im1(:,:,3);
gamma = 0.9 + val/5;
c = 1/(1.0^gamma);

filtered1 =255.0*c*((filtered1/255.0).^gamma);
filtered2 =255.0*c*((filtered2/255.0).^gamma);
filtered3 =255.0*c*((filtered3/255.0).^gamma);

filtered = Im1;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
axes(handles.axesImage);
imshow(filtered)
axes(handles.axes2);
imhist(filtered(:,:,1));
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
global  duplicate;
C = (get(hObject,'Value')*510)-255;

filtered1 = duplicate(:,:,1);
filtered2 = duplicate(:,:,2);
filtered3 = duplicate(:,:,3);

F =(259*(C+255))/(255*(259-C));

filtered1 =uint8(F*(filtered1-128)+128);
filtered2 =uint8(F*(filtered2-128)+128);
filtered3 =uint8(F*(filtered3-128)+128);

filtered = duplicate;
filtered(:,:,1) =filtered1;
filtered(:,:,2) =filtered2;
filtered(:,:,3) =filtered3;
axes(handles.axesImage);
imshow(filtered)
axes(handles.axes2);
imhist(filtered(:,:,1));
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


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global duplicate
axes(handles.axesImage);
imshow(duplicate);
crop = imcrop(duplicate);
axes(handles.axes2);
imshow(crop);
set(handles.text10, 'String', 'Cropped Image');


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
% 
% if Cancel
%     msgbox(sprintf('Error'),'Error','Error');
%     return
% end
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
global Im1 vignetteAmount vignetteMidpoint;
vignetteAmount = get(hObject,'Value');
[nr, nc, nChannels]=size(Im1);
cx=ceil(nc/2);
cy=ceil(nr/2);

radius = vignetteMidpoint*max(nr,nc);
maxDistance = sqrt(cx.^2 + cy.^2)-radius;

vignetteImage=Im1;
for k = 1:nChannels
    for i=1:nr
        for j=1:nc
            distanceFromCenter = sqrt((abs(i-cy).^2)+(abs(j-cx).^2));
            if(distanceFromCenter>radius)
                scale= abs(vignetteAmount*(distanceFromCenter-radius)/maxDistance);
                if (vignetteAmount<0)
                    vignetteImage(i,j,k)= Im1(i,j,k)*(1-scale)-50*scale;
                else
                    vignetteImage(i,j,k)= Im1(i,j,k)/(1-scale)+ 50*scale;
                end
            end
        end
    end
end

axes(handles.axesImage);
imshow(vignetteImage);


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
global Im1 vignetteAmount vignetteMidpoint;
vignetteMidpoint = get(hObject,'Value');
[nr, nc, nChannels]=size(Im1);
cx=ceil(nc/2);
cy=ceil(nr/2);

radius = vignetteMidpoint*max(nr,nc);
maxDistance = sqrt(cx.^2 + cy.^2)-radius;

vignetteImage=Im1;
for k = 1:nChannels
    for i=1:nr
        for j=1:nc
            distanceFromCenter = sqrt((abs(i-cy).^2)+(abs(j-cx).^2));
            if(distanceFromCenter>radius)
                scale= abs(vignetteAmount*(distanceFromCenter-radius)/maxDistance);
                if (vignetteAmount<0)
                    vignetteImage(i,j,k)= Im1(i,j,k)*(1-scale)-50*scale;
                else
                    vignetteImage(i,j,k)= Im1(i,j,k)/(1-scale)+ 50*scale;
                end
            end
        end
    end
end

axes(handles.axesImage);
imshow(vignetteImage);


% --- Executes during object creation, after setting all properties.
function vignetteMidpointSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vignetteMidpointSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
