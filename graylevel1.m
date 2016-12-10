function varargout = graylevel1(varargin)
% GRAYLEVEL1 MATLAB code for graylevel1.fig
%      GRAYLEVEL1, by itself, creates a new GRAYLEVEL1 or raises the existing
%      singleton*.
%
%      H = GRAYLEVEL1 returns the handle to a new GRAYLEVEL1 or the handle to
%      the existing singleton*.
%
%      GRAYLEVEL1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRAYLEVEL1.M with the given input arguments.
%
%      GRAYLEVEL1('Property','Value',...) creates a new GRAYLEVEL1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before graylevel1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to graylevel1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help graylevel1

% Last Modified by GUIDE v2.5 11-Jul-2016 20:51:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @graylevel1_OpeningFcn, ...
                   'gui_OutputFcn',  @graylevel1_OutputFcn, ...
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


% --- Executes just before graylevel1 is made visible.
function graylevel1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to graylevel1 (see VARARGIN)

% Choose default command line output for graylevel1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes graylevel1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = graylevel1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in threshold.
function threshold_Callback(hobject, eventdata, handles)
% hObject    handle to threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  val =get(handles.thresholdvalue,'String');
 set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
 F=rgb2gray(theImage); 
[rows cols]=size(F);

    for i=1:rows
        for j=1:cols
            if(F(i,j)< val)
            F(i,j)=0;
            else
            F(i,j)=255;
            end
        end
    end  
axes(handles.proce);
imshow(F);
% --- Executes on button press in contraststretching.
function contraststretching_Callback(hObject, eventdata, handles)
% hObject    handle to contraststretching (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
itemp =theImage; %read the image
i = itemp(:,:,1);
rtemp = min(i);         % find the min. value of pixels in all the columns (row vector)
rmin = min(rtemp);      % find the min. value of pixel in the image
rtemp = max(i);         % find the max. value of pixels in all the columns (row vector)
rmax = max(rtemp);      % find the max. value of pixel in the image
m = 255/(rmax - rmin);  % find the slope of line joining point (0,255) to (rmin,rmax)
c = 255 - m*rmax;       % find the intercept of the straight line with the axis
i_new = m*i + c;        % transform the image according to new slope
imshow(i_new);          % display transformed image




% --- Executes on button press in bitplaneslicing.
function bitplaneslicing_Callback(hObject, eventdata, handles)
% hObject    handle to bitplaneslicing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','off');
set(handles.text5,'String','');
axes(handles.org);
theImage = getimage();
it=theImage;   %read the image
itemp = it(:,:,1);
[r,c]= size(itemp);         % get the dimensions of image
s = zeros(r,c,8);           % pre allocate a variable to store 8 bit planes of the image
for k = 1:8
    for i = 1:r
        for j = 1:c
            s(i,j,k) = bitget(itemp(i,j),k);    %get kth bit from each pixel 
        end
     end
    
end
figure;                                          %display all the 8 bit planes
    subplot(3,3,1);imshow(s(:,:,8));title('8th(MSB)Plane');
    subplot(3,3,2);imshow(s(:,:,7));title('7th Plane');
    subplot(3,3,3);imshow(s(:,:,6));title('6th Plane');
    subplot(3,3,4);imshow(s(:,:,5));title('5th Plane');
    subplot(3,3,5);imshow(s(:,:,4));title('4th Plane');
    subplot(3,3,6);imshow(s(:,:,3));title('3th Plane');
    subplot(3,3,7);imshow(s(:,:,2));title('2nd Plane');
    subplot(3,3,8);imshow(s(:,:,1));title('1st(LSB)Plane')
    % reconstruct the original image from generated bit planes
    rec=s(:,:,1)+s(:,:,2)*2+s(:,:,3)*4+s(:,:,4)*8+s(:,:,5)*16+s(:,:,6)*32+s(:,:,7)*64+s(:,:,8)*128; 
    subplot(3,3,9);imshow(uint8(rec));title('Recovered Image')%display the reconstructed image



% --- Executes on button press in histogram.
function histogram_Callback(~, ~, handles)
% hObject    handle to histogram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
x=rgb2gray(theImage);
y=histeq(x);
imshow(y);



% --- Executes on button press in flipimage.
function flipimage_Callback(hObject, eventdata, handles)
% hObject    handle to flipimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
switch get(handles.checkbox,'String')
    case Horizontal
        I2 = flipdim(theImage ,2); 
        imshow(I2);
    case Vertical
        I3 = flipdim(theImage ,1); 
        imshow(I3);
        ...
    otherwise 
    I4 = flipdim(I3 ,2); 
    imshow(I4);
    set(handles.text5,'String','Flip horizontal+vertical');
end






function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thresholdvalue_Callback(hObject, eventdata, handles)
% hObject    handle to thresholdvalue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thresholdvalue as text
%        str2double(get(hObject,'String')) returns contents of thresholdvalue as a double


% --- Executes during object creation, after setting all properties.
function thresholdvalue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thresholdvalue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in negativeimage.
function negativeimage_Callback(hObject, eventdata, handles)
% hObject    handle to negativeimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
x=imcomplement(theImage);
axes(handles.proce);
imshow(x);

% --- Executes on button press in exit.
function exit_Callback(hObject, eventdata, handles)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all;

% --- Executes on button press in filterimage.
function filterimage_Callback(hObject, eventdata, handles)
% hObject    handle to filterimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=get(handles.filter,'Value');
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
s=rgb2gray(theImage);

switch x
    case 1
        h = fspecial('gaussian',10);
        lpf=imfilter(s,h);
        axes(handles.proce);
        imshow(lpf);
    case 2 
        [m n]=size(s);
        f_transform=fft2(s);
        f_shift=fftshift(f_transform);
        p=m/2;
        q=n/2;
        d0=70;
        for i=1:m
            for j=1:n
                distance=sqrt((i-p)^2+(j-q)^2);
                low_filter(i,j)=1-exp(-(distance)^2/(2*(d0^2)));
            end
        end
        filter_apply=f_shift.*low_filter(i,j);
        image_orignal=ifftshift(filter_apply);
        image_filter_apply=abs(ifft2(image_orignal));
        axes(handles.proce);
        imshow(image_filter_apply,[])
    case 3
        h = fspecial('average',3);
        mpf=imfilter(s,h);
        axes(handles.proce);
        imshow(mpf);
end


 
% --- Executes on selection change in detection.
function detection_Callback(hObject, eventdata, handles)
% hObject    handle to detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns detection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from detection


% --- Executes during object creation, after setting all properties.
function detection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in filter.
function filter_Callback(hObject, eventdata, handles)
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns filter contents as cell array
%        contents{get(hObject,'Value')} returns selected item from filter


% --- Executes during object creation, after setting all properties.
function filter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1


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
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in reset.
function reset_Callback(hobject,eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
imshow(theImage);



% --- Executes on button press in grayscale.
function grayscale_Callback(hobject,eventdata, handles)
% hObject    handle to grayscale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
I=rgb2gray(theImage);
axes(handles.proce);
imshow(I);


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in edgedetection.
function edgedetection_Callback(hObject, eventdata, handles)
% hObject    handle to edgedetection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selectedItemval =get(handles.detection,'Value');
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
f=theImage;
switch selectedItemval
    case 1
        g=rgb2gray(f);
        BW = edge(g,'Sobel');
        axes(handles.proce);
        imshow(BW);

    case 2
        g=rgb2gray(f);
        BW1 = edge(g,'Canny');
        axes(handles.proce);
        imshow(BW1);
        
    case 3
        g=rgb2gray(f);
        H = fspecial('laplacian');
        blurred= imfilter(g,H);
        axes(handles.proce);
        imshow(blurred);
        
    case 4
        g=rgb2gray(f);
        BW2 = edge(g,'Prewitt');
        axes(handles.proce);
        imshow(BW2);
        ...
    otherwise 
    msgbox('PLEASE SELECT DETECTION METHOD');
end

% --- Executes on button press in loadimage.
function loadimage_Callback(~, ~, handles)
% hObject    handle to loadimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

axes(handles.org);
[fn pn] = uigetfile('*.jpg','select image');
if fn == 0
    msgbox('NO FILE SELECTED');
else
    handles.I = imread(fullfile(pn,fn));
    imshow(handles.I);
set(handles.proce,'visible','off');
set(handles.text5,'String','');
end


% --- Executes on button press in rgb.
function rgb_Callback(hObject, eventdata, handles)
% hObject    handle to rgb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
z=gray2rgb(theImage);
imshow(z);
% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
I2 = flipdim(theImage ,2); 
imshow(I2);




% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
I3 = flipdim(theImage ,1); 
imshow(I3);

% --- Executes on button press in green.
function green_Callback(hObject, eventdata, handles)
% hObject    handle to green (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of green
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
G = theImage;
G(:,:,[1 3]) = 0;
image(G);

% --- Executes on button press in red.
function red_Callback(hObject, eventdata, handles)
% hObject    handle to red (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of red

set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
R = theImage;
R(:,:,2:3) = 0;
image(R);


% --- Executes on button press in blue.
function blue_Callback(hObject, eventdata, handles)
% hObject    handle to blue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of blue
set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
B = theImage;
B(:,:,1:2) = 0;
image(B);


% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3

set(handles.proce,'visible','on');
set(handles.text5,'String','Processed Image');
axes(handles.org);
theImage = getimage();
axes(handles.proce);
I3 = flipdim(theImage ,1); 
I4 = flipdim(I3,2); 
imshow(I4);
