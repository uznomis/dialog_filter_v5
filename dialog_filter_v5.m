function varargout = dialog_filter_v5(varargin)
% DIALOG_FILTER_V5 MATLAB code for dialog_filter_v5.fig
%      DIALOG_FILTER_V5, by itself, creates a new DIALOG_FILTER_V5 or raises the existing
%      singleton*.
%
%      H = DIALOG_FILTER_V5 returns the handle to a new DIALOG_FILTER_V5 or the handle to
%      the existing singleton*.
%
%      DIALOG_FILTER_V5('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIALOG_FILTER_V5.M with the given input arguments.
%
%      DIALOG_FILTER_V5('Property','Value',...) creates a new DIALOG_FILTER_V5 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before dialog_filter_v5_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to dialog_filter_v5_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help dialog_filter_v5

% Last Modified by GUIDE v2.5 17-Oct-2016 14:25:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @dialog_filter_v5_OpeningFcn, ...
                   'gui_OutputFcn',  @dialog_filter_v5_OutputFcn, ...
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


% --- Executes just before dialog_filter_v5 is made visible.
function dialog_filter_v5_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to dialog_filter_v5 (see VARARGIN)

% Choose default command line output for dialog_filter_v5
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes dialog_filter_v5 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = dialog_filter_v5_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function editBoxDistance_Callback(hObject, eventdata, handles)
% hObject    handle to editBoxDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dd = str2double(get(hObject,'String'));
if isnan(dd)
    error('Distance invalid.');
else
    set(hObject,'UserData',dd);
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of editBoxDistance as text
%        str2double(get(hObject,'String')) returns contents of editBoxDistance as a double


% --- Executes during object creation, after setting all properties.
function editBoxDistance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editBoxDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
editBoxDistance_Callback(hObject, eventdata, handles);


% --- Executes on selection change in paramMenu.
function paramMenu_Callback(hObject, eventdata, handles)
% hObject    handle to paramMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = cellstr(get(hObject,'String'));
val = get(hObject,'Value');

switch str{val};
    case 'Axial'
        col = 2;
    case 'Shear'
        col = 3;
    case 'Dilation'
        col = 4;
    case 'Friction'
        col = 5;
end
set(hObject,'UserData',col);
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns paramMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from paramMenu


% --- Executes during object creation, after setting all properties.
function paramMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to paramMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
paramMenu_Callback(hObject, eventdata, handles);


% --- Executes on button press in importC40button.
function importC40button_Callback(hObject, eventdata, handles)
% hObject    handle to importC40button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[name,path] = uigetfile(...
    'C:\Users\User\Documents\Copy of test files on ROGA1\*.c40',...
    'Select a C40 file');
if name == 0
    error('No file imported!');
end
importButtonData.name = name;
importButtonData.path = path;
s = importdata([path,name]);
lineCrop = 2;
if isstruct(s)
    data = s.data(1+lineCrop:end-lineCrop,:);
    importButtonData.dt = data(2,21)-data(1,21);
    importButtonData.headers = s.colheaders;
else
    data = s(1+lineCrop:end-lineCrop,:);
end
set(hObject,'UserData',importButtonData);
set(handles.exportButton,'UserData',data);

data = [data(:,21:27) data(:,7)]; % 8-columns

d = data(:,2);

ts = timeseries(data,d);
editBoxDistance_Callback(handles.editBoxDistance, eventdata, handles);
new_d = d(1):...
    get(handles.editBoxDistance,'UserData')...
    :d(end); % new vector of time with fixed interval

data_res = resample(ts,new_d); % data resampled at constant interval
d_data = [data_res.time data_res.data(:,4:8) data_res.data(:,1)]; % 7-columns

set(handles.launchFilterButton,'UserData',d_data);
msgbox('Import successful!');
% handles.distance = data_res.time;
% handles.time = data_res.data(:,1);
% handles.axial = data_res.data(:,4);
% handles.shear = data_res.data(:,5);
% handles.dilation = data_res.data(:,6);
% handles.friction = data_res.data(:,7);

guidata(hObject, handles);

% --- Executes on button press in launchFilterButton.
function launchFilterButton_Callback(hObject, eventdata, handles)
% hObject    handle to launchFilterButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
d_data = get(hObject,'UserData');
col = get(handles.paramMenu,'UserData');
ifilter_mod(d_data(:,1),d_data(:,col));
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in saveTempButton.
function saveTempButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveTempButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    ry = evalin('base', 'ry');
    d_data = get(handles.launchFilterButton,'UserData');
    col = get(handles.paramMenu,'UserData');
    d_data(:,col) = ry';
    evalin('base','clear ry');
    set(handles.launchFilterButton,'UserData',d_data);
    msgbox('Temp data saved successfully!');
catch ME
    msgbox('Save unsuccessful!');
end
guidata(hObject, handles);


% --- Executes on button press in exportButton.
function exportButton_Callback(hObject, eventdata, handles)
% hObject    handle to exportButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data7col = get(handles.launchFilterButton,'UserData');
importData = get(handles.importC40button,'UserData');
rawData = get(hObject,'UserData');
t = data7col(:,7);
ts = timeseries(data7col,t);
new_t = t(1):importData.dt:t(end); % new vector of time with fixed interval
data_res = resample(ts,new_t); % data resampled at constant interval
ind = find(rawData(:,21)>=t(1));
data44col = [rawData(ind:ind+length(new_t)-1,:) data_res.data(:,2:5)];

[~,dryname,~] = fileparts(importData.name);
fid = fopen([importData.path,'Filtered_',dryname,'.c44'],'w');
    
% fprintf(fid,...
%     '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n',...
%     'Time_ms','MotorSpeed_volts','AxialLoad_volt','TorqueForce_volt','Unused','EncoderCounts','CROCPress_volt',...
%     'Eddy2_volt','Eddy3_volt','Eddy4_volt','TC1_C','TC2_C','EncoderVolts_volt','MotorPower_kW','Pump1PresssureVolts',...
%     'Pump2PressureVolts','Variable17','SealPressureVolts','RequestedControlVolts','RequestedProgramVolts','Time_s',...
%     'Distance_m','Velocity_m_s','AxialStress_MPa','ShearStress_MPa','Dilation_microns','Friction','Temperature_C',...
%     'EncoderCounts','EncoderVolts_volts','EddySensor1','EddySensor2','EddySensor3','EddySensor4',...
%     'Velocity_motor_ms','Distance_motor_m','AccelX1','AccelZ2','AccelY2','MER',...
%     'Filtered_axial','Filtered_shear','Filtered_dilation','Filtered_friction');

for i = 1:length(importData.headers)
    fprintf(fid,'%s,',importData.headers{i});
end

fprintf(fid,...
    '%s,%s,%s,%s\n',...
    'Filtered_axial','Filtered_shear','Filtered_dilation','Filtered_friction');

fprintf(fid,'%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e,%10e\n',...
    data44col');

fclose(fid);

msgbox('Export successful!');


% --- Executes during object creation, after setting all properties.
function textInstruction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textInstruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
instring = {...
    '1.Select distance interval for conversion.';
    '2.Import a c40 file.';
    '3.Select a parameter to filter.';
    '4.In the pop up ifilter app, filter and press S to save.';
    '5.Press Save temp to save the output of ifilter app temporarily.';
    '6.Repeat 3. ~ 5. until satisfied.';
    '7.Export the 44-column file *.c44 to the same folder.'
    };
outstr = textwrap(hObject,instring);
set(hObject,'String',outstr);
