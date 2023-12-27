function varargout = invKinematic(varargin)
% INVKINEMATIC MATLAB code for invKinematic.fig
%      INVKINEMATIC, by itself, creates a new INVKINEMATIC or raises the existing
%      singleton*.
%
%      H = INVKINEMATIC returns the handle to a new INVKINEMATIC or the handle to
%      the existing singleton*.
%
%      INVKINEMATIC('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INVKINEMATIC.M with the given input arguments.
%
%      INVKINEMATIC('Property','Value',...) creates a new INVKINEMATIC or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before invKinematic_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to invKinematic_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help invKinematic

% Last Modified by GUIDE v2.5 25-Dec-2023 08:31:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @invKinematic_OpeningFcn, ...
                   'gui_OutputFcn',  @invKinematic_OutputFcn, ...
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


% --- Executes just before invKinematic is made visible.
function invKinematic_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to invKinematic (see VARARGIN)

% Choose default command line output for invKinematic
handles.output = hObject;

% Create robotic arm model
handles.d_1 = 200;
handles.d_2 = 200;
handles.d_3 = 200;
handles.d_4 = 200; 
handles.d_5 = 200;

% theta d a alpha
L(1) = Link([0 handles.d_1 0 pi/2]);
L(2) = Link([0 0 handles.d_2 0]);
L(3) = Link([0 0 handles.d_3 0]);
L(4) = Link([0 0 0 pi/2]);
L(5) = Link([0 handles.d_5 0 0]);

L(1).qlim = [-pi pi];
L(2).qlim = [0 pi];
L(3).qlim = [-pi/2 0];
L(4).qlim = [0 pi];
L(5).qlim = [-pi pi];

Robot = SerialLink(L);
Robot.name = 'Lynxmotion Robot';

handles.points = [
    400, 0, 0, deg2rad(0), deg2rad(0);
    200, 0, 200, deg2rad(0), deg2rad(0);
    100, -150, 100, deg2rad(0), deg2rad(45);
    -100, -150, 200, deg2rad(30), deg2rad(45);
    200, 0, 100, deg2rad(30), deg2rad(45);
    0, 200, 150, deg2rad(30), deg2rad(45);
    -200, 0, 200, deg2rad(30), deg2rad(45);
    0, -200, 100, deg2rad(30), deg2rad(45);
];

% Store the robot arm model in the handles structure
handles.Robot = Robot;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes invKinematic wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = invKinematic_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btn_forward.
function btn_forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get joint angle
Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;
Th_4 = str2double(handles.Theta_4.String)*pi/180;
Th_5 = str2double(handles.Theta_5.String)*pi/180;

% Use the Robot model stored in the handles structure
Robot = handles.Robot;
 
% Draw a graphical representation of the robot using the current joint angles
Robot.plot([Th_1 Th_2 Th_3 Th_4 Th_5], 'tilesize', 120);

% Assume that the initial attitude is the initial transformation matrix T0
T0 = Robot.fkine([0 0 0 0 0]);
T0 = T0.double;

% Calculate the forward kinematics of the robot arm end effector
T = Robot.fkine([Th_1 Th_2 Th_3 Th_4 Th_5]);
T = T.double;

% Calculate the transformation matrix of the current posture relative to the initial posture
T_relative = T0 \ T;

% Extract relative rotation matrix
R_relative = T_relative(1:3, 1:3);

% Print information to the MATLAB console
fprintf('**********\n');
fprintf('Fkine done\n');
fprintf('__________\n');

cos_gamma = T(3,3);
gamma = atan2(sqrt(1-cos_gamma^2),-cos_gamma);

pitch = atan2(-R_relative(3,1), sqrt(R_relative(3,2)^2 + R_relative(3,3)^2));

beta = Th_5;

pitch_deg = pitch*180/pi;
gamma_deg = gamma*180/pi;
beta_deg = beta*180/pi;

% Update Roll-Pitch-Yaw display on GUI
handles.pitch.String = num2str(pitch_deg, '%.2f');
handles.yaw.String = num2str(beta_deg, '%.2f');

% Update the position of the robot arm end effector displayed on the GUI
handles.Pos_X.String = num2str(floor(T(1,4)));
handles.Pos_Y.String = num2str(floor(T(2,4)));
handles.Pos_Z.String = num2str(floor(T(3,4)));

% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get Object location
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);

roll = str2double(handles.roll.String)*pi/180;
pitch = str2double(handles.pitch.String)*pi/180;
yaw = str2double(handles.yaw.String)*pi/180;

% Use the Robot model stored in the handles structure
Robot = handles.Robot;

% T0 define the start point, all theta 0
%q0 = [0, 0, 0, 0, 0];
%T_0 = Robot.fkine(q0);
%T_0 = T_0.double;

% Define the transformation matrix of the target position
T_des = transl([PX PY PZ]) * rpy2tr(roll, pitch, yaw);

% n = 50;
% T = ctraj(T_0, T_des, n); % compute a Cartesian path

% Inverse kinematics by optimization with joint limits
J = Robot.ikcon(T_des) * 180/pi;
fprintf('**********\n');
fprintf('Ikine done\n');
fprintf('__________\n');

% Update the joint angles displayed on the GUI
handles.Theta_1.String = num2str(J(1));
handles.Theta_2.String = num2str(J(2));
handles.Theta_3.String = num2str(J(3));
handles.Theta_4.String = num2str(J(4));
handles.Theta_5.String = num2str(J(5));

% Draw the current position of the robot
Robot.plot(J*pi/180, 'tilesize', 120);

% --- Executes on button press in btn_inverse_close_form_Callback.
function btn_inverse_close_form_Callback_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse_close_form_Callback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Use the Robot model stored in the handles structure
Robot = handles.Robot;

% Get Object location
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);

pitch = str2double(handles.pitch.String)*pi/180;
yaw = str2double(handles.yaw.String)*pi/180;

gamma = pitch;

if(gamma > pi/2)
    rw = sqrt(PX^2 + PY^2) - handles.d_5*cos(gamma - pi/2);
    zw = PZ - handles.d_5*sin(gamma - pi/2);
end

if(gamma <= pi/2)
    rw = sqrt(PX^2 + PY^2) - handles.d_5*sin(gamma);
    zw = PZ + handles.d_5*cos(gamma);
end

theta1 = atan2(PY, PX);

cos_theta3 = ((zw - handles.d_1)^2 + rw^2 - handles.d_2^2 - handles.d_3^2)/(2*handles.d_2*handles.d_3);
sin_theta3 = -sqrt(1 - cos_theta3^2);

if(cos_theta3 == 1)
    sin_theta3 = 0;
end

theta3 = atan2(real(sin_theta3), cos_theta3);

beta = atan2(handles.d_3*sin(-theta3), handles.d_2 + handles.d_3*cos(-theta3));
theta2 = atan2(zw - handles.d_1,rw) + beta;

s = sqrt((zw - handles.d_1)^2 + rw^2);

cos_alpha = (s^2 + handles.d_3^2 - handles.d_2^2)/(2*s*handles.d_3);
sin_alpha = sqrt(1-cos_alpha^2);

if(cos_alpha == 1)
    sin_alpha = 0;
end

alpha = atan2(real(sin_alpha), cos_alpha);

theta4 = pitch + 2*alpha - (theta2 - 2*beta) - (-theta3);
theta5 = yaw;

% Update the joint angles displayed on the GUI
handles.Theta_1.String = theta1*180/pi;
handles.Theta_2.String = theta2*180/pi;
handles.Theta_3.String = theta3*180/pi;
handles.Theta_4.String = theta4*180/pi;
handles.Theta_5.String = theta5*180/pi;

% Draw the current position of the robot
Robot.plot([theta1, theta2, theta3, theta4, theta5], 'tilesize', 120);
fprintf('Ikine done\n');

 function slider1ContValCallback(hFigure,eventdata)
 % test it out - get the handles object and display the current value
 handles = guidata(hFigure);
% fprintf('slider value: %f\n',get(handles.Theta1_Slider,'Value'));
 sliderValue = get(handles.Theta1_Slider,'Value');
 set(handles.Theta_1,'String',num2str(floor(sliderValue)));
 
  function slider2ContValCallback(hFigure,eventdata)
 % test it out - get the handles object and display the current value
 handles = guidata(hFigure);
% fprintf('slider value: %f\n',get(handles.Theta2_Slider,'Value'));
 sliderValue = get(handles.Theta2_Slider,'Value');
 set(handles.Theta_2,'String',num2str(floor(sliderValue)));
 
  function slider3ContValCallback(hFigure,eventdata)
 % test it out - get the handles object and display the current value
 handles = guidata(hFigure);
% fprintf('slider value: %f\n',get(handles.Theta3_Slider,'Value'));
 sliderValue = get(handles.Theta3_Slider,'Value');
 set(handles.Theta_3,'String',num2str(floor(sliderValue)));

 function slider4ContValCallback(hFigure,eventdata)
 % test it out - get the handles object and display the current value
 handles = guidata(hFigure);
% fprintf('slider value: %f\n',get(handles.Theta4_Slider,'Value'));
 sliderValue = get(handles.Theta4_Slider,'Value');
 set(handles.Theta_4,'String',num2str(floor(sliderValue)));

 function slider5ContValCallback(hFigure,eventdata)
 % test it out - get the handles object and display the current value
 handles = guidata(hFigure);
% fprintf('slider value: %f\n',get(handles.Theta5_Slider,'Value'));
 sliderValue = get(handles.Theta5_Slider,'Value');
 set(handles.Theta_5,'String',num2str(floor(sliderValue)));

function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double

% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double

% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double

% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double

% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_5 as text
%        str2double(get(hObject,'String')) returns contents of Theta_5 as a double

% --- Executes during object creation, after setting all properties.
function Theta_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch as text
%        str2double(get(hObject,'String')) returns contents of pitch as a double


% --- Executes during object creation, after setting all properties.
function pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw as text
%        str2double(get(hObject,'String')) returns contents of yaw as a double


% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Theta_1.String = 0;
handles.Theta_2.String = 0;
handles.Theta_3.String = 0;
handles.Theta_4.String = 0;
handles.Theta_5.String = 0;


% --- Executes on button press in pushbutton4.
function resetThetaValues_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function btn_calculate_workspace_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Extract the robot from the handles structure
Robot = handles.Robot;

% Get the limits of each joint
qlim = Robot.qlim;

% Initialize the end effector position array
positions = [];

% Define the number of sampling points for each joint
n = 10;

% Generate workspace
for i = linspace(qlim(1,1), qlim(1,2), n)
    for j = linspace(qlim(2,1), qlim(2,2), n)
        for k = linspace(qlim(3,1), qlim(3,2), n)
            for l = linspace(qlim(4,1), qlim(4,2), n)
                for m = linspace(qlim(5,1), qlim(5,2), n)
                    T = Robot.fkine([i j k l m]);
                    T = T.double;
                    if isnumeric(T) && all(size(T) == [4 4])
                        positions = [positions; T(1:3,4)'];
                    else
                        error('Unexpected format of transformation matrix T.');
                    end
                end
            end
        end
    end
end

% Draw the workspace
figure;
scatter3(positions(:,1), positions(:,2), positions(:,3), '.');
title('workspace');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Indicate completion of workspace calculation
fprintf('Workspace calculation done.\n');



% --- Executes on button press in path_tracing.
function path_tracing_Callback(hObject, eventdata, handles)
% hObject    handle to path_tracing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Define 5 positions and directions of the end effector (expressed in Cartesian coordinates)

