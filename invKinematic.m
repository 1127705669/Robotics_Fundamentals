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

% Last Modified by GUIDE v2.5 27-Dec-2023 23:04:03

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

% Motion constraints for each jointD
L(1).qlim = [-pi pi];
L(2).qlim = [0 pi];
L(3).qlim = [-pi/2 0];
L(4).qlim = [0 pi];
L(5).qlim = [-pi pi];

% Define the maximum speed and acceleration of the end effector
handles.maxVelocity = 50;
handles.maxAcceleration = 20;

Robot = SerialLink(L);
Robot.name = 'Lynxmotion Robot';

handles.points = [
    400, 0, 0 ,deg2rad(0), deg2rad(0);
    200, 0, 0 ,deg2rad(0), deg2rad(0);
    200, 200, 0, deg2rad(0), deg2rad(0);
    200, 200, -100, deg2rad(0), deg2rad(0);
    200, 200, 100, deg2rad(0), deg2rad(0);
    200, -200, 100, deg2rad(0), deg2rad(0);
    200, -200, -100, deg2rad(0), deg2rad(0);
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

beta = Th_5;

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

pitch = atan2(T(3,1)/cos(Th_5), -T(3,3));

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

% Define the number of random samples for Monte Carlo simulation
n = 10000; % You can adjust this number based on the desired accuracy and computational resources

% Generate workspace using Monte Carlo sampling
for i = 1:n
    % Randomly sample joint angles within their limits
    q1 = qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand;
    q2 = qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand;
    q3 = qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand;
    q4 = qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand;
    q5 = qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand;

    % Compute forward kinematics for random joint angles
    T = Robot.fkine([q1 q2 q3 q4 q5]);
    T = T.double;
    if isnumeric(T) && all(size(T) == [4 4])
        positions = [positions; T(1:3,4)'];
    else
        error('Unexpected format of transformation matrix T.');
    end
end

% Draw the workspace
figure;
scatter3(positions(:,1), positions(:,2), positions(:,3), '.');
title('Robot Workspace using Monte Carlo Simulation');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Indicate completion of workspace calculation
fprintf('Workspace calculation done using Monte Carlo Simulation.\n');


% --- Executes on button press in free_motion_path_tracing_Callback.
function free_motion_path_tracing_Callback_Callback(hObject, eventdata, handles)
% hObject    handle to free_motion_path_tracing_Callback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 提取笛卡尔坐标点
points = handles.points;

% 提取最大速度和加速度
maxVelocity = handles.maxVelocity;
maxAcceleration = handles.maxAcceleration;

% 确定时间步长
dt = 0.1; % 例如，每0.1秒计算一次

% 使用线性插值生成路径
path = [];
for i = 1:(size(points, 1) - 1)
    segment = interp1([0 1], [points(i, 1:3); points(i + 1, 1:3)], 0:dt:1, 'linear');
    path = [path; segment];
end

% 初始化末端执行器轨迹存储
endEffectorPath = [];

% 遍历路径上的每个点
for i = 1:size(path, 1)
    % 假设path中的每行是一个目标位置
    target = path(i, :);

    % 更新handles结构体中的位置和姿态信息
    handles.Pos_X.String = num2str(target(1));
    handles.Pos_Y.String = num2str(target(2));
    handles.Pos_Z.String = num2str(target(3));
    % 假设pitch和yaw是固定的，您可以根据需要进行修改
    handles.pitch.String = '0'; 
    handles.yaw.String = '45';

    % 调用逆运动学函数计算关节角度
    btn_inverse_close_form_Callback_Callback(hObject, eventdata, handles);

    % 此处可能需要从handles结构体中提取计算出的关节角度
    % 然后使用这些角度更新机器人模型的姿态
    theta1 = str2double(handles.Theta_1.String)*pi/180;
    theta2 = str2double(handles.Theta_2.String)*pi/180;
    theta3 = str2double(handles.Theta_3.String)*pi/180;
    theta4 = str2double(handles.Theta_4.String)*pi/180;
    theta5 = str2double(handles.Theta_5.String)*pi/180;

    % 绘制当前机器人姿态
    handles.Robot.plot([theta1, theta2, theta3, theta4, theta5], 'tilesize', 120);
    % 获取并存储末端执行器当前位置
    T = handles.Robot.fkine([theta1, theta2, theta3, theta4, theta5]);
    T = T.double;
    endEffectorPosition = T(1:3, 4)';
    endEffectorPath = [endEffectorPath; endEffectorPosition];

    % 绘制末端执行器的轨迹
    hold on; % 保持当前图形
    plot3(endEffectorPath(:,1), endEffectorPath(:,2), endEffectorPath(:,3), 'r', 'LineWidth', 2);
    hold off; % 解除保持

    drawnow;
    pause(0.05); % 根据需要调整暂停时间以控制动画速度
end


% --- Executes on button press in straight_line_path_tracing.
function straight_line_path_tracing_Callback(hObject, eventdata, handles)
% hObject    handle to straight_line_path_tracing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 提取笛卡尔坐标点
points = handles.points;

% 时间步长
dt = 0.1;

% 生成样条插值路径
t = 1:size(points, 1);
tt = linspace(1, size(points, 1), 100*size(points, 1));
path = spline(t, points(:, 1:3)', tt)';

% 初始化末端执行器轨迹存储
endEffectorPath = [];

% 遍历路径上的每个点
for i = 1:size(path, 1)
    % 目标位置
    target = path(i, :);

    % 更新handles结构体中的位置和姿态信息
    handles.Pos_X.String = num2str(target(1));
    handles.Pos_Y.String = num2str(target(2));
    handles.Pos_Z.String = num2str(target(3));
    handles.pitch.String = '0'; 
    handles.yaw.String = '45';

    % 调用逆运动学函数计算关节角度
    btn_inverse_close_form_Callback_Callback(hObject, eventdata, handles);

    % 提取关节角度
    theta1 = str2double(handles.Theta_1.String)*pi/180;
    theta2 = str2double(handles.Theta_2.String)*pi/180;
    theta3 = str2double(handles.Theta_3.String)*pi/180;
    theta4 = str2double(handles.Theta_4.String)*pi/180;
    theta5 = str2double(handles.Theta_5.String)*pi/180;

    % 绘制当前机器人姿态
    handles.Robot.plot([theta1, theta2, theta3, theta4, theta5], 'tilesize', 120);

    % 获取并存储末端执行器当前位置
    T = handles.Robot.fkine([theta1, theta2, theta3, theta4, theta5]);
    T = T.double;
    endEffectorPosition = T(1:3, 4)';
    endEffectorPath = [endEffectorPath; endEffectorPosition];

    % 绘制末端执行器的轨迹
    hold on; % 保持当前图形
    plot3(endEffectorPath(:,1), endEffectorPath(:,2), endEffectorPath(:,3), 'r', 'LineWidth', 2);
    hold off; % 解除保持

    drawnow;
    pause(0.05);
end

function plot_cylinder_obstacle(center, radius, height)
    % center: 圆柱体中心的[x, y, z]坐标
    % radius: 圆柱体的半径
    % height: 圆柱体的高度

    [X, Y, Z] = cylinder(radius, 30); % 30表示圆柱体的面数，增加这个值可以使圆柱体更平滑
    Z = Z * height - height / 2 + center(3); % 调整圆柱体的高度
    X = X + center(1); % 调整圆柱体的X坐标
    Y = Y + center(2); % 调整圆柱体的Y坐标

    % 绘制圆柱体
    surf(X, Y, Z, 'FaceAlpha', 0.5); % FaceAlpha用于设置透明度
    hold on;

% --- Executes on button press in object_avoidance_path_tracing.
function object_avoidance_path_tracing_Callback(hObject, eventdata, handles)
% hObject    handle to object_avoidance_path_tracing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 提取关节角度
theta1 = str2double(handles.Theta_1.String)*pi/180;
theta2 = str2double(handles.Theta_2.String)*pi/180;
theta3 = str2double(handles.Theta_3.String)*pi/180;
theta4 = str2double(handles.Theta_4.String)*pi/180;
theta5 = str2double(handles.Theta_5.String)*pi/180;

% 绘制当前机器人姿态
handles.Robot.plot([theta1, theta2, theta3, theta4, theta5], 'tilesize', 120);
hold on; % 保持当前图形，以便添加障碍物

% 提取笛卡尔坐标点
points = handles.points;
% 定义并绘制障碍物
% 假设您想将障碍物放在点3和点4之间
points = handles.points;
center = (points(3,:) + points(4,:)) / 2;
radius = 50; % 圆柱体半径
height = 200; % 圆柱体高度

[X, Y, Z] = cylinder(radius, 30);
Z = Z * height - height / 2 + center(3);
X = X + center(1);
Y = Y + center(2);

% 绘制圆柱体
surf(X, Y, Z, 'FaceAlpha', 0.5); % FaceAlpha用于设置透明度

% 设置图形属性
axis equal;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off; % 解除保持
