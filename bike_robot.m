function varargout = Car_like(varargin)
% CAR_LIKE MATLAB code for Car_like.fig
%      CAR_LIKE, by itself, creates a new CAR_LIKE or raises the existing
%      singleton*.
%
%      H = CAR_LIKE returns the handle to a new CAR_LIKE or the handle to
%      the existing singleton*.
%
%      CAR_LIKE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAR_LIKE.M with the given input arguments.
%
%      CAR_LIKE('Property','Value',...) creates a new CAR_LIKE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Car_like_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Car_like_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Car_like

% Last Modified by GUIDE v2.5 04-Aug-2021 22:28:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Car_like_OpeningFcn, ...
                   'gui_OutputFcn',  @Car_like_OutputFcn, ...
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

% --- Executes just before Car_like is made visible.
function Car_like_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Car_like (see VARARGIN)

% Choose default command line output for Car_like
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes Car_like wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%%%%%LEO%%%%%
tic %inicia conteo de tiempo
handles.timer = timer(...
    'ExecutionMode','fixedSpacing', ...       % Run timer repeatedly
    'Period', 0.1, ...                        % Initial period 
    'TimerFcn', {@actualizar,hObject});       %'TimerFcn', {@actualizar,hObject}); % Specify callback function
start(handles.timer) % Configurar e iniciar temporizador
reset_var_globales(hObject, eventdata, handles); % Resetear variables globales

% --- Outputs from this function are returned to the command line.
function varargout = Car_like_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: delete(hObject) closes the figure
delete(hObject);

%%%%%LEO%%%%%
stop(timerfindall); %Parar temporizador
delete(timerfindall); % Eliminar temporizador
clc % Limpiar command window

% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1

%%%%%LEO%%%%% 
global act_f1 act_f2;
 act_f1=get(handles.radiobutton1,'Value'); % ¿Activar función 1 o 2?
 act_f2=get(handles.radiobutton2,'Value'); % ¿Activar función 1 o 2?
 reset_var_globales(hObject, eventdata, handles) % Resetear variables globales
 
 % --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2

global act_f1 act_f2
 act_f1=get(handles.radiobutton1,'Value'); % ¿Activar función 1 o 2?
 act_f2=get(handles.radiobutton2,'Value'); % ¿Activar función 1 o 2?
 reset_var_globales(hObject, eventdata, handles) % Resetear variables globales

 % --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on mouse press over figure background.
function figure1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%get(handles.figure1,'CurrentPoint')



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

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global px py
px=str2num(get(handles.edit1,'String'));
py=str2num(get(handles.edit2,'String'));



%%%%%LEO%%%%% FUNCIONES PROPIAS

%%%%%LEO%%%%% 
function actualizar(hObject,eventdata,hfigure) % Funcion del temporizador definidad en el "Car_like_OpeningFcn"
%function actualizar(hObject,eventdata,handles)
%Timer timer1 callback, called each time timer iterates.
%timerfindall y despues delete(ans) para borrar todos los timers

global vel angulo act_f1 act_f2 xd yd thetad px py

handles = guidata(hfigure); %linea desde ejemplo del timer matlab

set(handles.text7,'string',strcat('Tiempo [s]:  ',num2str(toc,'%.1f'))) %Muestra tiempo transcurrido desde el TIC
tomar_sliders(hObject, eventdata, handles) %Toma valor de los "sliders"

if act_f2==1 %si está activa la función 2 (Ir a P(x,y)) el angulo y velocidad no son las de los "sliders"
    vel=0.2*sqrt((px-xd)^2+(py-yd)^2); %estimación de velocidad en función de la distancia al punto deseado
    thetap=atan2(py-yd,px-xd); %estimación de dirección en función de la distancia al punto deseado
    angulo=(thetap-thetad); %calcula el angulo de dirección restando al angulo de orientación deseado el angulo de orientación actual
    if angulo>=pi/2 %no permitir que el angulo de dirección sea mayor que 90°
        angulo=85*pi/180;
    elseif angulo<=-pi/2 %no permitir que el angulo de dirección sea mayor que 90°
        angulo=-85*pi/180;
    end
end

[x,y,theta]=modelo1(vel,angulo,1); %forma de calcular 1 (aprox)
[x_d,y_d,theta_d]=modelo2(vel,angulo,1); %forma de calcular 2 (diff)

T1 = transl2(x, y) * trot2(theta); %toolbox de robotica
T2 = transl2(x_d, y_d) * trot2(theta_d); %toolbox de robotica
if act_f1==1 %graficar si esta activa la función 1
    plot(handles.axes1,[x x+T1(1,1)],[y y+T1(2,1)],x,y,'o',x+T1(1,1),y+T1(2,1),'o',...
        [x_d x_d+T2(1,1)],[y_d y_d+T2(2,1)],x_d,y_d,'*',x_d+T2(1,1),y_d+T2(2,1),'*')
    xlim(handles.axes1,[-10 10])
    ylim(handles.axes1,[-10 10]) 
end
if act_f2==1 %graficar si esta activa la función 1
    plot(handles.axes1,[x x+T1(1,1)],[y y+T1(2,1)],x,y,'o',x+T1(1,1),y+T1(2,1),'o',...
        [x_d x_d+T2(1,1)],[y_d y_d+T2(2,1)],x_d,y_d,'*',x_d+T2(1,1),y_d+T2(2,1),'*',px,py,'*')
    xlim(handles.axes1,[-10 10])
    ylim(handles.axes1,[-10 10]) 
end

if (x>10)||(x<-10)||(y>10)||(y<-10)||(x_d>10)||(x_d<-10)||(y_d>10)||(y_d<-10) % resetea si se sale de la gráfica
    reset_var_globales(hObject, eventdata, handles);
    tomar_sliders(hObject, eventdata, handles)
end

%%%%%LEO%%%%% 
function [x2,y2,theta2]=modelo1(v,ang,act) %ejecuta modelo 1 para cálculos
global  x1 y1 t1 theta1 L
if act==1 %activar el cálculo con este modelo?
    t2=toc; %Registra tiempo
    RB=L/tan(ang); % calcula RB en funcion de distancia entre ruedas y angulo de dirección
    theta2=v/RB*(t2-t1)+theta1; %Se calcula el ángulo que el robot ha rotado en ese intervalo de tiempo.
    vx2=v*cos(theta2); %se descompone la velocidad en X
    vy2=v*sin(theta2); %se descompone la velocidad en Y
    x2=vx2*(t2-t1)+x1; %Se calcula la distancia que el robot ha avanzado en X en ese intervalo de tiempo.
    y2=vy2*(t2-t1)+y1; %Se calcula la distancia que el robot ha avanzado en Y en ese intervalo de tiempo.
    % Se establecen condiciones iniciales para el siguiente ciclo:
    theta1=theta2; %CI ángulo rotado
    x1=x2; %CI posición en x
    y1=y2; %CI posición en y
    t1=t2; %tiempo anterior
end

%%%%%LEO%%%%% 
function [xt,yt,thetat]=modelo2(v,ang,act) %t significa para todo tiempo
 global x1d y1d t1d theta1d L thetad xd yd t_temd vd vel angd angulo
 
 t=toc;
 if (vd~=vel)||(angd~=angulo)
    theta1d=thetad;
    x1d=xd;
    y1d=yd;
    vd=vel;
    angd=angulo;
    t1d=t_temd;
end
if act==1 %activar el cálculo con este modelo?
    if ang==0 
        ang=1e-10; % si es cero queda indeterminado
    end
    RB=L/tan(ang);
    thetat=theta1d+v*(t-t1d)/RB;
    xt=x1d+RB*(sin(thetat)-sin(theta1d));
    yt=y1d-RB*(cos(thetat)-cos(theta1d));
    xd=xt;
    yd=yt;
    thetad=thetat;
    t_temd=t;
end

%%%%%LEO%%%%% 
function reset_var_globales(hObject, eventdata, handles) %reiniciar todas las variables globales

global L vel angulo act_f1 act_f2 px py %generales
L=1; % longitud (distancia) entre ruedas
vel =0; %velocidad tangencial del robot
angulo=0; %angulo de dirección
act_f1=get(handles.radiobutton1,'value'); %toogle función 1 de sliders para velocidad y angulo de dirección
act_f2=get(handles.radiobutton2,'value'); %toogle función 2 de mover a un punto
px=5; %valor por defecto de posición x deseada
py=5; %valor por defecto de posición y deseada

set(handles.text2,'string',strcat('Velocidad [%]:  ',num2str(vel,'%3.0f'))) %ajusta texto para el slider de velocidad
set(handles.text3,'string',strcat('Ang. Dir. [°]:   ',num2str(angulo,'%3.0f'))) %ajusta texto para el slider de angulo
set(handles.slider1,'value',vel); %ajusta slider de velocidad
set(handles.slider2,'value',angulo);  %ajusta slider de angulo

global t1 theta1 x1 y1 vx1 vy1 % para el modelo 1
t1=toc; %tiempo anterior
theta1=0; %Ø anterior
x1=0; %posición anterior en x
y1=0; %posición anterior en y
vx1=0; %velocidad anterior en x
vy1=0; %velocidad anterior en y

global t1d theta1d x1d y1d xd yd thetad vd angd t_temd % para el modelo 2 d->ecuaciones_diferenciales
t1d=toc; %tiempo inicial
theta1d=0; % Øinical
thetad=0; % theta actual
x1d=0; % posición inicial en x
y1d=0; % posición incial en y
xd=0; % posición actual en X
yd=0; % posición actual en Y
vd=0; % velocidad actual
angd=0; %angulo de dirección (gamma) actual
t_temd=toc; %control de tiempo para cuando cambia vel y angulo asumidos constantes. t1d->t_temd cuando los sliders cambien

%%%%%LEO%%%%% 
function tomar_sliders(hObject, eventdata, handles)
global vel angulo
e_vel=0.02; %escala de la velocidad
e_ang=pi/180; %escala del angulo
vel=get(handles.slider1,'value')*e_vel; %convierte valor del slider en velocidad
angulo=get(handles.slider2,'value')*e_ang; %convierte valor del slider en angulo
set(handles.text2,'string',strcat('Velocidad [%]:  ',num2str(vel/e_vel,'%3.0f'))); %Muestra valor del slider de velocidad
set(handles.text3,'string',strcat('Ang. Dir. [°]:   ',num2str(angulo/e_ang,'%3.0f'))); %Muestra valor del slider de ángulo
