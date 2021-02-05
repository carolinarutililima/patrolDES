function varargout = carol(varargin)


% CAROL MATLAB code for carol.fig
%      CAROL, by itself, creates a new CAROL or raises the existing
%      singleton*.
%
%      H = CAROL returns the handle to a new CAROL or the handle to
%      the existing singleton*.
%
%      CAROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAROL.M with the given input arguments.
%
%      CAROL('Property','Value',...) creates a new CAROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before carol_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to carol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help carol

% Last Modified by GUIDE v2.5 01-Jun-2019 23:06:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @carol_OpeningFcn, ...
    'gui_OutputFcn',  @carol_OutputFcn, ...
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



% --- Executes just before carol is made visible.
function carol_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to carol (see VARARGIN)

% Choose default command line output for carol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes carol wait for user response (see UIRESUME)
% uiwait(handles.figure1);



% --- Outputs from this function are returned to the command line.
function varargout = carol_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%% Starting the program
disp('Program started');
global vrep;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
global clientID;
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

disp('Connected to remote API server');

% enable the synchronous mode on the client:
vrep.simxSynchronous(clientID,true);

% start the simulation:
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
%Handles
global left_Motor;
[returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Kilobot_Revolute_jointLeftLeg',vrep.simx_opmode_blocking);
global right_Motor;
[returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Kilobot_Revolute_jointRightLeg',vrep.simx_opmode_blocking);
global left_Motor2;
[returnCode,left_Motor2]=vrep.simxGetObjectHandle(clientID,'Kilobot_Revolute_jointLeftLeg#0',vrep.simx_opmode_blocking);
global right_Motor2;
[returnCode,right_Motor2]=vrep.simxGetObjectHandle(clientID,'Kilobot_Revolute_jointRightLeg#0',vrep.simx_opmode_blocking);
global prox_Sensor1;
[returnCode,prox_Sensor1]=vrep.simxGetObjectHandle(clientID,'Kilobot_Proximity_sensor',vrep.simx_opmode_blocking);
global prox_Sensor2;
[returnCode,prox_Sensor2]=vrep.simxGetObjectHandle(clientID,'Kilobot_Proximity_sensor#0',vrep.simx_opmode_blocking);


[returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,prox_Sensor1,vrep.simx_opmode_streaming);
[returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,prox_Sensor2,vrep.simx_opmode_streaming);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor2,0,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor2,0,vrep.simx_opmode_blocking);
global num_kilobots;
global kilohandles;
disp(kilohandles);
vrep.simxSynchronous(clientID,false);
num_kilobots = 0;
[returnCode,handles,intData,floatData,stringData]=vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking);
a=size(handles);
s2='KilobotD';
for i=1:a(2)
    s1=stringData{i,1};
    s4 = size(s1);
    if (s4(2)>=8) %8 in KilobotD 13 for Vision_sensor
        %strcmp(s1,s2)
        s3 = s1(1:8); %8 in KilobotD
        if(strcmp(s3,s2))
            num_kilobots = num_kilobots+1;
            kilohandles(num_kilobots) = handles(i);
            vrep.simxGetObjectPosition(clientID,kilohandles(num_kilobots),-1,vrep.simx_opmode_streaming);
            vrep.simxGetObjectOrientation(clientID,kilohandles(num_kilobots),-1,vrep.simx_opmode_streaming);
        end
    end
end

runsignal = 3;
TEXT = ['There is(are)', num2str(num_kilobots) ,' kilot(s)'];
disp(TEXT);
vrep.simxSetIntegerSignal(clientID,'runsignal',runsignal,vrep.simx_opmode_oneshot);
pause(2);
vrep.simxClearIntegerSignal(clientID,'runsignal',vrep.simx_opmode_oneshot);

%% global variables
global list; % save the transitions
list = ["start"];
global PatrolAchieved;
PatrolAchieved = [0 0];
global Target;
Target = ["a21","b12"];
global counttimescolission;
counttimescolission = [0 0];
global Disable;
Disable = "";
global countPointRa;
countPointRa = 0;
global countPointRb;
countPointRb = 0;
global DisableRa;
DisableRa = [""];
global DisableRb;
DisableRb = [""];
global achievedPoint;
achievedPoint = [0 0];
global PatruCellRa;
PatruCellRa = 1;
global PatruCellRb;
PatruCellRb = 2;
global failMOV_Ra;
failMOV_Ra = 0;
global failMOV_Rb;
failMOV_Rb = 0;
global failSENS_Ra;
failSENS_Ra = 0;
global failSENS_Rb;
failSENS_Rb = 0;
global countR1;
countR1 = 0;
global countR2;
countR2 = 0;
global running;
running = 0;
global timeStart;
timeStart = clock;
global robotColision;
robotColision = [0 0];
global countcollision;
countcollision = [0 0];
global batSituationRa;
batSituationRa = 3;
global batSituationRb;
batSituationRb = 3;
global PatrollingRa;
PatrollingRa = 0;
global PatrollingRb;
PatrollingRb = 0;
global okRa;
okRa = 0;
global okRb;
okRb = 0;
global cantRb;
cantRb = 0;
global cantRa;
cantRa = 0;
global idlenessRa;
idlenessRa = [0 0 0 0];
global idlenessRb;
idlenessRb = [0 0 0 0];
 

%% Finate State Machines Variables
alltransitionsRa = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];
alltransitionsRb = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
global disableTransitionsRa;
disableTransitionsRa = ["a20F", "a02", "a20", "a21", "a23", "a32", "a34", "a41", "a43", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "ok_batRa", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra"];
global enableTransitionsRa;
enableTransitionsRa = setdiff(alltransitionsRa, disableTransitionsRa);
global disableTransitionsRb;
disableTransitionsRb = ["b02", "b12", "b14", "b20", "b20F", "b32", "b34", "b41", "b43", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "ok_batRb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb"];
global enableTransitionsRb;
enableTransitionsRb = setdiff(alltransitionsRb, disableTransitionsRb);
global elementsDFailRa;
elementsDFailRa = ["rstF_Ra"];
global elementsDFailRb;
elementsDFailRb = ["rstF_Rb"];
global elementsDBatRa;
elementsDBatRa = [""];
global elementsDBatRb;
elementsDBatRb = [""];
global elementsDgeoRa;
elementsDgeoRa = ["a20F", "a02", "a20", "a21", "a23", "a32", "a34", "a41", "a43", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "ok_batRa", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
global elementsDgeoRb;
elementsDgeoRb = ["b02", "b12", "b14", "b20", "b20F", "b32", "b34", "b41", "b43", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "ok_batRb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
global elementsDSupFailRa;
elementsDSupFailRa = ["rstF_Ra"];
global elementsDSupFailRb;
elementsDSupFailRb = ["rstF_Rb"];
global elementsDSupBatRa;
elementsDSupBatRa = ["a20", "ok_batRa"];
global elementsDSupBatRb;
elementsDSupBatRb = ["b20", "ok_batRb"]; 
global elementsDReturnC1_Ra;
elementsDReturnC1_Ra = [""];
global elementsDReturnC1_Rb;
elementsDReturnC1_Rb = [""];
global elementsDReturnC2_Ra;
elementsDReturnC2_Ra = [""];
global elementsDReturnC2_Rb;
elementsDReturnC2_Rb = [""];
global elementsDReturnC3_Ra;
elementsDReturnC3_Ra = [""];
global elementsDReturnC3_Rb;
elementsDReturnC3_Rb = [""];
global elementsDReturnC4_Ra;
elementsDReturnC4_Ra = [""];
global elementsDReturnC4_Rb;
elementsDReturnC4_Rb = [""];
global CurStateSUPFailRa;
CurStateSUPFailRa = 0;
global CurStateSUPFailRb;
CurStateSUPFailRb = 0;
global CurStateSUPBatRa;
CurStateSUPBatRa = 0;
global CurStateSUPBatRb;
CurStateSUPBatRb = 0;
global CurStateReturnC1_Ra;
CurStateReturnC1_Ra = 0;
global CurStateReturnC1_Rb;
CurStateReturnC1_Rb = 0;
global CurStateReturnC2_Ra;
CurStateReturnC2_Ra = 0;
global CurStateReturnC2_Rb;
CurStateReturnC2_Rb = 0;
global CurStateReturnC3_Ra;
CurStateReturnC3_Ra = 0;
global CurStateReturnC3_Rb;
CurStateReturnC3_Rb = 0;
global CurStateReturnC4_Ra;
CurStateReturnC4_Ra = 0;
global CurStateReturnC4_Rb;
CurStateReturnC4_Rb = 0;
global CurStateFailRa;
CurStateFailRa = 0;
global CurStateFailRb;
CurStateFailRb = 0;
global CurStateBatRa;
CurStateBatRa = 0;
global CurStateBatRb;
CurStateBatRb = 0;
global CurStateGeoA;
CurStateGeoA = 0;
global CurStateGeoB;
CurStateGeoB = 1;
global count
count = 1;


%% Robots Finite State Machine Espeficications


function [] = FailRa(event)
global CurStateFailRa;
global elementsDFailRa;
alltransitions = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];

switch (CurStateFailRa)
    case 0,
        if (event == "a02" || event == "a12" || event == "a14" || event ==  "a20" || event ==  "a21" ||  event ==  "a23" || event ==  "a32" || event ==  "a34" || event ==  "a41" || event == "a43" || event == "PatC1_Ra" || event == "PatC2_Ra" || event == "PatC3_Ra" || event == "PatC4_Ra" || event == "FPatC1_Ra" || event == "FPatC2_Ra" || event == "FPatC3_Ra" || event == "FPatC4_Ra")
            CurStateFailRa = 0;
            elementsDFailRa = ["rstF_Ra"];
        elseif (event == "failMOV_Ra")
            CurStateFailRa = 1;
            elementsDFailRa = ["a02", "a12", "a14", "a20", "a21", "a23", "a32", "a34", "a41", "a43", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        elseif (event == "failSENS_Ra") 
            CurStateFailRa = 2;
            elementsDFailRa = ["PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        end
        
    case 1,      
        if (event == "rstF_Ra")
            elementsDFailRa = ["rstF_Ra"];
        end
    
    case 2,       
        if (event == "rstF_Ra") 
            elementsDFailRa = ["rstF_Ra"];
        elseif (event == "failSENS_Ra") 
            CurStateFailRa = 1;
            elementsDFailRa = ["PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        end        
end
enableTransitions = setdiff(alltransitions, elementsDFailRa);       
refreshTransitionsRa(enableTransitions);        

function [] = FailRb(event)
global CurStateFailRb;
alltransitions = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
global elementsDFailRb;

switch (CurStateFailRb)
    case 0,
        if (event == "b02" || event == "b12" || event == "b14" || event ==  "b20" || event ==  "b21" ||  event ==  "b23" || event ==  "b32" || event ==  "b34" || event ==  "b41" || event == "b43" || event == "PatC1_Rb" || event == "PatC2_Rb" || event == "PatC3_Rb" || event == "PatC4_Rb" || event == "FPatC1_Rb" || event == "FPatC2_Rb" || event == "FPatC3_Rb" || event == "FPatC4_Rb") 
            CurStateFailRb = 0;
            elementsDFailRb = ["rstF_Rb"];       
        elseif (event == "failMOV_Rb") 
            CurStateFailRb = 1;
            elementsDFailRb = ["b02", "b12", "b14", "b20", "b21", "b23", "b32", "b34", "b41", "b43", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        elseif (event == "failSENS_Rb")
            CurStateFailRb = 2;
            elementsDFailRb = ["PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        end
        
    case 1,
        
        if (event == "rstF_Rb") 
            CurStateFailRb = 0;
            elementsDFailRb = ["rstF_Rb"];
        end
        
    case 2,
        
        if (event == "rstF_Rb") 
            elementsDFailRb = ["rstF_Rb"];
        elseif (event == "failSENS_Rb")
            CurStateFailRb = 1;
            elementsDFailRb = ["PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb"];
        end
end
enableTransitions = setdiff(alltransitions, elementsDFailRb);
refreshTransitionsRb(enableTransitions);        

function [] = BatRa(event)
global CurStateBatRa;
alltransitions = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];
global elementsDBatRa;

switch (CurStateBatRa)
    case 0,
        if (event == "warning_batRa") 
            CurStateBatRa = 1;
            elementsDBatRa = ["warning_batRa"];
        elseif (event == "low_batRa")
            CurStateBatRa = 2;
            elementsDBatRa = ["warning_batRa", "low_batRa"];
        end
        
    case 1,        
        if (event == "ok_batRa") 
            CurStateBatRa = 0;
            elementsDBatRa = [""];
        elseif (event == "low_batRa") 
            CurStateBatRa = 2;
            elementsDBatRa = ["warning_batRa", "low_batRa"];
        end
        
    case 2,
        
        if (event == "ok_batRa") 
            CurStateBatRa = 0;
            elementsDBatRa = [""];
        end
end
enableTransitions = setdiff(alltransitions, elementsDBatRa);       
refreshTransitionsRa(enableTransitions);        

function [] = BatRb(event)
global CurStateBatRb;
alltransitions = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
global elementsDBatRb;

switch (CurStateBatRb)
    case 0,
        if (event == "warning_batRb") 
            CurStateBatRb = 1;
            elementsDBatRb = ["warning_batRb"];
        elseif (event == "low_batRb") 
            CurStateBatRb = 2;
            elementsDBatRb = ["warning_batRb", "low_batRb"];
        end
        
    case 1,
        
        if (event == "ok_batRb") 
            CurStateBatRb = 0;
            elementsDBatRb = [""];
        elseif (event == "low_batRb")
            CurStateBatRb = 2;
            elementsDBatRb = ["warning_batRb", "low_batRb"];
        end
        
    case 2,
        
        if (event == "ok_batRb")
            CurStateBatRb = 0;
            elementsDBatRb = [""];
        end
end

enableTransitions = setdiff(alltransitions, elementsDBatRb);
refreshTransitionsRb(enableTransitions);        

function [] = geoA(event)
global CurStateGeoA;
alltransitions = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];
global elementsDgeoA;

switch (CurStateGeoA)
    case 0,
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa")
            CurStateGeoA = 0;
            elementsDgeoA = ["a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "PatC1_Ra") 
            CurStateGeoA = 5;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "a12") 
            CurStateGeoA = 1;
            elementsDgeoA = ["a12", "a14", "a32", "a34", "a41", "a43", "a02", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "a14")
            CurStateGeoA = 3;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra"];
        end
        
    case 1,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa") 
            CurStateGeoA = 1;
            elementsDgeoA = ["a12", "a14", "a32", "a34", "a41", "a43", "a02", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "PatC2_Ra") 
            CurStateGeoA = 6;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "a21") 
            CurStateGeoA = 0;
            elementsDgeoA = ["a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "a23") 
            CurStateGeoA = 2;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC4_Ra"];
        elseif (event == "a20" || event == "a20F")
            CurStateGeoA = 4;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a20", "a20F", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
        
    case 2,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa") 
            CurStateGeoA = 2;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC4_Ra"];
        elseif (event == "PatC3_Ra")
            CurStateGeoA = 7;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC2_Ra", "FPatC1_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
       elseif (event == "a32") 
            CurStateGeoA = 1;
            elementsDgeoA = ["a12", "a14", "a32", "a34", "a41", "a43", "a02", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC3_Ra", "PatC4_Ra"];
       elseif (event == "a34") 
            CurStateGeoA = 3;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a02", "a20", "a20F", "FPatC1_Ra", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra"];
        end
        
    case 3,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa")
            CurStateGeoA = 3;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a02", "a20", "a20F", "FPatC1_Ra", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra"];
       elseif (event == "PatC4_Ra")
            CurStateGeoA = 8;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC3_Ra", "FPatC2_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
       elseif (event == "a41") 
            CurStateGeoA = 0;
            elementsDgeoA = ["a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        elseif (event == "a43")
            CurStateGeoA = 2;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC4_Ra"];
        end
        
    case 4,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa" || event == "ok_batRa")
            CurStateGeoA = 4;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a41", "a43", "a20", "a20F", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
       elseif (event == "a02") 
            CurStateGeoA = 1;
            elementsDgeoA = ["a12", "a14", "a32", "a34", "a41", "a43", "a02", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
        
    case 5,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa" || event == "FPatC1_Ra") 
            CurStateGeoA = 0;
            elementsDgeoA = ["a21", "a23", "a32", "a34", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
        
    case 6,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa" || event == "FPatC2_Ra") 
            CurStateGeoA = 1;
            elementsDgeoA = ["a12", "a14", "a32", "a34", "a41", "a43", "a02", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
        
    case 7,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa" || event == "FPatC3_Ra")
            CurStateGeoA = 2;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a41", "a43", "a02", "a20", "a20F", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC4_Ra"];
        end
        
    case 8,
        
        if (event == "failMOV_Ra" || event == "failSENS_Ra" || event == "low_batRa" || event == "FPatC4_Ra")
            CurStateGeoA = 3;
            elementsDgeoA = ["a12", "a14", "a21", "a23", "a32", "a34", "a02", "a20", "a20F", "FPatC1_Ra", "ok_batRa", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra"];
        end
        
end
enableTransitions = setdiff(alltransitions, elementsDgeoA);
refreshTransitionsRa(enableTransitions);        

function [] = geoB(event)
global CurStateGeoB;
alltransitions = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
global elementsDgeoB;

switch (CurStateGeoB)
    case 0,
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb") 
            CurStateGeoB = 0;
            elementsDgeoB = ["b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "PatC1_Rb") 
            CurStateGeoB = 5;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b12") 
            CurStateGeoB = 1;
            elementsDgeoB = ["b12", "b14", "b32", "b34", "b41", "b43", "b02", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b14") 
            CurStateGeoB = 3;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb"];
        end
            
    case 1,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb") 
            CurStateGeoB = 1;
            elementsDgeoB = ["b12", "b14", "b32", "b34", "b41", "b43", "b02", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "PatC2_Rb") 
            CurStateGeoB = 6;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b21") 
            CurStateGeoB = 0;
            elementsDgeoB = ["b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];                    
        elseif (event == "b23") 
            CurStateGeoB = 2;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC4_Rb"];
        elseif (event == "b20" || event == "b20F") 
            CurStateGeoB = 4;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b20", "b20F", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        end
        
    case 2,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb")
            CurStateGeoB = 2;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC4_Rb"];
        elseif (event == "PatC3_Rb")
            CurStateGeoB = 7;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC2_Rb", "FPatC1_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b32") 
            CurStateGeoB = 1;
            elementsDgeoB = ["b12", "b14", "b32", "b34", "b41", "b43", "b02", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b34") 
            CurStateGeoB = 3;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b02", "b20", "b20F", "FPatC1_Rb", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb"];
       end
        
    case 3,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb")
            CurStateGeoB = 3;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b02", "b20", "b20F", "FPatC1_Rb", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb"];
        elseif (event == "PatC4_Rb") 
            CurStateGeoB = 8;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC3_Rb", "FPatC2_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b41")
            CurStateGeoB = 0;
            elementsDgeoB = ["b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b43") 
            CurStateGeoB = 2;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC4_Rb"];
        end
        
    case 4,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb" || event == "ok_batRb")
            CurStateGeoB = 4;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b41", "b43", "b20", "b20F", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        elseif (event == "b02") 
            CurStateGeoB = 2;
            elementsDgeoB = ["b12", "b14", "b32", "b34", "b41", "b43", "b02", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
        end
        
    case 5,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb" || event == "FPatC1_Rb") 
            CurStateGeoB = 0;
            elementsDgeoB = ["b21", "b23", "b32", "b34", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        end
        
    case 6,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb" || event == "FPatC2_Rb") 
            CurStateGeoB = 1;
            elementsDgeoB = ["b12", "b14", "b32", "b34", "b41", "b43", "b02", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC3_Rb", "PatC4_Rb"];
        end
        
    case 7,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb" || event == "FPatC3_Rb") 
            CurStateGeoB = 2;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b41", "b43", "b02", "b20", "b20F", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC4_Rb"];
        end
        
    case 8,
        
        if (event == "failMOV_Rb" || event == "failSENS_Rb" || event == "low_batRb" || event == "FPatC4_Rb")
            CurStateGeoB = 3;
            elementsDgeoB = ["b12", "b14", "b21", "b23", "b32", "b34", "b02", "b20", "b20F", "FPatC1_Rb", "ok_batRb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb"];
        end
        
end

enableTransitions = setdiff(alltransitions, elementsDgeoB);
refreshTransitionsRb(enableTransitions);        

%% Robots Finite State Machine Espeficications
%Sup 1
function [] = SUPFailRa(event)
global CurStateSUPFailRa;
global elementsDSupFailRa;

switch (CurStateSUPFailRa)
    case 0,
        if (event == "a02" || event == "a12" || event == "a14" || event ==  "a20" || event ==  "a21" ||  event ==  "a23" || event ==  "a32" || event ==  "a34" || event ==  "a41" || event == "a43" || event == "PatC1_Ra" || event == "PatC2_Ra" || event == "PatC3_Ra" || event == "PatC4_Ra")
            CurStateSUPFailRa = 0;
            elementsDSupFailRa = ["a20F", "rstF_Ra"];
        elseif (event == "failSENS_Ra" || event == "failMOV_Ra")
            CurStateSUPFailRa = 1;
            elementsDSupFailRa = ["a02", "a14", "a20", "a21", "a23", "a34", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
 
    case 1,
        
        if (event == "rstF_Ra")
            CurStateSUPFailRa = 0;
            elementsDSupFailRa = ["a20F", "rstF_Ra"];
        elseif (event == "failSENS_Ra" || event == "failMOV_Ra" || event == "a12" || event == "a20F" || event == "a32" || event == "a41" || event == "a43")
            CurStateSUPFailRa = 1;
            elementsDSupFailRa = ["a02", "a14", "a20", "a21", "a23", "a34", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra"];
        end
              
end

refreshTransitionsSupRa(elementsDSupFailRa)

function [] = SUPFailRb(event)
global CurStateSUPFailRb;
global elementsDSupFailRb;
switch (CurStateSUPFailRb)
    case 0,
        if (event == "b02" || event == "b12" || event == "b14" || event ==  "b20" || event ==  "b21" ||  event ==  "b23" || event ==  "b32" || event ==  "b34" || event ==  "b41" || event == "b43" || event == "PatC1_Rb" || event == "PatC2_Rb" || event == "PatC3_Rb" || event == "PatC4_Rb")
            CurStateSUPFailRb = 0;
            elementsDSupFailRb = ["b20F", "rstF_Rb"];
        elseif (event == "failSENS_Rb" || event == "failMOV_Rb")
            CurStateSUPFailRb = 1;
            elementsDSupFailRb = ["b02", "b14", "b20", "b21", "b23", "b34", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        end
        
    case 1,
        
        if (event == "rstF_Rb")
            CurStateSUPFailRb = 0;
            elementsDSupFailRb = ["b20F", "rstF_Rb"];
        elseif (event == "failSENS_Rb" || event == "failMOV_Rb" || event == "b12" || event == "b20F" || event == "b32" || event == "b41" || event == "b43")
            CurStateSUPFailRb = 1;
            elementsDSupFailRb = ["b02", "b14", "b20", "b21", "b23", "b34", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb"];
        end     
end

refreshTransitionsSupRb(elementsDSupFailRb)

function [] = SUPBatRa(event)
global elementsDSupBatRa;
global CurStateSUPBatRa;

switch (CurStateSUPBatRa)
    case 0,
        if (event == "a02" || event == "a12" || event == "a14" || event ==  "a21" ||  event ==  "a23" || event ==  "a32" || event ==  "a34" || event ==  "a41" || event == "a43" || event == "PatC1_Ra" || event == "PatC2_Ra" || event == "PatC3_Ra" || event == "PatC4_Ra")
            CurStateSUPBatRa = 0;
            elementsDSupBatRa = ["a20", "ok_batRa"];
        elseif (event == "low_batRa" || event == "warning_batRa")
            CurStateSUPBatRa = 1;
            elementsDSupBatRa = ["a02", "a14", "a21", "a23", "a34", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "warning_batRa"];
        end
        
    case 1,
        
        if (event == "ok_batRa")
            CurStateSUPBatRa = 0;
            elementsDSupBatRa = ["a20", "ok_batRa"];
        elseif (event == "low_batRa" || event == "a12" || event == "a20"|| event == "a32" || event == "a41" || event == "a43")
            CurStateSUPBatRa = 1;
            elementsDSupBatRa = ["a02", "a14", "a21", "a23", "a34", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra",  "warning_batRa"];
        end
        
        
end

refreshTransitionsSupRa(elementsDSupBatRa)

function [] = SUPBatRb(event)
global CurStateSUPBatRb;
global elementsDSupBatRb;

switch (CurStateSUPBatRb)
    case 0,
        if (event == "b02" || event == "b12" || event == "b14" || event ==  "b21" ||  event ==  "b23" || event ==  "b32" || event ==  "b34" || event ==  "b41" || event == "b43" || event == "PatC1_Rb" || event == "PatC2_Rb" || event == "PatC3_Rb" || event == "PatC4_Rb")
            CurStateSUPBatRb = 0;
            elementsDSupBatRb = ["b20", "ok_batRb"];
        elseif (event == "low_batRb" || event == "warning_batRb")
            CurStateSUPBatRb = 1;
            elementsDSupBatRb = ["b02", "b14", "b21", "b23", "b34", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "warning_batRb"];
        end
        
    case 1,
        
        if (event == "ok_batRb")
            CurStateSUPBatRb = 0;
            elementsDSupBatRb = ["b20", "ok_batRb"];
        elseif (event == "low_batRb" || event == "b12" || event == "b20"|| event == "b32" || event == "b41" || event == "b43") 
            CurStateSUPBatRb = 1;
            elementsDSupBatRb = ["b02", "b14", "b21", "b23", "b34", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb",  "warning_batRb"];
        end
          
end

refreshTransitionsSupRb(elementsDSupBatRb)

function [] = SUPReturnC1_Ra(event)
global CurStateReturnC1_Ra;
global elementsDReturnC1_Ra;

switch (CurStateReturnC1_Ra)
    case 0,
        if (event ==  "a21" ||  event ==  "a41" || event == "FPatC2_Ra" || event == "FPatC3_Ra" || event == "FPatC4_Ra" || event == "low_batRa" || event == "failMOV_Ra")
            CurStateReturnC1_Ra = 0;
            elementsDReturnC1_Ra = [""];
        elseif (event == "a12" || event == "a14")
            CurStateReturnC1_Ra = 1;
            elementsDReturnC1_Ra = ["a21", "a41", "a12", "a14"];
        end
        
    case 1,
        
        if (event == "FPatC2_Ra" || event == "FPatC3_Ra" || event == "FPatC4_Ra" || event == "failMOV_Ra" || event == "low_batRa")
            CurStateReturnC1_Ra = 0;
            elementsDReturnC1_Ra = [""];
        end
end   

refreshTransitionsSupRb(elementsDReturnC1_Ra)       

function [] = SUPReturnC1_Rb(event)
global CurStateReturnC1_Rb;
global elementsDReturnC1_Rb;

switch (CurStateReturnC1_Rb)
    case 0,
        if (event ==  "b21" ||  event ==  "b41" || event == "FPatC2_Rb" || event == "FPatC3_Rb" || event == "FPatC4_Rb" || event == "low_batRb" || event == "failMOV_Rb") 
            CurStateReturnC1_Rb = 0;
            elementsDReturnC1_Rb = [""];
        elseif (event == "b12" || event == "b14" ) 
            CurStateReturnC1_Rb = 1;
            elementsDReturnC1_Rb = ["b21", "b41", "b12", "b14"];
        end
        
    case 1,
        
        if (event == "FPatC2_Rb" || event == "FPatC3_Rb" || event == "FPatC4_Rb" || event == "failMOV_Rb" || event == "low_batRb")
            CurStateReturnC1_Rb = 0;
            elementsDReturnC1_Rb = [""];
        end
end  

refreshTransitionsSupRb(elementsDReturnC1_Rb)              

function [] = SUPReturnC2_Ra(event)
global CurStateReturnC2_Ra;
global elementsDReturnC2_Ra;

switch (CurStateReturnC2_Ra)
    case 0,
        if (event ==  "a32" ||  event ==  "a12" || event == "FPatC1_Ra" || event == "FPatC3_Ra" || event == "FPatC4_Ra" || event == "low_batRa" || event == "failMOV_Ra") 
            CurStateReturnC2_Ra = 0;
            elementsDReturnC2_Ra = [""];
        elseif (event == "a23" || event == "a21" ) 
            CurStateReturnC2_Ra = 1;
            elementsDReturnC2_Ra = ["a21", "a23", "a12", "a32"];
        end
        
    case 1,
        
        if (event == "FPatC1_Ra" || event == "FPatC3_Ra" || event == "FPatC4_Ra" || event == "failMOV_Ra" || event == "low_batRa")
            CurStateReturnC2_Ra = 0;
            elementsDReturnC2_Ra = [""];
        end
end 

refreshTransitionsSupRb(elementsDReturnC2_Ra)              
        
function [] = SUPReturnC2_Rb(event)
global CurStateReturnC2_Rb;
global elementsDReturnC2_Rb;

switch (CurStateReturnC2_Rb)
    case 0,
        if (event ==  "b32" ||  event ==  "b12" || event == "FPatC1_Rb" || event == "FPatC3_Rb" || event == "FPatC4_Rb" || event == "low_batRb" || event == "failMOV_Rb")
            CurStateReturnC2_Rb = 0;
            elementsDReturnC2_Rb = [""];
        elseif (event == "b23" || event == "b21") 
            CurStateReturnC2_Rb = 1;
            elementsDReturnC2_Rb = ["b21", "b23", "b12", "b32"];
        end
        
    case 1,
        
        if (event == "FPatC1_Rb" || event == "FPatC3_Rb" || event == "FPatC4_Rb" || event == "failMOV_Rb" || event == "low_batRb")
            CurStateReturnC2_Rb = 0;
            elementsDReturnC2_Rb = [""];
        end
end 

refreshTransitionsSupRb(elementsDReturnC2_Rb)                      

function [] = SUPReturnC3_Ra(event)
global CurStateReturnC3_Ra;
global elementsDReturnC3_Ra;

switch (CurStateReturnC3_Ra)
    case 0,
        if (event ==  "a23" ||  event ==  "a43" || event == "FPatC1_Ra" || event == "FPatC2_Ra" || event == "FPatC4_Ra" || event == "low_batRa" || event == "failMOV_Ra") 
            CurStateReturnC3_Ra = 0;
            elementsDReturnC3_Ra = [""];
        elseif (event == "a32" || event == "a34" ) 
            CurStateReturnC3_Ra = 1;
            elementsDReturnC3_Ra = ["a32", "a34", "a23", "a43"];
        end
        
    case 1,
        
        if (event == "FPatC1_Ra" || event == "FPatC2_Ra" || event == "FPatC4_Ra" || event == "failMOV_Ra" || event == "low_batRa")
            CurStateReturnC3_Ra = 0;
            elementsDReturnC3_Ra = [""];
        end
end   

refreshTransitionsSupRb(elementsDReturnC3_Ra)                              

function [] = SUPReturnC3_Rb(event)
global CurStateReturnC3_Rb;
global elementsDReturnC3_Rb; 

switch (CurStateReturnC3_Rb)
    case 0,
        if (event ==  "b23" ||  event ==  "b43" || event == "FPatC1_Rb" || event == "FPatC2_Rb" || event == "FPatC4_Rb" || event == "low_batRb" || event == "failMOV_Rb")
            CurStateReturnC3_Rb = 0;
            elementsDReturnC3_Rb = [""];
        elseif (event == "b32" || event == "b34" ) 
            CurStateReturnC3_Rb = 1;
            elementsDReturnC3_Rb = ["b32", "b34", "b23", "b43"];
        end
        
    case 1,
        
        if (event == "FPatC1_Rb" || event == "FPatC2_Rb" || event == "FPatC4_Rb" || event == "failMOV_Rb" || event == "low_batRb")
            CurStateReturnC3_Rb = 0;
            elementsDReturnC3_Rb = [""];
        end
end 

refreshTransitionsSupRb(elementsDReturnC3_Rb)                                      

function [] = SUPReturnC4_Ra(event)
global CurStateReturnC4_Ra;
global elementsDReturnC4_Ra;

switch (CurStateReturnC4_Ra)
    case 0,
        if (event ==  "a14" ||  event ==  "a34" || event == "FPatC1_Ra" || event == "FPatC2_Ra" || event == "FPatC3_Ra" || event == "low_batRa" || event == "failMOV_Ra")
            CurStateReturnC4_Ra = 0;
            elementsDReturnC4_Ra = [""];
        elseif (event == "a41" || event == "a43" ) 
            CurStateReturnC4_Ra = 1;
            elementsDReturnC4_Ra = ["a41", "a43", "a14", "a34"];
        end
        
    case 1,
        
        if (event == "FPatC1_Ra" || event == "FPatC2_Ra" || event == "FPatC3_Ra" || event == "failMOV_Ra" || event == "low_batRa")
            CurStateReturnC4_Ra = 0;
            elementsDReturnC4_Ra = [""];
        end
end  

refreshTransitionsSupRb(elementsDReturnC4_Ra)                                              

function [] = SUPReturnC4_Rb(event)
global CurStateReturnC4_Rb;
global elementsDReturnC4_Rb;

switch (CurStateReturnC4_Rb)
    case 0,
        if (event ==  "b14" ||  event ==  "b34" || event == "FPatC1_Rb" || event == "FPatC2_Rb" || event == "FPatC3_Rb" || event == "low_batRb" || event == "failMOV_Rb") 
            CurStateReturnC4_Rb = 0;
            elementsDReturnC4_Rb = [""];
        elseif (event == "b41" || event == "b43" )
            CurStateReturnC4_Rb = 1;
            elementsDReturnC4_Rb = ["b41", "b43", "b14", "b34"];
        end
        
    case 1,
        
        if (event == "FPatC1_Rb" || event == "FPatC2_Rb" || event == "FPatC3_Rb" || event == "failMOV_Rb" || event == "low_batRb")
            CurStateReturnC4_Rb = 0;
            elementsDReturnC4_Rb = [""];
        end
end        
refreshTransitionsSupRb(elementsDReturnC4_Rb)                                             


%% Refresh the transitions for the robots
function [] = refreshTransitionsSupRa(x)
global disableTransitionsRa;
global enableTransitionsRa;

disableTransitionsRa = [disableTransitionsRa, x];
enableTransitionsRa = setdiff(enableTransitionsRa, disableTransitionsRa);

function [] = refreshTransitionsSupRb(x)
global disableTransitionsRb;
global enableTransitionsRb;

disableTransitionsRb = [disableTransitionsRb, x];
enableTransitionsRb = setdiff(enableTransitionsRb, disableTransitionsRb);

% Function to refresh the disable and enable transitions
function [] = refreshTransitionsRa(x)
alltransitions = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];
global disableTransitionsRa;
global enableTransitionsRa;
global count;
if count == 1
    enableTransitionsRa = x;
    count = count + 1;
else 
    enableTransitionsRa = intersect(enableTransitionsRa, x);
    count = count + 1;
    if count == 3
        count = 0;
    end 
end 
disableTransitionsRa = setdiff(alltransitions, enableTransitionsRa);

function [] = refreshTransitionsRb(x)
alltransitions = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
global disableTransitionsRb;
global enableTransitionsRb;
global count;
if count == 1
    enableTransitionsRb = x;
    count = count + 1;
else 
    enableTransitionsRb = intersect(enableTransitionsRb, x);
    count = count + 1;
    if count == 3
        count = 0;
    end 
end 
disableTransitionsRb = setdiff(alltransitions, enableTransitionsRb);

%% Function to determite the robot position and transition
function [a, b] = position(handles, PosX, PosY, idx, c)
global list;
global batSituationRa;
global batSituationRb;
global failSENS_Ra;
global failSENS_Rb;

b = 0;
a = 0;

if idx == 1
    cR1 = c;
    if (PosX < 0.375) && (PosY < 0.375)
        aR1 = ['Robot ', num2str(idx) ,' cel 1'];
        set(handles.text1,'String', aR1);
        bR1 = 1;
    elseif (PosX < 0.375) && (PosY > 0.375)
        aR1 = ['Robot ', num2str(idx) ,' cel 2'];
        set(handles.text1,'String', aR1);
        bR1 = 2;
    elseif (PosX > 0.375) && (PosY > 0.375)
        aR1 = ['Robot ', num2str(idx) ,' cel 3'];
        set(handles.text1,'String', aR1);
        bR1 = 3;
    elseif (PosX > 0.375) && (PosY < 0.375)
        aR1 = ['Robot ', num2str(idx) ,' cel 4'];
        set(handles.text1,'String', aR1);
        bR1 = 4;
    else
        aR1 = ['Robot ', num2str(idx) ,' estation'];
        set(handles.text1,'String', aR1);
        bR1 = 0;
    end
    if bR1 ~= cR1
        a = 1;
        disp('fire one transiton Robot 1')
        if bR1 == 1 && cR1 == 4
            list = [list, "a41"];
        elseif bR1 == 1 && cR1 == 2
            list = [list, "a21"];
        elseif bR1 == 2 && cR1 == 1
            list = [list, "a12"];
        elseif bR1 == 2 && cR1 == 3
            list = [list, "a32"];
        elseif bR1 == 3 && cR1 == 2
            list = [list, "a23"];
        elseif bR1 == 3 && cR1 == 4
            list = [list, "a43"];
        elseif bR1 == 4 && cR1 == 1
            list = [list, "a14"];
        elseif bR1 == 4 && cR1 == 3
            list = [list, "a34"];
        elseif bR1 == 0 && cR1 == 2 && (batSituationRa == 1 || batSituationRa == 2)
            list = [list, "a20"];
        elseif bR1 == 0 && cR1 == 2 && (failSENS_Ra == 1)
            list = [list, "a20F"];
        elseif bR1 == 2 && cR1 == 0
            list = [list, "a02"];
        end
    end
    b = bR1;
    a = aR1;
elseif idx == 2
    cR2 = c;
    if (PosX < 0.375) && (PosY < 0.375)
        aR2 = ['Robot ', num2str(idx) ,' cel 1'];
        set(handles.text11,'String', aR2);
        bR2 = 1;
    elseif (PosX < 0.375) && (PosY > 0.375)
        aR2 = ['Robot ', num2str(idx) ,' cel 2'];
        set(handles.text11,'String', aR2);
        bR2 = 2;
    elseif (PosX > 0.375) && (PosY > 0.375)
        aR2 = ['Robot ', num2str(idx) ,' cel 3'];
        set(handles.text11,'String', aR2);
        bR2 = 3;
    elseif (PosX > 0.375) && (PosY < 0.375)
        aR2 = ['Robot ', num2str(idx) ,' cel 4'];
        set(handles.text11,'String', aR2);
        bR2 = 4;
    else
        aR2 = ['Robot ', num2str(idx) ,' estation'];
        set(handles.text11,'String', aR2);
        bR2 = 0;
    end
    if bR2 ~= cR2
        a = 1;
        disp('fire one transiton Robot 2')
        if bR2 == 1 && cR2 == 4
            list = [list, "b41"];
        elseif bR2 == 1 && cR2 == 2
            list = [list, "b21"];
        elseif bR2 == 2 && cR2 == 1
            list = [list, "b12"];
        elseif bR2 == 2 && cR2 == 3
            list = [list, "b32"];
        elseif bR2 == 3 && cR2 == 2
            list = [list, "b23"];
        elseif bR2 == 3 && cR2 == 4
            list = [list, "b43"];
        elseif bR2 == 4 && cR2 == 1
            list = [list, "b14"];
        elseif bR2 == 4 && cR2 == 3
            list = [list, "b34"];
        elseif bR2 == 0 && cR2 == 2 && (batSituationRb == 1 || batSituationRb == 2)
            list = [list, "b20"];
        elseif bR2 == 0 && cR2 == 2 && (failSENS_Rb == 1)
            list = [list, "b20F"];
        elseif bR2 == 2 && cR2 == 0
            list = [list, "b02"];
        end
    end
    b = bR2;
    a = aR2;
end


%% Get the postion and orientation for the each robot
function [x,y,theta] = getPosition(idx)
global vrep;
global clientID;
global kilohandles;
[returnCode,position]=vrep.simxGetObjectPosition(clientID,kilohandles(idx),-1,vrep.simx_opmode_buffer);
[returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,kilohandles(idx),-1,vrep.simx_opmode_buffer);

x =  position(1);
y = position(2);
theta = orientation(3);


%% Function to count the failure time
function [] = faultRobots(idx)
global failMOV_Ra;
global failMOV_Rb;
global countR1;
global countR2

if (failMOV_Ra == 1) && (idx == 1)
    countR1 = countR1 + 1;
    if countR1 == 20
        disp("timeoutRa");
    end
elseif (failMOV_Rb == 1) && (idx == 2)
    countR2 = countR2 + 1;
    if countR2 == 20
        disp("timeoutRb");
    end
end

%% Set the velocity for each robot
function [] = setVelocity(idx, VL, VR)
global vrep;
global clientID;
global left_Motor;
global right_Motor;
global left_Motor2;
global right_Motor2;
global failMOV_Ra;
global failMOV_Rb;
global robotColision;

faultRobots(idx);

if (idx == 1) && (failMOV_Ra == 0) && (robotColision(idx) == 0)
    vrep.simxSetJointTargetVelocity(clientID,left_Motor,VL,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor,VR,vrep.simx_opmode_blocking);
elseif (idx == 2) && (failMOV_Rb == 0) && (robotColision(idx) == 0)
    vrep.simxSetJointTargetVelocity(clientID,left_Motor2,VL,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor2,VR,vrep.simx_opmode_blocking);
elseif (idx == 1) && (failMOV_Ra == 1)
    vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
elseif (idx == 2) && (failMOV_Rb == 1)
    vrep.simxSetJointTargetVelocity(clientID,left_Motor2,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor2,0,vrep.simx_opmode_blocking);
end
pause(0.05)

%% Calculate the new velocity according to the delta error
function [nvelL, nvelR]  = computeVelocity(x, y, theta, velL, velR, tx, ty,thetaT)
deltatheta = thetaT - theta;
if x > tx
    if theta < thetaT
        nvelL = deltatheta*velL;
        nvelR = velR;
    elseif theta > thetaT
        nvelR = deltatheta*velR;
        nvelL = velL;
    end
elseif x < tx
    if theta > thetaT
        nvelL = deltatheta*velL;
        nvelR = velR;
    elseif theta < thetaT
        nvelR = deltatheta*velR;
        nvelL = velL;
    end
end

%% Update the robot variables
function [] = updateRobot(idx, tx, ty, thetaT, VR, VL)
[x, y, theta] = getPosition(idx);
[nVL, nVR]  = computeVelocity(x, y, theta, VL, VR, tx, ty, thetaT);
setVelocity(idx, nVL, nVR);

%% First rotation to the set point to R1
function [] = rot (theta, thetaT, idx)
global fase;
if (theta > (thetaT + 0.2)) || (theta < (thetaT - 0.2))
    setVelocity(idx, 0, 4);
else
    TEXT = ['inicial position R', num2str(idx)];
    disp(TEXT);
    fase(idx) = 2;
    setVelocity(idx, 0, 0);
end

%% Compute the axes point to x and y
function [tx, ty] = sortPoint(idx, tx, ty)
global Target;
global PatrollingRa;
global PatrollingRb;
global PatruCellRa;
global PatruCellRb;

target = 0;

if (idx == 1)
    if (PatrollingRa == 0)
        if (Target(idx) == "a21" || Target(idx) == "a41")
            target = 1;
        elseif(Target(idx) == "a12" || Target(idx) == "a32")
            target = 2;
        elseif(Target(idx) == "a23" || Target(idx) == "a43")
            target = 3;
        elseif(Target(idx) == "a14" || Target(idx) == "a34")
            target = 4;
        elseif(Target(idx) == "a20" || Target(idx) == "a20F")
            target = 5; 
        end
        switch (target)
            case 1,
                tx = 0.2;
                ty = 0.2;
            case 2,
                tx = 0.2;
                ty = 0.55;
            case 3,
                tx = 0.55;
                ty = 0.55;
            case 4,
                tx = 0.55;
                ty = 0.2;
            case 5,
                tx = -0.075;
                ty = 0.6;
        end
    end
    if PatrollingRa == 1
        if PatruCellRa == 1
            x = 0.2;
            y = 0.2;
        elseif PatruCellRa == 2
            x = 0.2;
            y = 0.55;
        elseif PatruCellRa == 3
            x = 0.55;
            y = 0.55;
        elseif PatruCellRa == 4
            x = 0.55;
            y = 0.2;
        end
        if Target(idx) == num2str(1)
            tx = x - 0.15;
            ty = y - 0.15;
        elseif Target(idx) == num2str(2)
            tx = x - 0.15;
            ty = y + 0.15;
        elseif Target(idx) == num2str(3)
            tx = x + 0.15;
            ty = y + 0.15;
        elseif Target(idx) == num2str(4)
            tx = x + 0.15;
            ty = y - 0.15;
        end
    end
end



if idx == 2
    if (PatrollingRb == 0)
        if (Target(idx) == "b21" || Target(idx) == "b41")
            target = 1;
        elseif(Target(idx) == "b12" || Target(idx) == "b32")
            target = 2;
        elseif(Target(idx) == "b23" || Target(idx) == "b43")
            target = 3;
        elseif(Target(idx) == "b14" || Target(idx) == "b34")
            target = 4;
        elseif(Target(idx) == "b20" || Target(idx) == "b20F")
            target = 5;
        end
        switch (target)
            case 1,
                tx = 0.2;
                ty = 0.2;
            case 2,
                tx = 0.2;
                ty = 0.55;
            case 3,
                tx = 0.55;
                ty = 0.55;
            case 4,
                tx = 0.55;
                ty = 0.2;
            case 5,
                tx = -0.075;
                ty = 0.6;
        end
    end
    if (PatrollingRb == 1)
        if PatruCellRb == 1
            x = 0.2;
            y = 0.2;
        elseif PatruCellRb == 2
            x = 0.2;
            y = 0.55;
        elseif PatruCellRb == 3
            x = 0.55;
            y = 0.55;
        elseif PatruCellRb == 4
            x = 0.55;
            y = 0.2;
        end
        if Target(idx) == num2str(1)
            tx = x - 0.15;
            ty = y - 0.15;
        elseif Target(idx) == num2str(2)
            tx = x - 0.15;
            ty = y + 0.15;
        elseif Target(idx) == num2str(3)
            tx = x + 0.15;
            ty = y + 0.15;
        elseif Target(idx) == num2str(4)
            tx = x + 0.15;
            ty = y - 0.15;
        end
    end
end


TEXT = ['the target robot ', num2str(idx) ,'  is tx: ', num2str(tx), ' and ty: ', num2str(ty)];
disp(TEXT);

%% Theta calculation
function [thetaT] = SetOrientation(idx, tx, ty, thetaT)
global vrep;
global clientID;
global kilohandles;

[returnCode,position]=vrep.simxGetObjectPosition(clientID,kilohandles(idx),-1,vrep.simx_opmode_buffer);
x = position(1);
y = position(2);

deltaX = tx - x;

deltaY = ty - y;

thetaT = atan(deltaY/deltaX);

[returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,kilohandles(idx),-1,vrep.simx_opmode_buffer);

theta = orientation(3);

rot(theta, thetaT, idx);

%% Check if the robot acheived their target
function [achieved] = achivedTarget(idx, tx, ty, x, y)
global fase;
global achievedPoint;
global PatrolAchieved;
global PatrollingRb;
global PatrollingRa;

if (x < (tx+0.07)) && (x > (tx-0.07)) && (y < (ty+0.07)) && (y > (ty-0.07))
    achieved = 1;
    fase(idx) = 0;
    achievedPoint(idx) = 1;
    TEXT = ['robot ', num2str(idx) ,'  achieved the target'];
    disp(TEXT);
    if PatrollingRb == 1 && idx == 2
        PatrolAchieved(idx) = 1;
    elseif PatrollingRa == 1 && idx == 1
        PatrolAchieved(idx) = 1;
    end
    
else
    achieved = 0;
    achievedPoint(idx) = 0;
    
    if PatrollingRb == 1 && idx == 2
        PatrolAchieved(idx) = 0;
    elseif PatrollingRa == 1 && idx == 1
        PatrolAchieved(idx) = 0;
    end
    
end

%% Setting the velocity according to position and if the robot achieved the target point
function [velR, velL] = settingInitialVelocity(idx, tx, ty, x, y)
[achieved] = achivedTarget(idx, tx, ty, x, y);

if (achieved == 1)
    velR = 0;
    velL = 0;
elseif (achieved == 0) 
    if  (x < tx)
        velR = - 7;
        velL = - 7;
    elseif (x > tx)
        velR = 7;
        velL = 7;
    end
end

%% change the states according to the events that accur to Rb
function [] = ChangeStateMachineRb(handles)
global list;
global disableTransitionsRb;
global enableTransitionsRb;

alltransitions = ["b02", "b12", "b14", "b20", "b20F", "b21", "b23", "b32", "b34", "b41", "b43", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Rb", "FPatC2_Rb", "FPatC3_Rb", "FPatC4_Rb", "low_batRb", "ok_batRb", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "rstF_Rb", "warning_batRb"];
event = (list(end));
if ismember(event, alltransitions)
    BatRb(event);
    FailRb(event);
    geoB(event);
    SUPFailRb(event);
    SUPBatRb(event);
    SUPReturnC1_Rb(event)
    SUPReturnC2_Rb(event)
    SUPReturnC3_Rb(event)
    SUPReturnC4_Rb(event)
end

set(handles.text18,'String', disableTransitionsRb);
set(handles.text19,'String', enableTransitionsRb);

%% change the states according to the events that accur to Rb
function [] = ChangeStateMachineRa(handles)
global list;
global disableTransitionsRa;
global enableTransitionsRa;
alltransitions = ["a02", "a12", "a14", "a20", "a20F", "a21", "a23", "a32", "a34", "a41", "a43", "failMOV_Ra", "failSENS_Ra", "FPatC1_Ra", "FPatC2_Ra", "FPatC3_Ra", "FPatC4_Ra", "low_batRa", "ok_batRa", "PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "rstF_Ra", "warning_batRa"];
event = (list(end));
if ismember(event, alltransitions) 
    BatRa(event);
    FailRa(event);
    geoA(event);
    SUPFailRa(event)
    SUPBatRa(event)
    SUPReturnC1_Ra(event)
    SUPReturnC2_Ra(event)
    SUPReturnC3_Ra(event)
    SUPReturnC4_Ra(event)
end

set(handles.text14,'String', disableTransitionsRa);
set(handles.text15,'String', enableTransitionsRa);

%% Heuristic to implement the robot patrolling in the best place
function[] =  TARGETPatrolling(idx, b)
global PatruCellRa;
global PatruCellRb;
global batSituationRb;
global batSituationRa;
global failSENS_Rb;
global failSENS_Ra;
global idlenessRa;
global idlenessRb;

if idx == 1 && PatruCellRa == 0 && (batSituationRa == 3) && (failSENS_Ra == 0)
    if b == 1
        W = [1 3 2 4];
    elseif b == 2
        W = [4 1 2 3];
    elseif b == 3
        W = [3 2 1 4];
    elseif b == 4
        W = [4 2 3 1];
    elseif b == 0 
        W = [4 3 2 3];
    end    
    idlenessResultRa =  idlenessRa .* W;
    [M,I] = max(idlenessResultRa);
    if I == 1
        PatruCellRa = 1;
    elseif I == 2
        PatruCellRa = 2;
    elseif I == 3
        PatruCellRa = 3;
    elseif I == 4
        PatruCellRa = 4;
    end
    disp("Patrolling Ra cel")
    disp(PatruCellRa);
end

if idx == 2 && PatruCellRb == 0 && (batSituationRb == 3) && (failSENS_Rb == 0)
    if b == 1
        W = [1 4 2 3];
    elseif b == 2
        W = [3 1 4 2];
    elseif b == 3
        W = [2 4 1 3];
    elseif b == 4
        W = [3 2 4 1];
    elseif b == 0 
        W = [4 3 2 3];
    end
    idlenessResultRb = idlenessRb .* W;
    [M,I] = max(idlenessResultRb);
    if I == 1
        PatruCellRb = 1;
    elseif I == 2
        PatruCellRb = 2;
    elseif I == 3
        PatruCellRb = 3;
    elseif I == 4
        PatruCellRb = 4;
    end
    disp("Patrolling Rb cel")
    disp(PatruCellRb); 
end

%% Disable the events that dont make the robot to achieve the target
function [] = disableEventsTarget(idx)
global DisableRa;
global DisableRb;
global PatruCellRa;
global PatruCellRb;
global batSituationRa;   
global batSituationRb;
global failSENS_Ra;
global failSENS_Rb;

if idx == 1
    if PatruCellRa == 1
        DisableRa = ["a12", "a14", "a23", "a43"];
    elseif PatruCellRa == 2
        DisableRa = ["a23", "a14", "a34", "a21"];
    elseif PatruCellRa == 3
        DisableRa = ["a32", "a34", "a21", "a41"];
    elseif PatruCellRa == 4
        DisableRa = ["a41", "a43", "a32", "a12"];
    elseif PatruCellRa == 0 && ((batSituationRa ~= 3) || (failSENS_Ra == 1))
         DisableRa = ["a23", "a14", "a21"];
    else
        DisableRa = [""];
    end
end

if idx == 2
    if PatruCellRb == 1
        DisableRb = ["b12", "b14", "b23", "b43"];
    elseif PatruCellRb == 2
        DisableRb = ["b23", "b14", "b34", "b21"];
    elseif PatruCellRb == 3
        DisableRb = ["b32", "b34", "b21", "b41"];
    elseif PatruCellRb == 4
        DisableRb = ["b41", "b43", "b32", "b12"];
    elseif PatruCellRb == 0 && ((batSituationRb ~= 3) || (failSENS_Rb == 1))
        DisableRb = ["b23", "b14", "b21"];
    else
        DisableRb = [""];        
    end
end

%% When one robot is near another one and it's patrolling, the other one can perceive this
function [detectionState, b] = findAnotherRobotPatrolling(idx)
global PatrollingRa;
global PatrollingRb;
[x1, y1, theta1] = getPosition(1);
[x2, y2, theta2] = getPosition(2);
b = 0;        
if idx == 1
    if (x2 < x1 + 0.3 && x2 > x1 - 0.3) && (y2 < y1 + 0.3 && y2 > y1 - 0.3) && (PatrollingRb == 1)
        detectionState = 1;
        if (x2 < 0.375) && (y2 < 0.375)
            b = 1;
        elseif (x2 < 0.375) && (y2 > 0.375)
            b = 2;
        elseif (x2 > 0.375) && (y2 > 0.375)
            b = 3;
        elseif (x2 > 0.375) && (y2 < 0.375)
            b = 4;
        else
            b = 0;
        end 
    else 
        detectionState = 0;
    end 
end 
if idx == 2
    if (x1 < x2 + 0.3 && x1 > x2 - 0.3) && (y1 < y2 + 0.3 && y1 > y2 - 0.3) && (PatrollingRa == 1)
        detectionState = 1;
         if (x1 < 0.375) && (y1 < 0.375)
            b = 1;
        elseif (x1 < 0.375) && (y1 > 0.375)
            b = 2;
        elseif (x1 > 0.375) && (y1 > 0.375)
            b = 3;
        elseif (x1 > 0.375) && (y1 < 0.375)
            b = 4;
        else
            b = 0;
        end 
    else
        detectionState = 0;
    end 
end 

%% Function to execute when the robots are patrolling 
function [] = StatusPatrol(idx, b, handles)
global PatrollingRa;
global PatrollingRb;
global PatruCellRa;
global PatruCellRb;
global Target;
global countPointRa;
global countPointRb;
pointstopatrol = [1 2 3 4];
pointstopatrol2 = [4 3 2 1];
global list;
global achievedPoint;
global randomPointRa;
global randomPointRb;
global PatrolAchieved;
global idlenessRa;
global idlenessRb;

if idx == 1
    if countPointRa == 0
        PatrollingRa = 1;
        disp("Ra is patrolling");
        randomPointRa = randi([1, 2]);
        TEXT = strcat("PatC", num2str(b) ,"_Ra");
        list = [list, TEXT];
        countPointRa = countPointRa + 1;
        count = countPointRa;
        target = pointstopatrol(count);
        Target(idx) = num2str(target)
    end
    if (randomPointRa == 1) && (PatrolAchieved(idx) == 1)
        countPointRa = countPointRa + 1
        if (countPointRa < 5)
            count = countPointRa;
            target = pointstopatrol(count);
            Target(idx) = num2str(target)
            PatrolAchieved(idx) = 0;
            idlenessRa(1)  = toc + idlenessRa(1);
            idlenessRa(2)  = toc + idlenessRa(2);
            idlenessRa(3)  = toc + idlenessRa(3);
            idlenessRa(4)  = toc + idlenessRa(4);
            idlenessRa(b)  = 0;
            tic
        end
    elseif (randomPointRa == 2) && (PatrolAchieved(idx) == 1)
        countPointRa = countPointRa + 1;
        if (countPointRa < 5)
            count = countPointRa;
            target = pointstopatrol2(count);
            Target(idx) = num2str(target);
            PatrolAchieved(idx) = 0;
            idlenessRa(1)  = toc + idlenessRa(1);
            idlenessRa(2)  = toc + idlenessRa(2);
            idlenessRa(3)  = toc + idlenessRa(3);
            idlenessRa(4)  = toc + idlenessRa(4);
            idlenessRa(b)  = 0;
            tic
        end       
    end
    if (countPointRa > 5)
        idlenessRa(b) = 0;
        PatrollingRa = 0;
        countPointRa = 0;
        TEXT = strcat("FPatC", num2str(b) ,"_Ra");
        randomPointRa = 0;
        PatrolAchieved(idx) = 0;
        list = [list, TEXT];
        PatruCellRa = 0;
        achievedPoint(idx) = 0;
        ChangeStateMachineRa(handles);
        idlenessRa(1)  = toc + idlenessRa(1);
        idlenessRa(2)  = toc + idlenessRa(2);
        idlenessRa(3)  = toc + idlenessRa(3);
        idlenessRa(4)  = toc + idlenessRa(4);
        idlenessRa(b)  = 0;
        tic
    end
end


if idx == 2
    if countPointRb == 0
        disp("Rb is patrolling");
        PatrollingRb = 1;
        randomPointRb = randi([1, 2]);
        TEXT = strcat("PatC", num2str(b) ,"_Rb");
        list = [list, TEXT];
        countPointRb = countPointRb + 1;
        count = countPointRb;
        target = pointstopatrol(count);
        Target(idx) = num2str(target);
    end
    if (randomPointRb == 1) && (PatrolAchieved(idx) == 1)
        countPointRb = countPointRb + 1;
        if (countPointRb < 5)
            count = countPointRb;
            target = pointstopatrol(count);
            Target(idx) = num2str(target)
            PatrolAchieved(idx) = 0;
            idlenessRb(1)  = toc + idlenessRb(1);
            idlenessRb(2)  = toc + idlenessRb(2);
            idlenessRb(3)  = toc + idlenessRb(3);
            idlenessRb(4)  = toc + idlenessRb(4);
            idlenessRb(b)  = 0;
        tic
        end
    elseif (randomPointRb == 2) && (PatrolAchieved(idx) == 1)
        countPointRb = countPointRb + 1;
        if (countPointRb < 5)
            count = countPointRb;
            target = pointstopatrol(count);
            Target(idx) = num2str(target);
            PatrolAchieved(idx) = 0;
            idlenessRb(1)  = toc + idlenessRb(1);
            idlenessRb(2)  = toc + idlenessRb(2);
            idlenessRb(3)  = toc + idlenessRb(3);
            idlenessRb(4)  = toc + idlenessRb(4);
            idlenessRb(b)  = 0;
        tic
        end
    end
    if (countPointRb > 5)
        idlenessRb(b) = 0;
        PatrollingRb = 0;
        countPointRb = 0;
        TEXT = strcat("FPatC", num2str(b) ,"_Rb");
        randomPointRb = 0;
        list = [list, TEXT];
        PatruCellRb = 0;
        achievedPoint(idx) = 0;
        PatrolAchieved(idx) = 0;
        ChangeStateMachineRb(handles);
        idlenessRb(1)  = toc + idlenessRb(1);
        idlenessRb(2)  = toc + idlenessRb(2);
        idlenessRb(3)  = toc + idlenessRb(3);
        idlenessRb(4)  = toc + idlenessRb(4);
        idlenessRb(b)  = 0;
        tic
    end
end

%% Delivered a new place to go, start patrolling and refresh the idleness function
function [] = CheckEnebleStates(idx, b, handles)
global enableTransitionsRa;
global enableTransitionsRb;
global Target;
global achievedPoint;
global PatruCellRa;
global PatruCellRb;
global PatrollingRa;
global PatrollingRb;
global DisableRa;
global DisableRb;
global batSituationRb;
global batSituationRa;
global failSENS_Ra;
global failSENS_Rb;
global idlenessRa;
global idlenessRb;

All = ["PatC1_Ra", "PatC2_Ra", "PatC3_Ra", "PatC4_Ra", "PatC1_Rb", "PatC2_Rb", "PatC3_Rb", "PatC4_Rb", "failMOV_Ra", "failMOV_Rb", "failSENS_Ra", "failSENS_Rb", "FPatC1_Ra", "FPatC1_Rb", "FPatC2_Ra", "FPatC2_Rb", "FPatC3_Ra", "FPatC3_Rb", "FPatC4_Ra", "FPatC4_Rb", "low_batRa", "low_batRb", "ok_batRa", "ok_batRb", "rstF_Ra", "rstF_Rb", "warning_batRa", "warning_batRb"];

if idx == 1
    if PatruCellRa == b && achievedPoint(idx) == 1 && (batSituationRa ~= 1) && (failSENS_Ra == 0)
        StatusPatrol(idx, b, handles);
    end
    if achievedPoint(idx) == 1 && PatrollingRa == 0
        enableA = enableTransitionsRa;
        enableA = setdiff(enableA, All);
        disableEventsTarget(idx);
        enableA = setdiff(enableA, DisableRa)
        LA = length(enableA)
        RandomA = randi([1, LA]);
        Target(idx) = enableA(RandomA)
        achievedPoint(idx) = 0;
    end
    [detectionState, c] = findAnotherRobotPatrolling(idx);
    if (detectionState == 1) 
        TEXT = ["Detected Rb patrolling at ", num2str(c) ," cel"];
        idlenessRa(1)  = toc + idlenessRa(1);
        idlenessRa(2)  = toc + idlenessRa(2);
        idlenessRa(3)  = toc + idlenessRa(3);
        idlenessRa(4)  = toc + idlenessRa(4);
        idlenessRa(c)  = 0;
        tic
        if c == b 
            PatruCellRa = 0;
        end
    elseif (detectionState == 0)
        idlenessRa(1)  = toc + idlenessRa(1);
        idlenessRa(2)  = toc + idlenessRa(2);
        idlenessRa(3)  = toc + idlenessRa(3);
        idlenessRa(4)  = toc + idlenessRa(4);
        tic        
    end
end



if idx == 2
    if PatruCellRb == b && achievedPoint(idx) == 1 && (batSituationRb ~= 1) && (failSENS_Rb == 0)
        StatusPatrol(idx, b, handles);
    end
    if PatrollingRb == 0 && achievedPoint(idx) == 1 
        enableB = enableTransitionsRb;
        enableB = setdiff(enableB, All);
        disableEventsTarget(idx);
        disp(PatruCellRb);
        disp(DisableRb);
        enableB = setdiff(enableB, DisableRb);
        LB = length(enableB);
        RandomB = randi([1, LB]);
        Target(idx) = enableB(RandomB)       
        achievedPoint(idx) = 0;
   end
    
    [detectionState, c] = findAnotherRobotPatrolling(idx);
    if (detectionState == 1)
        TEXT = ["Detected Ra patrolling at ", num2str(c) ," cel"];
        idlenessRb(1)  = toc + idlenessRb(1);
        idlenessRb(2)  = toc + idlenessRb(2);
        idlenessRb(3)  = toc + idlenessRb(3);
        idlenessRb(4)  = toc + idlenessRb(4);
        idlenessRb(c)  = 0;
        tic
        if c == b 
            PatruCellRb = 0;
        end 
    elseif (detectionState == 0)
        idlenessRb(1)  = toc + idlenessRb(1);
        idlenessRb(2)  = toc + idlenessRb(2);
        idlenessRb(3)  = toc + idlenessRb(3);
        idlenessRb(4)  = toc + idlenessRb(4);
        tic        
    end 
end

%% Main function to controle the robots velocity and arrived at the desire points
function [tx, ty, thetaT] = mainstep(idx, tx, ty, thetaT)
global fase;
if (fase(idx) == 0)
    [tx, ty] = sortPoint(idx, tx, ty);
    fase(idx) = 1;
elseif (fase(idx)== 1)
    [thetaT] = SetOrientation(idx, tx, ty, thetaT);
elseif (fase(idx) == 2)
    [x, y, theta] = getPosition(idx);
    [velR, velL] = settingInitialVelocity(idx, tx, ty, x, y);
    updateRobot(idx, tx, ty, thetaT, velR, velL);
end

%% Function to accomplish all the functions
function [] = maincycle(handles)
global running;
global fase;
fase = [0 0];
tx = [0 0];
ty = [0 0];
thetaT = [0 0];
c = [1 2];
tic
while(1)
    if running == 1
        for idx=1:2
            ChangeStateMachineRa(handles);
            ChangeStateMachineRb(handles);
            [x, y, theta] = getPosition(idx);
            [a, b] = position(handles, x, y, idx, c(idx));
            c(idx) = b;
            CheckEnebleStates(idx, b, handles);
            TARGETPatrolling(idx, b)
            [tx(idx), ty(idx), thetaT(idx)] = mainstep(idx, tx(idx), ty(idx), thetaT(idx));
        end
    end
    pause(0.1);
end


%% GUI Part, interface with the user

% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global running;
running = 1;
maincycle(handles);

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vrep;
global clientID;
%global list;
global running;
running = 0;

%dlmwrite('transitions.txt', list,'delimiter','');
disp('Loop ended!');
delete(gcp('nocreate'))
vrep.simxClearIntegerSignal(clientID,'runsignal',vrep.simx_opmode_oneshot);
vrep.simxClearFloatSignal(clientID,'',vrep.simx_opmode_oneshot);
vrep.simxSynchronous(clientID,true);
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxClearFloatSignal(clientID,'',vrep.simx_opmode_oneshot);
vrep.simxFinish(clientID);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global list;
global failMOV_Ra;
global PatrollingRa;
global countPointRa;
global randomPointRa;
global PatrolAchieved;
global PatruCellRa;
global achievedPoint;

PatrollingRa = 0;
countPointRa = 0;
randomPointRa = 0;
PatrolAchieved(1) = 0;
PatruCellRa = 0;
achievedPoint(1) = 0;
failMOV_Ra = 1;
list = [list, "failMOV_Ra"];

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global list;
global failMOV_Rb;
global PatrollingRb;
global countPointRb;
global randomPointRb;
global PatruCellRb;
global achievedPoint;
global PatrolAchieved;

PatrollingRb = 0;
countPointRb = 0;
randomPointRb = 0;
PatruCellRb = 0;
achievedPoint(2) = 0;
PatrolAchieved(2) = 0;
failMOV_Rb = 1;

list = [list, "failMOV_Rb"]

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global list;
global batSituationRa;
global PatrollingRa;
global countPointRa;
global randomPointRa;
global PatrolAchieved;
global PatruCellRa;
global achievedPoint;

a = get(hObject,'Value');
set(handles.text2,'String', a);
if a < 50 && a > 21
    list = [list, "warning_batRa"];
    batSituationRa = 2;
elseif a < 20
    list = [list, "low_batRa"]
    PatrollingRa = 0;
    countPointRa = 0;
    randomPointRa = 0;
    PatrolAchieved(1) = 0;
    PatruCellRa = 0;
    achievedPoint(1) = 1;
    batSituationRa = 1;
elseif a > 51
    list = [list, "ok_batRa"]
    batSituationRa = 3;
    TARGETPatrolling(1);
end

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global batSituationRb;
global list;
global PatrollingRb;
global countPointRb;
global randomPointRb;
global PatruCellRb;
global achievedPoint;
global PatrolAchieved;

a = get(hObject,'Value');
set(handles.text3,'String', a);
if a < 50 && a > 21
    list = [list, "warning_batRb"];   
    batSituationRb = 2;
elseif a < 20
    list = [list, "low_batRb"];
    PatrollingRb = 0;
    countPointRb = 0;
    randomPointRb = 0;
    PatruCellRb = 0;
    achievedPoint(2) = 1;
    PatrolAchieved(2) = 0;
    batSituationRb = 1;
elseif a > 51
    list = [list, "ok_batRb"]
    TARGETPatrolling(2);
    batSituationRb = 3;
end

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.5 .4 .3]);
end

% --- Executes on button press in pushbuttonrstF_Ra.
function pushbuttonResetR1_Callback(hObject, eventdata, handles)
global list;
global failMOV_Ra;
global countR1;
global failSENS_Ra;

if failMOV_Ra == 1
    if countR1 > 20
        failMOV_Ra = 1;
    else
        failMOV_Ra = 0;
        list = [list, "rstF_Ra"]
        TARGETPatrolling(1);
    end
end
if failSENS_Ra == 1
    list = [list, "rstF_Ra"];
    failSENS_Ra = 0;
end

% --- Executes on button press in pushbuttonrstF_Rb.
function pushbuttonResetR2_Callback(hObject, eventdata, handles)

% hObject    handle to pushbuttonrstF_Rb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of pushbuttonrstF_Rb
global list;
global failMOV_Rb;
global countR2;
global failSENS_Rb;

if failMOV_Rb == 1
    if countR2 > 20
        failMOV_Rb = 1;
    else
        failMOV_Rb = 0;
        list = [list, "rstF_Rb"]
        TARGETPatrolling(2);
    end
end
if failSENS_Rb == 1
    list = [list, "rstF_Rb"];
    failSENS_Rb = 0;
end

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global list;
global failSENS_Ra;
global PatrollingRa;
global countPointRa;
global randomPointRa;
global PatrolAchieved;
global PatruCellRa;
global achievedPoint;

PatrollingRa = 0;
countPointRa = 0;
randomPointRa = 0;
PatrolAchieved(1) = 0;
PatruCellRa = 0;
achievedPoint(1) = 0;



failSENS_Ra = 1;
list = [list, "failSENS_Ra"]

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global list;
global failSENS_Rb;
global PatrollingRb;
global countPointRb;
global randomPointRb;
global PatruCellRb;
global achievedPoint;
global PatrolAchieved;


PatrollingRb = 0;
countPointRb = 0;
randomPointRb = 0;
PatruCellRb = 0;
achievedPoint(2) = 0;
PatrolAchieved(2) = 0;

failSENS_Rb = 1;
list = [list, "failSENS_Rb"]

% --- Executes during object creation, after setting all properties.
function text1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

function pushbuttonResetR2_CreateFcn(hObject, eventdata, handles)

function pushbuttonResetR1_CreateFcn(hObject, eventdata, handles)
