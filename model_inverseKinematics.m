% 首先以IK_input.trc的第一帧数据进行验证
% 骨骼质量已经缩放过，无需 *scaleFactor.mass
% 相邻关节的位移关系未缩放，需要 *scaleFactor.xxx（对应缩放骨骼的名称）

% syms x [2 1]
% y = (x1-1.5473)^2 + (x2+3)^2;
% fun = matlabFunction(y,"Vars",{x});
% x = fmincon(fun,[3 6]', [1 1],10)

clear all, close all, clc
import org.opensim.modeling.*

%% Read model information

load scaleFactor.mat, load markers.mat, load bodies.mat, load joints.mat
load dofMap.mat, load markerInputMap.mat
syms q [33 1] % 广义坐标（关节自由度）

%% Read walking TRC input (markers)

trcFile = MarkerData('.\marker_input_walking.trc');

markerPos_ref = zeros(trcFile.getNumFrames, trcFile.getNumMarkers, 3);
for i = 0 : trcFile.getNumMarkers-1 % for every marker
    for frame = 0 : trcFile.getNumFrames-1 % for every frame
        tempPos = trcFile.getFrame(frame).getMarker(i);
        markerPos_ref(frame+1, i+1, :) = [tempPos.get(0), tempPos.get(1), tempPos.get(2)] / 1000;
    end
end

% walking的marker点竟然不同于static，需要重新构建映射
% 要检查到底有哪些点可以参与优化
markerInputMap = containers.Map({'RASIS', 'LASIS', 'VSacral', 'RThighUpper', ...
    'RThighFront', 'RThighRear', 'LThighUpper', 'LThighFront', ...
    'LThighRear', 'RShankUpper', 'RShankFront', 'RShankRear', 'LShankUpper', ...
    'LShankFront', 'LShankRear', 'RHeel', 'RMidfootSup', 'RMidfootLat', ...
    'RToeTip', 'LHeel', 'LMidfootSup', 'LMidfootLat', 'LToeTip', ...
    'Sternum', 'RAcromium', 'LAcromium', 'RBicep', 'LBicep', 'RElbow', ...
    'LElbow', 'RWristMed', 'RWristLat', 'LWristMed', 'LWristLat', ...
    'RToeLat', 'RToeMed', 'LToeLat', 'LToeMed', 'RTemple', ...
    'LTemple', 'TopHead'}, num2cell(1:trcFile.getNumMarkers));

clear tempPos i frame currentMarkerPos

%% Calculate body positions according to q

% Topology view:
% 1. [ground] (ROOT) --- ground_pelvis --[pelvis]--> hip_r --[femur_r]--> knee_r --[tibia_r]--> ankle_r --[foot_r]
% 2. [pelvis]--> hip_l --[femur_l]--> knee_l --[tibia_l]--> ankle_l --[foot_l]
% 3. [pelvis]--> back --[torso]--> neck --[head]
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]

% ----- route 1 ----- %
% [ground]--> ground_pelvis --[pelvis]
pelvis.T = sym(pelvis.T);
pelvis.T(1,4) = q(dofMap('pelvis_tx'));
pelvis.T(2,4) = q(dofMap('pelvis_ty'));
pelvis.T(3,4) = q(dofMap('pelvis_tz'));

% [pelvis]--> hip_r --[femur_r]
P = hip_r.translation_parent * scaleFactor.pelvis;
R = eye(3);
femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_r]--> knee_r --[tibia_r]
P = knee_r.translation_parent * scaleFactor.femur_r;
R1 = RotSym(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
R2 = RotSym(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
R3 = RotSym(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
R = R*R1*R2*R3;
tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
femur_r.T(1:3,1:3) = R;

% [tibia_r]--> ankle_r --[foot_r]
P = ankle_r.translation_parent * scaleFactor.tibia_r;
R = R*RotSym(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
foot_r.T = [R, R*P+tibia_r.T(1:3,4); 0 0 0 1];
tibia_r.T(1:3,1:3) = R;

R1 = RotSym(ankle_r.axis(1:3,1), q(dofMap('ankle_dorsiflexion_r')));
R2 = RotSym(ankle_r.axis(1:3,2), q(dofMap('ankle_adduction_r')));
R = R*R1*R2;
foot_r.T(1:3,1:3) = R;

% ----- route 2 ----- %
% [pelvis]--> hip_l --[femur_l]
P = hip_l.translation_parent * scaleFactor.pelvis;
R = eye(3);
femur_l.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_l]--> knee_l --[tibia_l]
P = knee_l.translation_parent * scaleFactor.femur_l;
R1 = RotSym(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
R2 = RotSym(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
R3 = RotSym(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
R = R*R1*R2*R3;
tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
femur_l.T(1:3,1:3) = R;

% [tibia_l]--> ankle_l --[foot_l]
P = ankle_l.translation_parent * scaleFactor.tibia_l;
R = R*RotSym(knee_l.axis(1:3,1), q(dofMap('knee_flexion_l')));
foot_l.T = [R, R*P+tibia_l.T(1:3,4); 0 0 0 1];
tibia_l.T(1:3,1:3) = R;

R1 = RotSym(ankle_l.axis(1:3,1), q(dofMap('ankle_dorsiflexion_l')));
R2 = RotSym(ankle_l.axis(1:3,2), q(dofMap('ankle_adduction_l')));
R = R*R1*R2;
foot_l.T(1:3,1:3) = R;

% ----- route 3 ----- %
% 3. [pelvis]--> back --[torso]--> neck --[head]
% [pelvis]--> back --[torso]
P = back.translation_parent * scaleFactor.torso;
R = eye(3);
torso.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [torso]--> neck --[head]
P = neck.translation_parent * scaleFactor.torso;
R1 = RotSym(back.axis(1:3,1), q(dofMap('lumbar_extension')));
R2 = RotSym(back.axis(1:3,2), q(dofMap('lumbar_bending')));
R3 = RotSym(back.axis(1:3,3), q(dofMap('lumbar_rotation')));
R = R*R1*R2*R3;
head.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];
torso.T(1:3,1:3) = R;

R1 = RotSym(neck.axis(1:3,1), q(dofMap('neck_extension')));
R2 = RotSym(neck.axis(1:3,2), q(dofMap('neck_bending')));
R3 = RotSym(neck.axis(1:3,3), q(dofMap('neck_rotation')));
R = R*R1*R2*R3;
head.T(1:3,1:3) = R;

% ----- route 4 ----- %
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% [torso]--> shoulder_r --[arm_r]
P = shoulder_r.translation_parent * scaleFactor.torso;
R = torso.T(1:3,1:3);
arm_r.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_r]--> elbow_r -- [forearm_r]
P = elbow_r.translation_parent * scaleFactor.height;
R1 = RotSym(shoulder_r.axis(1:3,1), q(dofMap('arm_flex_r')));
R2 = RotSym(shoulder_r.axis(1:3,2), q(dofMap('arm_add_r')));
R3 = RotSym(shoulder_r.axis(1:3,2), q(dofMap('arm_rot_r')));
R = R*R1*R2*R3;
forearm_r.T = [R, R*P+arm_r.T(1:3,4); 0 0 0 1];
arm_r.T(1:3,1:3) = R;

% [forearm_r]--> wrist_r -- [hand_r]
P = wrist_r.translation_parent * scaleFactor.height;
R1 = RotSym(elbow_r.axis(1:3,1), q(dofMap('elbow_flex_r')));
R2 = RotSym(elbow_r.axis(1:3,2), q(dofMap('elbow_rot_r')));
R = R*R1*R2;
hand_r.T = [R, R*P+forearm_r.T(1:3,4); 0 0 0 1];
forearm_r.T(1:3,1:3) = R;

R = R*RotSym(wrist_r.axis(1:3,1), q(dofMap('wrist_flex_r')));
hand_r.T(1:3,1:3) = R;

% ----- route 5 ----- %
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]
% [torso]--> shoulder_l --[arm_l]
P = shoulder_l.translation_parent * scaleFactor.torso;
R = torso.T(1:3,1:3);
arm_l.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_l]--> elbow_l -- [forearm_l]
P = elbow_l.translation_parent * scaleFactor.height;
R1 = RotSym(shoulder_l.axis(1:3,1), q(dofMap('arm_flex_l')));
R2 = RotSym(shoulder_l.axis(1:3,2), q(dofMap('arm_add_l')));
R3 = RotSym(shoulder_l.axis(1:3,2), q(dofMap('arm_rot_l')));
R = R*R1*R2*R3;
forearm_l.T = [R, R*P+arm_l.T(1:3,4); 0 0 0 1];
arm_l.T(1:3,1:3) = R;

% [forearm_l]--> wrist_l -- [hand_l]
P = wrist_l.translation_parent * scaleFactor.height;
R1 = RotSym(elbow_l.axis(1:3,1), q(dofMap('elbow_flex_l')));
R2 = RotSym(elbow_l.axis(1:3,2), q(dofMap('elbow_rot_l')));
R = R*R1*R2;
hand_l.T = [R, R*P+forearm_l.T(1:3,4); 0 0 0 1];
forearm.T(1:3,1:3) = R;

R = R*RotSym(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
hand_l.T(1:3,1:3) = R;

%% Calculate marker positions according to q

% --- Marker position calculation --- %
% 1. markers attached to torso
location = eval(Sternum.attachedBody).T * [Sternum.deviation; 1];
Sternum.location = location(1:3);
location = eval(LAcromium.attachedBody).T * [LAcromium.deviation; 1];
LAcromium.location = location(1:3);
location = eval(TopHead.attachedBody).T * [TopHead.deviation; 1];
TopHead.location = location(1:3);
location = eval(RASIS.attachedBody).T * [RASIS.deviation; 1];
RASIS.location = location(1:3);
location = eval(LASIS.attachedBody).T * [LASIS.deviation; 1];
LASIS.location = location(1:3);
location = eval(VSacral.attachedBody).T * [VSacral.deviation; 1];
VSacral.location = location(1:3);
location = eval(RThighUpper.attachedBody).T * [RThighUpper.deviation; 1];
RThighUpper.location = location(1:3);
location = eval(RThighFront.attachedBody).T * [RThighFront.deviation; 1];
RThighFront.location = location(1:3);
location = eval(RThighRear.attachedBody).T * [RThighRear.deviation; 1];
RThighRear.location = location(1:3);
location = eval(RKneeLat.attachedBody).T * [RKneeLat.deviation; 1];
RKneeLat.location = location(1:3);
location = eval(RKneeMed.attachedBody).T * [RKneeMed.deviation; 1];
RKneeMed.location = location(1:3);
location = eval(RShankUpper.attachedBody).T * [RShankUpper.deviation; 1];
RShankUpper.location = location(1:3);
location = eval(RShankFront.attachedBody).T * [RShankFront.deviation; 1];
RShankFront.location = location(1:3);
location = eval(RShankRear.attachedBody).T * [RShankRear.deviation; 1];
RShankRear.location = location(1:3);
location = eval(RAnkleLat.attachedBody).T * [RAnkleLat.deviation; 1];
RAnkleLat.location = location(1:3);
location = eval(RAnkleMed.attachedBody).T * [RAnkleMed.deviation; 1];
RAnkleMed.location = location(1:3);
location = eval(RHeel.attachedBody).T * [RHeel.deviation; 1];
RHeel.location = location(1:3);
location = eval(RMidfootSup.attachedBody).T * [RMidfootSup.deviation; 1];
RMidfootSup.location = location(1:3);
location = eval(RMidfootLat.attachedBody).T * [RMidfootLat.deviation; 1];
RMidfootLat.location = location(1:3);
location = eval(RMidToeLat.attachedBody).T * [RMidToeLat.deviation; 1];
RMidToeLat.location = location(1:3);
location = eval(RMidToeMed.attachedBody).T * [RMidToeMed.deviation; 1];
RMidToeMed.location = location(1:3);
location = eval(RMidToeTip.attachedBody).T * [RMidToeTip.deviation; 1];
RMidToeTip.location = location(1:3);

location = eval(LThighUpper.attachedBody).T * [LThighUpper.deviation; 1];
LThighUpper.location = location(1:3);
location = eval(LThighFront.attachedBody).T * [LThighFront.deviation; 1];
LThighFront.location = location(1:3);
location = eval(LThighRear.attachedBody).T * [LThighRear.deviation; 1];
LThighRear.location = location(1:3);
location = eval(LKneeLat.attachedBody).T * [LKneeLat.deviation; 1];
LKneeLat.location = location(1:3);
location = eval(LKneeMed.attachedBody).T * [LKneeMed.deviation; 1];
LKneeMed.location = location(1:3);
location = eval(LShankUpper.attachedBody).T * [LShankUpper.deviation; 1];
LShankUpper.location = location(1:3);
location = eval(LShankFront.attachedBody).T * [LShankFront.deviation; 1];
LShankFront.location = location(1:3);
location = eval(LShankRear.attachedBody).T * [LShankRear.deviation; 1];
LShankRear.location = location(1:3);
location = eval(LAnkleLat.attachedBody).T * [LAnkleLat.deviation; 1];
LAnkleLat.location = location(1:3);
location = eval(LAnkleMed.attachedBody).T * [LAnkleMed.deviation; 1];
LAnkleMed.location = location(1:3);
location = eval(LHeel.attachedBody).T * [LHeel.deviation; 1];
LHeel.location = location(1:3);
location = eval(LMidfootSup.attachedBody).T * [LMidfootSup.deviation; 1];
LMidfootSup.location = location(1:3);
location = eval(LMidfootLat.attachedBody).T * [LMidfootLat.deviation; 1];
LMidfootLat.location = location(1:3);
location = eval(LMidToeLat.attachedBody).T * [LMidToeLat.deviation; 1];
LMidToeLat.location = location(1:3);
location = eval(LMidToeMed.attachedBody).T * [LMidToeMed.deviation; 1];
LMidToeMed.location = location(1:3);
location = eval(LMidToeTip.attachedBody).T * [LMidToeTip.deviation; 1];
LMidToeTip.location = location(1:3);

%% Calculate the position error of marker positions

% weights
weight(markerInputMap('Sternum')) = 1;
weight(markerInputMap('LAcromium')) = 0.5;
weight(markerInputMap('TopHead')) = 0.1;
weight(markerInputMap('RASIS')) = 10;
weight(markerInputMap('LASIS')) = 10;
weight(markerInputMap('VSacral')) = 10; 
weight(markerInputMap('RThighUpper')) = 1;
weight(markerInputMap('RThighFront')) = 1;
weight(markerInputMap('RThighRear')) = 1;
weight(markerInputMap('RShankUpper')) = 1;
weight(markerInputMap('RShankFront')) = 1;
weight(markerInputMap('RShankRear')) = 1;
weight(markerInputMap('RHeel')) = 10;
weight(markerInputMap('RMidfootSup')) = 1;
weight(markerInputMap('RMidfootLat')) = 1;
weight(markerInputMap('RToeLat')) = 1;
weight(markerInputMap('RToeMed')) = 1;
weight(markerInputMap('LThighUpper')) = 1;
weight(markerInputMap('LThighFront')) = 1;
weight(markerInputMap('LThighRear')) = 1;
weight(markerInputMap('LShankUpper')) = 1;
weight(markerInputMap('LShankFront')) = 1;
weight(markerInputMap('LShankRear')) = 1;
weight(markerInputMap('LHeel')) = 10;
weight(markerInputMap('LMidfootSup')) = 1;
weight(markerInputMap('LMidfootLat')) = 1;
weight(markerInputMap('LToeLat')) = 1;
weight(markerInputMap('LToeMed')) = 1;

% position errors (Euclidean distance)
error(markerInputMap('Sternum')) = norm(Sternum.location - reshape(markerPos_ref(1,markerInputMap('Sternum'),:),[3 1]));
error(markerInputMap('LAcromium')) = norm(LAcromium.location - reshape(markerPos_ref(1,markerInputMap('LAcromium'),:),[3 1]));
error(markerInputMap('TopHead')) = norm(TopHead.location - reshape(markerPos_ref(1,markerInputMap('TopHead'),:),[3 1]));
error(markerInputMap('RASIS')) = norm(RASIS.location - reshape(markerPos_ref(1,markerInputMap('RASIS'),:),[3 1]));
error(markerInputMap('LASIS')) = norm(LASIS.location - reshape(markerPos_ref(1,markerInputMap('LASIS'),:),[3 1]));
error(markerInputMap('VSacral')) = norm(VSacral.location - reshape(markerPos_ref(1,markerInputMap('VSacral'),:),[3 1]));
error(markerInputMap('RThighUpper')) = norm(RThighUpper.location - reshape(markerPos_ref(1,markerInputMap('RThighUpper'),:),[3 1]));
error(markerInputMap('RThighFront')) = norm(RThighFront.location - reshape(markerPos_ref(1,markerInputMap('RThighFront'),:),[3 1]));
error(markerInputMap('RThighRear')) = norm(RThighRear.location - reshape(markerPos_ref(1,markerInputMap('RThighRear'),:),[3 1]));
error(markerInputMap('RShankUpper')) = norm(RShankUpper.location - reshape(markerPos_ref(1,markerInputMap('RShankUpper'),:),[3 1]));
error(markerInputMap('RShankFront')) = norm(RShankFront.location - reshape(markerPos_ref(1,markerInputMap('RShankFront'),:),[3 1]));
error(markerInputMap('RShankRear')) = norm(RShankRear.location - reshape(markerPos_ref(1,markerInputMap('RShankRear'),:),[3 1]));
error(markerInputMap('RHeel')) = norm(RHeel.location - reshape(markerPos_ref(1,markerInputMap('RHeel'),:),[3 1]));
error(markerInputMap('RMidfootSup')) = norm(RMidfootSup.location - reshape(markerPos_ref(1,markerInputMap('RMidfootSup'),:),[3 1]));
error(markerInputMap('RMidfootLat')) = norm(RMidfootLat.location - reshape(markerPos_ref(1,markerInputMap('RMidfootLat'),:),[3 1]));
error(markerInputMap('LThighUpper')) = norm(LThighUpper.location - reshape(markerPos_ref(1,markerInputMap('LThighUpper'),:),[3 1]));
error(markerInputMap('LThighFront')) = norm(LThighFront.location - reshape(markerPos_ref(1,markerInputMap('LThighFront'),:),[3 1]));
error(markerInputMap('LThighRear')) = norm(LThighRear.location - reshape(markerPos_ref(1,markerInputMap('LThighRear'),:),[3 1]));
error(markerInputMap('LShankUpper')) = norm(LShankUpper.location - reshape(markerPos_ref(1,markerInputMap('LShankUpper'),:),[3 1]));
error(markerInputMap('LShankFront')) = norm(LShankFront.location - reshape(markerPos_ref(1,markerInputMap('LShankFront'),:),[3 1]));
error(markerInputMap('LShankRear')) = norm(LShankRear.location - reshape(markerPos_ref(1,markerInputMap('LShankRear'),:),[3 1]));
error(markerInputMap('LHeel')) = norm(LHeel.location - reshape(markerPos_ref(1,markerInputMap('LHeel'),:),[3 1]));
error(markerInputMap('LMidfootSup')) = norm(LMidfootSup.location - reshape(markerPos_ref(1,markerInputMap('LMidfootSup'),:),[3 1]));
error(markerInputMap('LMidfootLat')) = norm(LMidfootLat.location - reshape(markerPos_ref(1,markerInputMap('LMidfootLat'),:),[3 1]));

Loss = weight * error';

%% Optimize q to minimize the marker position error

fun = matlabFunction(Loss, "Vars", {q}); % optimization problem
q0 = zeros(height(q), 1); q0(dofMap('pelvis_ty')) = 0.95; % initial position
% A*q ≤ b 暂时还未利用上
A = []; % eye(height(q));
b = []; % deg2rad(180) * ones(height(q), 1);
% Aeq*q = beq
Aeq = []; % eye(height(q)); Aeq(1:2,1:2) = 0;
beq = []; % zeros(height(q),1);
% lb ≤ q ≤ ub (lower bound, upper bound)
lb = []; % deg2rad(-180) * ones(height(q), 1);
ub = []; % deg2rad(180) * ones(height(q), 1);

% OPTIMIZATION
q_ = fmincon(fun, q0, A, b, Aeq, beq, lb, ub); % q_'

%% Visualize the optimization results



%% Read IK input (for validation)

motFile = Storage('IK_input.mot', false);
labels = motFile.getColumnLabels;
numLabels = labels.getSize; % including the label 'time'
npts = motFile.getSize;
time = nan(npts, 1);
IKdata = nan(npts, numLabels-1);
for i = 1 : npts
    state = motFile.getStateVector(i-1);
    time(i) = state.getTime;
    for j = 1 : dofMap.length
        IKdata(i,j) = state.getData.getitem(j-1);
    end
end
clear state i j labels numLabels