% TODO:
% 1. 根据actual marker position，调整virtual marker deviation（如何做？）
% Question:
% 1. bone length scale和marker deviation tunning是交替迭代的，还是一次性完成的？

close all
import org.opensim.modeling.*

%% Read model information

% from model_definition.m:
load markers.mat, load bodies.mat, load joints.mat
load dofMap.mat

%% Read static TRC input (markers)

% these are the actual markers (not the defined virtual markers above) from
% experimental data. They will be in consistent with the virtual markers in
% the future.
trcFile = MarkerData('.\marker_input_static.trc');
% rate = trcFile.getCameraRate;
markerNum = trcFile.getNumMarkers;

markerPos_input = zeros(markerNum, 3);
for i = 0 : markerNum-1 % for every marker
    currentMarkerPos = zeros(trcFile.getNumFrames, 3); % temporary buffer for marker position in STATIC position
    for frame = 0 : trcFile.getNumFrames-1 % for every frame
        tempPos = trcFile.getFrame(frame).getMarker(i);
        currentMarkerPos(frame+1, :) = [tempPos.get(0), tempPos.get(1), tempPos.get(2)] / 1000;
    end
    markerPos_input(i+1, :) = mean(currentMarkerPos);
end

markerInputMap = containers.Map({'RASIS', 'LASIS', 'VSacral', 'RThighUpper', ...
    'RThighFront', 'RThighRear', 'LThighUpper', 'LThighFront', ...
    'LThighRear', 'RKneeLat', 'RKneeMed', 'LKneeLat', 'LKneeMed', ...
    'RShankUpper', 'RShankFront', 'RShankRear', 'LShankUpper', ...
    'LShankFront', 'LShankRear', 'RAnkleLat', 'RAnkleMed', ...
    'LAnkleLat', 'LAnkleMed', 'RHeel', 'RMidfootSup', 'RMidfootLat', ...
    'RToeTip', 'LHeel', 'LMidfootSup', 'LMidfootLat', 'LToeTip', ...
    'Sternum', 'RAcromium', 'LAcromium', 'RBicep', 'LBicep', 'RElbow', ...
    'LElbow', 'RWristMed', 'RWristLat', 'LWristMed', 'LWristLat', ...
    'RToeLat', 'RToeMed', 'LToeLat', 'LToeMed', 'RTemple', ...
    'LTemple', 'TopHead'}, num2cell(1:markerNum));
clear tempPos markerNum currentMarkerPos i frame
save markerInputMap.mat markerInputMap

%% Scale bone length at static pose
% NOTE THAT THIS PROCEDURE SHOULD BE EXECUTED ITERATIVELY UNTIL SCALE
% FACTORS ARE NEAR 1
q = zeros(height(dofMap),1); q(dofMap('pelvis_ty')) = 0.95;

% Topology view:
% 1. [ground] (ROOT) --- ground_pelvis --[pelvis]--> hip_r --[femur_r]--> knee_r --[tibia_r]--> ankle_r --[foot_r]
% 2. [pelvis]--> hip_l --[femur_l]--> knee_l --[tibia_l]--> ankle_l --[foot_l]
% 3. [pelvis]--> back --[torso]--> neck --[head]
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]

% ----- route 1 ----- %
% [ground]--> ground_pelvis --[pelvis]
pelvis.T(1,4) = q(dofMap('pelvis_tx'));
pelvis.T(2,4) = q(dofMap('pelvis_ty'));
pelvis.T(3,4) = q(dofMap('pelvis_tz'));

% [pelvis]--> hip_r --[femur_r]
P = hip_r.translation_parent;
R = eye(3);
femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_r]--> knee_r --[tibia_r]
P = knee_r.translation_parent;
R1 = Rot(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
R2 = Rot(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
R3 = Rot(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
R = R*R1*R2*R3;
tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
femur_r.T(1:3,1:3) = R;

% [tibia_r]--> ankle_r --[foot_r]
P = ankle_r.translation_parent;
R = R*Rot(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
foot_r.T = [R, R*P+tibia_r.T(1:3,4); 0 0 0 1];
tibia_r.T(1:3,1:3) = R;

R1 = Rot(ankle_r.axis(1:3,1), q(dofMap('ankle_dorsiflexion_r')));
R2 = Rot(ankle_r.axis(1:3,2), q(dofMap('ankle_adduction_r')));
R = R*R1*R2;
foot_r.T(1:3,1:3) = R;

% ----- route 2 ----- %
% [pelvis]--> hip_l --[femur_l]
P = hip_l.translation_parent;
R = eye(3);
femur_l.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [femur_l]--> knee_l --[tibia_l]
P = knee_l.translation_parent;
R1 = Rot(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
R2 = Rot(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
R3 = Rot(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
R = R*R1*R2*R3;
tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
femur_l.T(1:3,1:3) = R;

% [tibia_l]--> ankle_l --[foot_l]
P = ankle_l.translation_parent;
R = R*Rot(knee_l.axis(1:3,1), q(dofMap('knee_flexion_l')));
foot_l.T = [R, R*P+tibia_l.T(1:3,4); 0 0 0 1];
tibia_l.T(1:3,1:3) = R;

R1 = Rot(ankle_l.axis(1:3,1), q(dofMap('ankle_dorsiflexion_l')));
R2 = Rot(ankle_l.axis(1:3,2), q(dofMap('ankle_adduction_l')));
R = R*R1*R2;
foot_l.T(1:3,1:3) = R;

% ----- route 3 ----- %
% 3. [pelvis]--> back --[torso]--> neck --[head]
% [pelvis]--> back --[torso]
P = back.translation_parent;
R = eye(3);
torso.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

% [torso]--> neck --[head]
P = neck.translation_parent;
R1 = Rot(back.axis(1:3,1), q(dofMap('lumbar_extension')));
R2 = Rot(back.axis(1:3,2), q(dofMap('lumbar_bending')));
R3 = Rot(back.axis(1:3,3), q(dofMap('lumbar_rotation')));
R = R*R1*R2*R3;
head.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];
torso.T(1:3,1:3) = R;

R1 = Rot(neck.axis(1:3,1), q(dofMap('neck_extension')));
R2 = Rot(neck.axis(1:3,2), q(dofMap('neck_bending')));
R3 = Rot(neck.axis(1:3,3), q(dofMap('neck_rotation')));
R = R*R1*R2*R3;
head.T(1:3,1:3) = R;

% ----- route 4 ----- %
% 4. [torso]--> shoulder_r --[arm_r]--> elbow_r --[forearm_r]--> wrist_r --[hand_r]
% [torso]--> shoulder_r --[arm_r]
P = shoulder_r.translation_parent;
R = torso.T(1:3,1:3);
arm_r.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_r]--> elbow_r -- [forearm_r]
P = elbow_r.translation_parent;
R1 = Rot(shoulder_r.axis(1:3,1), q(dofMap('arm_flex_r')));
R2 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_add_r')));
R3 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_rot_r')));
R = R*R1*R2*R3;
forearm_r.T = [R, R*P+arm_r.T(1:3,4); 0 0 0 1];
arm_r.T(1:3,1:3) = R;

% [forearm_r]--> wrist_r -- [hand_r]
P = wrist_r.translation_parent;
R1 = Rot(elbow_r.axis(1:3,1), q(dofMap('elbow_flex_r')));
R2 = Rot(elbow_r.axis(1:3,2), q(dofMap('elbow_rot_r')));
R = R*R1*R2;
hand_r.T = [R, R*P+forearm_r.T(1:3,4); 0 0 0 1];
forearm_r.T(1:3,1:3) = R;

R = R*Rot(wrist_r.axis(1:3,1), q(dofMap('wrist_flex_r')));
hand_r.T(1:3,1:3) = R;

% ----- route 5 ----- %
% 5. [torso]--> shoulder_l --[arm_l]--> elbow_l --[forearm_l]--> wrist_l --[hand_l]
% [torso]--> shoulder_l --[arm_l]
P = shoulder_l.translation_parent;
R = torso.T(1:3,1:3);
arm_l.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

% [arm_l]--> elbow_l -- [forearm_l]
P = elbow_l.translation_parent;
R1 = Rot(shoulder_l.axis(1:3,1), q(dofMap('arm_flex_l')));
R2 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_add_l')));
R3 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_rot_l')));
R = R*R1*R2*R3;
forearm_l.T = [R, R*P+arm_l.T(1:3,4); 0 0 0 1];
arm_l.T(1:3,1:3) = R;

% [forearm_l]--> wrist_l -- [hand_l]
P = wrist_l.translation_parent;
R1 = Rot(elbow_l.axis(1:3,1), q(dofMap('elbow_flex_l')));
R2 = Rot(elbow_l.axis(1:3,2), q(dofMap('elbow_rot_l')));
R = R*R1*R2;
hand_l.T = [R, R*P+forearm_l.T(1:3,4); 0 0 0 1];
forearm.T(1:3,1:3) = R;

R = R*Rot(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
hand_l.T(1:3,1:3) = R;


% --- Marker position calculation --- %
% 1. markers attached to torso
location = eval(Sternum.attachedBody).T * [Sternum.deviation; 1];
Sternum.location = location(1:3);
location = eval(RAcromium.attachedBody).T * [RAcromium.deviation; 1];
RAcromium.location = location(1:3);
location = eval(LAcromium.attachedBody).T * [LAcromium.deviation; 1];
LAcromium.location = location(1:3);
% 2. markers attached to pelvis
location = eval(RASIS.attachedBody).T * [RASIS.deviation; 1];
RASIS.location = location(1:3);
location = eval(LASIS.attachedBody).T * [LASIS.deviation; 1];
LASIS.location = location(1:3);
location = eval(RPSIS.attachedBody).T * [RPSIS.deviation; 1];
RPSIS.location = location(1:3);
location = eval(LPSIS.attachedBody).T * [LPSIS.deviation; 1];
LPSIS.location = location(1:3);
% 3. markers attached to femur, r/l
location = eval(RTIB.attachedBody).T * [RTIB.deviation; 1];
RTIB.location = location(1:3);
location = eval(LTIB.attachedBody).T * [LTIB.deviation; 1];
LTIB.location = location(1:3);
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
% 4. markers attached to tibia, r/l
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
% 5. markers attached to foot, r/l
location = eval(RHeel.attachedBody).T * [RHeel.deviation; 1];
RHeel.location = location(1:3);
location = eval(LHeel.attachedBody).T * [LHeel.deviation; 1];
LHeel.location = location(1:3);
location = eval(RToeLat.attachedBody).T * [RToeLat.deviation; 1];
RToeLat.location = location(1:3);
location = eval(LToeLat.attachedBody).T * [LToeLat.deviation; 1];
LToeLat.location = location(1:3);
location = eval(RToeTip.attachedBody).T * [RToeTip.deviation; 1];
RToeTip.location = location(1:3);
location = eval(LToeTip.attachedBody).T * [LToeTip.deviation; 1];
LToeTip.location = location(1:3);
% 6. markers attached to arm, r/l
location = eval(RBicep.attachedBody).T * [RBicep.deviation; 1];
RBicep.location = location(1:3);
location = eval(LBicep.attachedBody).T * [LBicep.deviation; 1];
LBicep.location = location(1:3);
location = eval(RElbow.attachedBody).T * [RElbow.deviation; 1];
RElbow.location = location(1:3);
location = eval(LElbow.attachedBody).T * [LElbow.deviation; 1];
LElbow.location = location(1:3);
% 7. markers attached to forearm
location = eval(RFAsuperior.attachedBody).T * [RFAsuperior.deviation; 1];
RFAsuperior.location = location(1:3);
location = eval(LFAsuperior.attachedBody).T * [LFAsuperior.deviation; 1];
LFAsuperior.location = location(1:3);
location = eval(RWristLat.attachedBody).T * [RWristLat.deviation; 1];
RWristLat.location = location(1:3);
location = eval(LWristLat.attachedBody).T * [LWristLat.deviation; 1];
LWristLat.location = location(1:3);
location = eval(RWristMed.attachedBody).T * [RWristMed.deviation; 1];
RWristMed.location = location(1:3);
location = eval(LWristMed.attachedBody).T * [LWristMed.deviation; 1];
LWristMed.location = location(1:3);
% 8. markers attached to head
location = eval(RFHD.attachedBody).T * [RFHD.deviation; 1];
RFHD.location = location(1:3);
location = eval(RFHD.attachedBody).T * [LFHD.deviation; 1];
LFHD.location = location(1:3);
location = eval(RFHD.attachedBody).T * [RBHD.deviation; 1];
RBHD.location = location(1:3);
location = eval(RFHD.attachedBody).T * [LBHD.deviation; 1];
LBHD.location = location(1:3);

% --- Scale factor calculation --- %
% algorithm: experimental displacement / virtual displacement
scaleFactor.pelvis = norm(markerPos_input(markerInputMap('RASIS'),:) - markerPos_input(markerInputMap('LASIS'),:)) / norm(RASIS.location - LASIS.location);
scaleFactor.femur_r = norm(markerPos_input(markerInputMap('RASIS'),:) - markerPos_input(markerInputMap('RKneeLat'),:)) / norm(RASIS.location - RKneeLat.location);
scaleFactor.femur_l = norm(markerPos_input(markerInputMap('LASIS'),:) - markerPos_input(markerInputMap('LKneeLat'),:)) / norm(LASIS.location - LKneeLat.location);
scaleFactor.tibia_r = norm(markerPos_input(markerInputMap('RKneeLat'),:) - markerPos_input(markerInputMap('RAnkleLat'),:)) / norm(RKneeLat.location - RAnkleLat.location);
scaleFactor.tibia_l = norm(markerPos_input(markerInputMap('LKneeLat'),:) - markerPos_input(markerInputMap('LAnkleLat'),:)) / norm(LKneeLat.location - LAnkleLat.location);
scaleFactor.foot_r = norm(markerPos_input(markerInputMap('RHeel'),:) - markerPos_input(markerInputMap('RToeTip'),:)) / norm(RHeel.location - RToeTip.location);
scaleFactor.foot_l = norm(markerPos_input(markerInputMap('LHeel'),:) - markerPos_input(markerInputMap('LToeTip'),:)) / norm(LHeel.location - LToeTip.location);
torsoSup = mean([markerPos_input(markerInputMap('RAcromium'),:); markerPos_input(markerInputMap('LAcromium'),:)]);
% torso后续的缩放规则改为C7~VSacral(mean([RPSIS, LPSIS]))
scaleFactor.torso = norm(torsoSup - markerPos_input(markerInputMap('VSacral'),:)) / norm(mean([RAcromium.location'; LAcromium.location']) - mean([RPSIS.location'; LPSIS.location']));
% head暂时没有缩放规则，后续改为mean([RFHD, LFHD, RBHD, LBHD])~C7
scaleFactor.head = scaleFactor.torso;
scaleFactor.arm_r = norm(markerPos_input(markerInputMap('RAcromium'),:) - markerPos_input(markerInputMap('RElbow'),:)) / norm(RAcromium.location - RElbow.location);
scaleFactor.arm_l = norm(markerPos_input(markerInputMap('LAcromium'),:) - markerPos_input(markerInputMap('LElbow'),:)) / norm(LAcromium.location - LElbow.location);
scaleFactor.forearm_r = norm(markerPos_input(markerInputMap('RElbow'),:) - markerPos_input(markerInputMap('RWristLat'),:)) / norm(RElbow.location - RWristLat.location);
scaleFactor.forearm_l = norm(markerPos_input(markerInputMap('LElbow'),:) - markerPos_input(markerInputMap('LWristLat'),:)) / norm(LElbow.location - LWristLat.location);

save scaleFactor.mat scaleFactor