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
syms q [33 1] % generalized coordinates (degrees of freedom)

%% Read walking TRC input (markers)

trcFile = MarkerData('.\marker_input_walking.trc');

markerPos_ref = zeros(trcFile.getNumFrames, trcFile.getNumMarkers, 3);
for i = 0 : trcFile.getNumMarkers-1 % for every marker
    for frame = 0 : trcFile.getNumFrames-1 % for every frame
        tempPos = trcFile.getFrame(frame).getMarker(i);
        markerPos_ref(frame+1, i+1, :) = [tempPos.get(0), tempPos.get(1), tempPos.get(2)] / 1000;
    end
end

% 实验数据中的marker种类（不同于自己构建的marker）
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

%% Calculate absolute poses of body axes according to measured marker positions

% 1. For every body, do:
% a. Determine P, Q: virtual deviations relative to the attached bodies,
% and measured deviations, of the selected markers (≥3)
% b. Compute homogeneous transformation matrix relative to {W}
% 
% 2. Compute homogeneous transformation matrix relative to its parent body
% 
% 3. Derive Joint angles by decompositing the homogeneous transformation
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            % matrices

% pelvis
% related virtual markers: RASIS, LASIS, RPSIS, LPSIS
% related experimental markers: RASIS, LASIS, VSacral[(RPSIS+LPSIS)/2]
VSacral.deviation = (RPSIS.deviation + LPSIS.deviation)/2;
P = [RASIS.deviation, LASIS.deviation, VSacral.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('RASIS'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LASIS'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('VSacral'),:),3,1)];
absT.pelvis = compute_homogeneous_transform(P, Q); 

% torso
% related virtual markers: Sternum, RAcromium, LAcromium, C7, T10, Clavicle
% related experimental markers: Sternum, RAcromium, LAcromium
P = [Sternum.deviation, RAcromium.deviation, LAcromium.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('Sternum'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RAcromium'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LAcromium'),:),3,1)];
absT.torso = compute_homogeneous_transform(P, Q);

% femur
% related virtual markers: RThighUpper, RThighFront, RThighRear,
% LThighUpper, LThighFront, LThighRear, RKneeLat, RKneeMed, LKneeLat,
% LKneeMed
% related experimental markers: RThighUpper, RThighFront, RThighRear,
% LThighUpper, LThighFront, LThighRear
P = [RThighUpper.deviation, RThighFront.deviation, RThighRear.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('RThighUpper'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RThighFront'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RThighRear'),:),3,1)];
absT.femur_r = compute_homogeneous_transform(P, Q);

P = [LThighUpper.deviation, LThighFront.deviation, LThighRear.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('LThighUpper'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LThighFront'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LThighRear'),:),3,1)];
absT.femur_l = compute_homogeneous_transform(P, Q);

% tibia
% related virtual markers: RTIB, LTIB, RShankUpper, RShankFront,
% RShankRear, LShankUpper, LShankFront, LShankRear, RAnkleLat, RAnkleMed,
% LAnkleLat, LAnkleMed
% related experimental markers: RShankUpper, RShankFront, RShankRear, 
% LShankUpper, LShankFront, LShankRear
P = [RShankUpper.deviation, RShankFront.deviation, RShankRear.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('RShankUpper'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RShankFront'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RShankRear'),:),3,1)];
absT.tibia_r = compute_homogeneous_transform(P, Q);

P = [LShankUpper.deviation, LShankFront.deviation, LShankRear.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('LShankUpper'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LShankFront'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LShankRear'),:),3,1)];
absT.tibia_l = compute_homogeneous_transform(P, Q);

% foot
% related virtual markers: RHeel, RToeLat, RToeTip, LHeel, LToeLat, LToeTip
% related experimental markers: RMidfootLat, RMidfootSup, LMidfootLat,
% LMidfootSup, RHeel, LHeel, RToeLat, RToeMed, RToeTip, LToeLat, LToeMed,
% LToeTip
P = [RHeel.deviation, RToeLat.deviation, RToeTip.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('RHeel'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RToeLat'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('RToeTip'),:),3,1)];
absT.foot_r = compute_homogeneous_transform(P, Q);

P = [LHeel.deviation, LToeLat.deviation, LToeTip.deviation];
Q = [reshape(markerPos_ref(1,markerInputMap('LHeel'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LToeLat'),:),3,1), ...
     reshape(markerPos_ref(1,markerInputMap('LToeTip'),:),3,1)];
absT.foot_l = compute_homogeneous_transform(P, Q);

% arm
% related virtual markers: RBicep, LBicep, RElbow, LElbow
% related experimental markers: RBicep, LBicep, RElbow, LElbow
% 附着骨骼的只有两个点，似乎不够做逆运动学?

% forearm
% related virtual markers: RFAsuperior, LFAsuperior, RWristLat, RWristMed, 
% LWristLat, LWristMed
% related experimental markers: RWristLat, RWristMed, LWristLat, LWristMed



% head
% related virtual markers: RFHD, LFHD, RBHD, LBHD
% related experimental markers: TopHead, RTemple, LTemple


clear P Q;
% plot
% axis definition: x/y/z in OpenSim -> -z/-x/y in MATLAB
plot_abs_pose(absT);


%% Calculate relative poses of body axes

relT.pelvis = absT.pelvis;
relT.femur_r = absT.pelvis \ absT.femur_r;
relT.femur_l = absT.pelvis \ absT.femur_l;
relT.tibia_r = absT.femur_r \ absT.tibia_r;
relT.tibia_l = absT.femur_l \ absT.tibia_l;
relT.foot_r = absT.tibia_r \ absT.foot_r;
relT.foot_l = absT.tibia_l \ absT.foot_l;

%% Calculate joint angles

[x,y,z,t] = extract_angles_and_translation(relT.pelvis);
ikResults.pelvis_list = x;
ikResults.pelvis_rotation = y;
ikResults.pelvis_tilt = z;
ikResults.pelvis_tx = t(1);
ikResults.pelvis_ty = t(2);
ikResults.pelvis_tz = t(3);

[x,y,z,~] = extract_angles_and_translation(relT.femur_r);
ikResults.hip_adduction_r = x;
ikResults.hip_rotation_r = y;
ikResults.hip_flexion_r = z;

[x,y,z,~] = extract_angles_and_translation(relT.tibia_r);
ikResults.knee_adduction_r = x;
ikResults.knee_rotation_r = y;
ikResults.knee_flexion_r = z;

[x,y,z,~] = extract_angles_and_translation(relT.foot_r);
ikResults.ankle_adduction_r = x;
ikResults.ankle_rotation_r = y;
ikResults.ankle_flexion_r = z;

[x,y,z,~] = extract_angles_and_translation(relT.femur_l);
ikResults.hip_adduction_l = x;
ikResults.hip_rotation_l = y;
ikResults.hip_flexion_l = z;

[x,y,z,~] = extract_angles_and_translation(relT.tibia_l);
ikResults.knee_adduction_l = x;
ikResults.knee_rotation_l = y;
ikResults.knee_flexion_l = z;

[x,y,z,~] = extract_angles_and_translation(relT.foot_l);
ikResults.ankle_adduction_l = x;
ikResults.ankle_rotation_l = y;
ikResults.ankle_flexion_l = z;

clear x y z t
disp(['pelvis list: ' num2str(rad2deg(ikResults.pelvis_list)) ' °;']);
disp(['pelvis rotation: ' num2str(rad2deg(ikResults.pelvis_rotation)) ' °;']);
disp(['pelvis tilt: ' num2str(rad2deg(ikResults.pelvis_tilt)) ' °;']);
disp(['pelvis tx: ' num2str(ikResults.pelvis_tx) ' m;']);
disp(['pelvis ty: ' num2str(ikResults.pelvis_ty) ' m;']);
disp(['pelvis tz: ' num2str(ikResults.pelvis_tz) ' m;']);
disp(['right hip adduction: ' num2str(rad2deg(ikResults.hip_adduction_r)) ' °;']);
disp(['right hip rotation: ' num2str(rad2deg(ikResults.hip_rotation_r)) ' °;']);
disp(['right hip flexion: ' num2str(rad2deg(ikResults.hip_flexion_r)) ' °;']);
disp(['right knee adduction: ' num2str(rad2deg(ikResults.knee_adduction_r)) ' °;']);
disp(['right knee rotation: ' num2str(rad2deg(ikResults.knee_rotation_r)) ' °;']);
disp(['right knee flexion: ' num2str(rad2deg(ikResults.knee_flexion_r)) ' °;']);
disp(['right ankle adduction: ' num2str(rad2deg(ikResults.ankle_adduction_l)) ' °;']);
disp(['right ankle rotation: ' num2str(rad2deg(ikResults.ankle_rotation_l)) ' °;']);
disp(['right ankle flexion: ' num2str(rad2deg(ikResults.ankle_flexion_l)) ' °;']);
disp(['left hip adduction: ' num2str(rad2deg(ikResults.hip_adduction_l)) ' °;']);
disp(['left hip rotation: ' num2str(rad2deg(ikResults.hip_rotation_l)) ' °;']);
disp(['left hip flexion: ' num2str(rad2deg(ikResults.hip_flexion_l)) ' °;']);
disp(['left knee adduction: ' num2str(rad2deg(ikResults.knee_adduction_l)) ' °;']);
disp(['left knee rotation: ' num2str(rad2deg(ikResults.knee_rotation_l)) ' °;']);
disp(['left knee flexion: ' num2str(rad2deg(ikResults.knee_flexion_l)) ' °;']);
disp(['left ankle adduction: ' num2str(rad2deg(ikResults.ankle_adduction_l)) ' °;']);
disp(['left ankle rotation: ' num2str(rad2deg(ikResults.ankle_rotation_l)) ' °;']);
disp(['left ankle flexion: ' num2str(rad2deg(ikResults.ankle_flexion_l)) ' °;']);

%% Visualize the optimization results

screenSize = get(0, 'ScreenSize');
width = screenSize(3) * 0.8;
left = screenSize(3) * 0.1;
bottom = screenSize(4) * 0.1;
height = screenSize(4) * 0.8;
fig = figure('Position', [left bottom width height], 'Color', 'w');
set(fig, 'DoubleBuffer', 'on');
% nextFrameTime = 0;
% timescale = 0.01;
FramePerSec = trcFile.getDataRate;

mov = VideoWriter(['SkeletonPoseAnimation_' char(datetime('today'))]);
movFrameRate = FramePerSec;
mov.Quality = 100;
open(mov);

clear width left bottom height fig FramePerSec screenSize

for frame = 1 : height(ikResult)
    %% Calculate body poses according to IK input

    q = ikResult(frame,:);

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
    P = hip_r.translation_parent * scaleFactor.pelvis;
    R = eye(3);
    femur_r.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [femur_r]--> knee_r --[tibia_r]
    P = knee_r.translation_parent * scaleFactor.femur_r;
    R1 = Rot(hip_r.axis(1:3,1), q(dofMap('hip_flexion_r')));
    R2 = Rot(hip_r.axis(1:3,2), q(dofMap('hip_adduction_r')));
    R3 = Rot(hip_r.axis(1:3,3), q(dofMap('hip_rotation_r')));
    R = R*R1*R2*R3;
    tibia_r.T = [R, R*P+femur_r.T(1:3,4); 0 0 0 1];
    femur_r.T(1:3,1:3) = R;

    % [tibia_r]--> ankle_r --[foot_r]
    P = ankle_r.translation_parent * scaleFactor.tibia_r;
    R = R*Rot(knee_r.axis(1:3,1), q(dofMap('knee_flexion_r')));
    foot_r.T = [R, R*P+tibia_r.T(1:3,4); 0 0 0 1];
    tibia_r.T(1:3,1:3) = R;

    R1 = Rot(ankle_r.axis(1:3,1), q(dofMap('ankle_dorsiflexion_r')));
    R2 = Rot(ankle_r.axis(1:3,2), q(dofMap('ankle_adduction_r')));
    R = R*R1*R2;
    foot_r.T(1:3,1:3) = R;

    % ----- route 2 ----- %
    % [pelvis]--> hip_l --[femur_l]
    P = hip_l.translation_parent * scaleFactor.pelvis;
    R = eye(3);
    femur_l.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [femur_l]--> knee_l --[tibia_l]
    P = knee_l.translation_parent * scaleFactor.femur_l;
    R1 = Rot(hip_l.axis(1:3,1), q(dofMap('hip_flexion_l')));
    R2 = Rot(hip_l.axis(1:3,2), q(dofMap('hip_adduction_l')));
    R3 = Rot(hip_l.axis(1:3,3), q(dofMap('hip_rotation_l')));
    R = R*R1*R2*R3;
    tibia_l.T = [R, R*P+femur_l.T(1:3,4); 0 0 0 1];
    femur_l.T(1:3,1:3) = R;

    % [tibia_l]--> ankle_l --[foot_l]
    P = ankle_l.translation_parent * scaleFactor.tibia_l;
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
    P = back.translation_parent * scaleFactor.torso;
    R = eye(3);
    torso.T = [R, R*P+pelvis.T(1:3,4); 0 0 0 1];

    % [torso]--> neck --[head]
    P = neck.translation_parent * scaleFactor.torso;
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
    P = shoulder_r.translation_parent * scaleFactor.torso;
    R = torso.T(1:3,1:3);
    arm_r.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

    % [arm_r]--> elbow_r -- [forearm_r]
    P = elbow_r.translation_parent * scaleFactor.height;
    R1 = Rot(shoulder_r.axis(1:3,1), q(dofMap('arm_flex_r')));
    R2 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_add_r')));
    R3 = Rot(shoulder_r.axis(1:3,2), q(dofMap('arm_rot_r')));
    R = R*R1*R2*R3;
    forearm_r.T = [R, R*P+arm_r.T(1:3,4); 0 0 0 1];
    arm_r.T(1:3,1:3) = R;

    % [forearm_r]--> wrist_r -- [hand_r]
    P = wrist_r.translation_parent * scaleFactor.height;
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
    P = shoulder_l.translation_parent * scaleFactor.torso;
    R = torso.T(1:3,1:3);
    arm_l.T = [R, R*P+torso.T(1:3,4); 0 0 0 1];

    % [arm_l]--> elbow_l -- [forearm_l]
    P = elbow_l.translation_parent * scaleFactor.height;
    R1 = Rot(shoulder_l.axis(1:3,1), q(dofMap('arm_flex_l')));
    R2 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_add_l')));
    R3 = Rot(shoulder_l.axis(1:3,2), q(dofMap('arm_rot_l')));
    R = R*R1*R2*R3;
    forearm_l.T = [R, R*P+arm_l.T(1:3,4); 0 0 0 1];
    arm_l.T(1:3,1:3) = R;

    % [forearm_l]--> wrist_l -- [hand_l]
    P = wrist_l.translation_parent * scaleFactor.height;
    R1 = Rot(elbow_l.axis(1:3,1), q(dofMap('elbow_flex_l')));
    R2 = Rot(elbow_l.axis(1:3,2), q(dofMap('elbow_rot_l')));
    R = R*R1*R2;
    hand_l.T = [R, R*P+forearm_l.T(1:3,4); 0 0 0 1];
    forearm.T(1:3,1:3) = R;

    R = R*Rot(wrist_l.axis(1:3,1), q(dofMap('wrist_flex_l')));
    hand_l.T(1:3,1:3) = R;

    %% Calculate marker positions according to body poses and IK input

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

    %% plot
    % skeleton points
    % disp(['pelvis---hip_r---knee_r---ankle_r: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_r.T(1:3,4)') ']---[' num2str(tibia_r.T(1:3,4)') ']---[' num2str(foot_r.T(1:3,4)') ']']);
    % figure(1),
    hold off
    plot3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
        [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
        [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    % disp(['pelvis---hip_l---knee_l---ankle_l: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(femur_l.T(1:3,4)') ']---[' num2str(tibia_l.T(1:3,4)') ']---[' num2str(foot_l.T(1:3,4)') ']']);
    hold on
    scatter3([torso.T(3,4) femur_r.T(3,4) tibia_r.T(3,4) foot_r.T(3,4)], ...
        [torso.T(1,4) femur_r.T(1,4) tibia_r.T(1,4) foot_r.T(1,4)], ...
        [torso.T(2,4) femur_r.T(2,4) tibia_r.T(2,4) foot_r.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    plot3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
        [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
        [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
        'LineWidth', 2, 'Color', 'k');
    scatter3([torso.T(3,4) femur_l.T(3,4) tibia_l.T(3,4) foot_l.T(3,4)], ...
        [torso.T(1,4) femur_l.T(1,4) tibia_l.T(1,4) foot_l.T(1,4)], ...
        [torso.T(2,4) femur_l.T(2,4) tibia_l.T(2,4) foot_l.T(2,4)], ...
        'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % % disp(['pelvis---torso---head: [' num2str(pelvis.T(1:3,4)'), ']---[' num2str(torso.T(1:3,4)') ']---[' num2str(head.T(1:3,4)') ']']);
    % plot3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
    %     [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
    %     [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
    %     'LineWidth', 2, 'Color', 'k');
    % scatter3([torso.T(3,4) torso.T(3,4) head.T(3,4)], ...
    %     [torso.T(1,4) torso.T(1,4) head.T(1,4)], ...
    %     [torso.T(2,4) torso.T(2,4) head.T(2,4)], ...
    %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % % disp(['arm_r---forearm_r---hand_r: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_r.T(1:3,4)') ']---[' num2str(forearm_r.T(1:3,4)') ']---[' num2str(hand_r.T(1:3,4)') ']']);
    % plot3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
    %     [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
    %     [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
    %     'LineWidth', 2, 'Color', 'k');
    % scatter3([arm_r.T(3,4) forearm_r.T(3,4) hand_r.T(3,4)], ...
    %     [arm_r.T(1,4) forearm_r.T(1,4) hand_r.T(1,4)], ...
    %     [arm_r.T(2,4) forearm_r.T(2,4) hand_r.T(2,4)], ...
    %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);
    % % disp(['torso---arm_l---forearm_l---hand_l: [' num2str(torso.T(1:3,4)'), ']---[' num2str(arm_l.T(1:3,4)') ']---[' num2str(forearm_l.T(1:3,4)') ']---[' num2str(hand_l.T(1:3,4)') ']']);
    % plot3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
    %     [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
    %     [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
    %     'LineWidth', 2, 'Color', 'k');
    % scatter3([arm_r.T(3,4) arm_l.T(3,4) forearm_l.T(3,4) hand_l.T(3,4)], ...
    %     [arm_r.T(1,4) arm_l.T(1,4) forearm_l.T(1,4) hand_l.T(1,4)], ...
    %     [arm_r.T(2,4) arm_l.T(2,4) forearm_l.T(2,4) hand_l.T(2,4)], ...
    %     'LineWidth', 2, 'MarkerFaceColor',[0.8 0.2 0.2], 'MarkerEdgeColor',[0.8 0 0]);

    % virtual marker points
    scatter3(Sternum.location(3), Sternum.location(1), Sternum.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RAcromium.location(3), RAcromium.location(1), RAcromium.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAcromium.location(3), LAcromium.location(1), LAcromium.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RASIS.location(3), RASIS.location(1), RASIS.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LASIS.location(3), LASIS.location(1), LASIS.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighUpper.location(3), RThighUpper.location(1), RThighUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighFront.location(3), RThighFront.location(1), RThighFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RThighRear.location(3), RThighRear.location(1), RThighRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RKneeLat.location(3), RKneeLat.location(1), RKneeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RKneeMed.location(3), RKneeMed.location(1), RKneeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankUpper.location(3), RShankUpper.location(1), RShankUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankFront.location(3), RShankFront.location(1), RShankFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RShankRear.location(3), RShankRear.location(1), RShankRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RAnkleLat.location(3), RAnkleLat.location(1), RAnkleLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RAnkleMed.location(3), RAnkleMed.location(1), RAnkleMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RHeel.location(3), RHeel.location(1), RHeel.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RToeLat.location(3), RToeLat.location(1), RToeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(RToeTip.location(3), RToeTip.location(1), RToeTip.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighUpper.location(3), LThighUpper.location(1), LThighUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighFront.location(3), LThighFront.location(1), LThighFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LThighRear.location(3), LThighRear.location(1), LThighRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LKneeLat.location(3), LKneeLat.location(1), LKneeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LKneeMed.location(3), LKneeMed.location(1), LKneeMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankUpper.location(3), LShankUpper.location(1), LShankUpper.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankFront.location(3), LShankFront.location(1), LShankFront.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LShankRear.location(3), LShankRear.location(1), LShankRear.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAnkleLat.location(3), LAnkleLat.location(1), LAnkleLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LAnkleMed.location(3), LAnkleMed.location(1), LAnkleMed.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LHeel.location(3), LHeel.location(1), LHeel.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LToeLat.location(3), LToeLat.location(1), LToeLat.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    scatter3(LToeTip.location(3), LToeTip.location(1), LToeTip.location(2), 10, 'filled', 'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerEdgeColor', [0.2 0.6 0.9])
    % experimental marker points
    scatter3(markerPos_ref(frame,:,3), markerPos_ref(frame,:,1), markerPos_ref(frame,:,2), 10, 'filled', 'MarkerFaceColor', [0.2 0.9 0.6], 'MarkerEdgeColor', [0.2 0.9 0.6])

    axis equal, view(45,-30)
    xlim([-1 1]), ylim([-0.5 1.5]), zlim([-0.5 2]);
    xlabel('z'), ylabel('x'), zlabel('y');
    drawnow;
    F = getframe(gcf);
    writeVideo(mov, F);
end

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