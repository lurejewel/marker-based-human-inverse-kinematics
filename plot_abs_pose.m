function plot_abs_pose(absT)

% plot center of joints
xArray = [absT.pelvis(1,4), absT.femur_r(1,4), absT.femur_l(1,4), ...
    absT.tibia_r(1,4), absT.tibia_l(1,4), absT.foot_r(1,4), absT.foot_l(1,4)];
yArray = [absT.pelvis(2,4), absT.femur_r(2,4), absT.femur_l(2,4), ...
    absT.tibia_r(2,4), absT.tibia_l(2,4), absT.foot_r(2,4), absT.foot_l(2,4)];
zArray = [absT.pelvis(3,4), absT.femur_r(3,4), absT.femur_l(3,4), ...
    absT.tibia_r(3,4), absT.tibia_l(3,4), absT.foot_r(3,4), absT.foot_l(3,4)];
figure, scatter3(-zArray, -xArray, yArray,'filled');

hold on

% define foot
toeTip_r = absT.foot_r * [0.2 -0.04 0.008 1]';
toeTip_l = absT.foot_l * [0.2 -0.04 -0.008 1]';
heel_r = absT.foot_r * [-0.07 -0.04 0.008 1]';
heel_l = absT.foot_l * [-0.07 -0.04 -0.008 1]';

% plot skeletons
plot3(-[absT.pelvis(3,4)-0.1, absT.femur_r(3,4), absT.tibia_r(3,4), absT.foot_r(3,4), toeTip_r(3), heel_r(3), absT.foot_r(3,4)], ...
    -[absT.pelvis(1,4)-0.1, absT.femur_r(1,4), absT.tibia_r(1,4), absT.foot_r(1,4), toeTip_r(1), heel_r(1), absT.foot_r(1,4)], ...
    [absT.pelvis(2,4)-0.1, absT.femur_r(2,4), absT.tibia_r(2,4), absT.foot_r(2,4), toeTip_r(2), heel_r(2), absT.foot_r(2,4)], 'k--')
plot3(-[absT.pelvis(3,4)-0.1, absT.femur_l(3,4), absT.tibia_l(3,4), absT.foot_l(3,4), toeTip_l(3), heel_l(3), absT.foot_l(3,4)], ...
    -[absT.pelvis(1,4)-0.1, absT.femur_l(1,4), absT.tibia_l(1,4), absT.foot_l(1,4), toeTip_l(1), heel_l(1), absT.foot_l(1,4)], ...
    [absT.pelvis(2,4)-0.1, absT.femur_l(2,4), absT.tibia_l(2,4), absT.foot_l(2,4), toeTip_l(2), heel_l(2), absT.foot_l(2,4)], 'k--')

% plot axes
% pelvis
xAxis = absT.pelvis * [0.05 0 0 1]';
yAxis = absT.pelvis * [0 0.05 0 1]';
zAxis = absT.pelvis * [0 0 0.05 1]';
plot3(-[absT.pelvis(3,4), xAxis(3)], -[absT.pelvis(1,4), xAxis(1)], [absT.pelvis(2,4) xAxis(2)], 'r');
plot3(-[absT.pelvis(3,4), yAxis(3)], -[absT.pelvis(1,4), yAxis(1)], [absT.pelvis(2,4) yAxis(2)], 'g');
plot3(-[absT.pelvis(3,4), zAxis(3)], -[absT.pelvis(1,4), zAxis(1)], [absT.pelvis(2,4) zAxis(2)], 'b');
% femur_r
xAxis = absT.femur_r * [0.05 0 0 1]';
yAxis = absT.femur_r * [0 0.05 0 1]';
zAxis = absT.femur_r * [0 0 0.05 1]';
plot3(-[absT.femur_r(3,4), xAxis(3)], -[absT.femur_r(1,4), xAxis(1)], [absT.femur_r(2,4) xAxis(2)], 'r');
plot3(-[absT.femur_r(3,4), yAxis(3)], -[absT.femur_r(1,4), yAxis(1)], [absT.femur_r(2,4) yAxis(2)], 'g');
plot3(-[absT.femur_r(3,4), zAxis(3)], -[absT.femur_r(1,4), zAxis(1)], [absT.femur_r(2,4) zAxis(2)], 'b');
% tibia_r
xAxis = absT.tibia_r * [0.05 0 0 1]';
yAxis = absT.tibia_r * [0 0.05 0 1]';
zAxis = absT.tibia_r * [0 0 0.05 1]';
plot3(-[absT.tibia_r(3,4), xAxis(3)], -[absT.tibia_r(1,4), xAxis(1)], [absT.tibia_r(2,4) xAxis(2)], 'r');
plot3(-[absT.tibia_r(3,4), yAxis(3)], -[absT.tibia_r(1,4), yAxis(1)], [absT.tibia_r(2,4) yAxis(2)], 'g');
plot3(-[absT.tibia_r(3,4), zAxis(3)], -[absT.tibia_r(1,4), zAxis(1)], [absT.tibia_r(2,4) zAxis(2)], 'b');
% foot_r
xAxis = absT.foot_r * [0.05 0 0 1]';
yAxis = absT.foot_r * [0 0.05 0 1]';
zAxis = absT.foot_r * [0 0 0.05 1]';
plot3(-[absT.foot_r(3,4), xAxis(3)], -[absT.foot_r(1,4), xAxis(1)], [absT.foot_r(2,4) xAxis(2)], 'r');
plot3(-[absT.foot_r(3,4), yAxis(3)], -[absT.foot_r(1,4), yAxis(1)], [absT.foot_r(2,4) yAxis(2)], 'g');
plot3(-[absT.foot_r(3,4), zAxis(3)], -[absT.foot_r(1,4), zAxis(1)], [absT.foot_r(2,4) zAxis(2)], 'b');
% femur_l
xAxis = absT.femur_l * [0.05 0 0 1]';
yAxis = absT.femur_l * [0 0.05 0 1]';
zAxis = absT.femur_l * [0 0 0.05 1]';
plot3(-[absT.femur_l(3,4), xAxis(3)], -[absT.femur_l(1,4), xAxis(1)], [absT.femur_l(2,4) xAxis(2)], 'r');
plot3(-[absT.femur_l(3,4), yAxis(3)], -[absT.femur_l(1,4), yAxis(1)], [absT.femur_l(2,4) yAxis(2)], 'g');
plot3(-[absT.femur_l(3,4), zAxis(3)], -[absT.femur_l(1,4), zAxis(1)], [absT.femur_l(2,4) zAxis(2)], 'b');
% tibia_l
xAxis = absT.tibia_l * [0.05 0 0 1]';
yAxis = absT.tibia_l * [0 0.05 0 1]';
zAxis = absT.tibia_l * [0 0 0.05 1]';
plot3(-[absT.tibia_l(3,4), xAxis(3)], -[absT.tibia_l(1,4), xAxis(1)], [absT.tibia_l(2,4) xAxis(2)], 'r');
plot3(-[absT.tibia_l(3,4), yAxis(3)], -[absT.tibia_l(1,4), yAxis(1)], [absT.tibia_l(2,4) yAxis(2)], 'g');
plot3(-[absT.tibia_l(3,4), zAxis(3)], -[absT.tibia_l(1,4), zAxis(1)], [absT.tibia_l(2,4) zAxis(2)], 'b');
% foot_l
xAxis = absT.foot_l * [0.05 0 0 1]';
yAxis = absT.foot_l * [0 0.05 0 1]';
zAxis = absT.foot_l * [0 0 0.05 1]';
plot3(-[absT.foot_l(3,4), xAxis(3)], -[absT.foot_l(1,4), xAxis(1)], [absT.foot_l(2,4) xAxis(2)], 'r');
plot3(-[absT.foot_l(3,4), yAxis(3)], -[absT.foot_l(1,4), yAxis(1)], [absT.foot_l(2,4) yAxis(2)], 'g');
plot3(-[absT.foot_l(3,4), zAxis(3)], -[absT.foot_l(1,4), zAxis(1)], [absT.foot_l(2,4) zAxis(2)], 'b');

axis equal, xlabel('-z'), ylabel('-x'), zlabel('y');

end