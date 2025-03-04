function [alpha, beta, gamma, translation] = extract_angles_and_translation(T)
% 输入：齐次变换矩阵 T (4x4)
% 输出：旋转角度 angles (ZYX角，单位：弧度)，平移量 translation (3x1向量)

% 提取旋转矩阵 R
R = T(1:3, 1:3);

% 提取平移向量 t
translation = T(1:3, 4);

% 计算 ZYX 角
alpha = atan2(R(3, 2), R(3, 3)); % 绕 X 轴旋转角度
beta = atan2(-R(3, 1), sqrt(R(1, 1)^2 + R(2, 1)^2)); % 绕 Y 轴旋转角度
gamma = atan2(R(2, 1), R(1, 1)); % 绕 Z 轴旋转角度

end