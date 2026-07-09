function v_clipped = uniformScaleClip(v, L, U)
% UNIFORMSCALECLIP 对向量进行等比例缩放限幅，保持各维度比例
%   v_clipped = uniformScaleClip(v, L, U)
%
%   输入：
%       v : 数值向量（行或列）或矩阵（每列为一个待处理向量）
%       L : 下界，可为标量（所有维度共用）或与 v 同维度的向量（逐维独立）
%       U : 上界，格式同 L
%
%   输出：
%       v_clipped : 限幅后的数据，尺寸与 v 相同
%
%   算法：
%       计算全局缩放系数 k = min(1, min(U_i/v_i) for v_i>U_i, min(L_i/v_i) for v_i<L_i)
%       若 k < 1，则 v_clipped = k * v；否则保持不变。
%
%   示例：
%       % 单向量，全局上下界
%       v = [10; -5; 1; 2; 3; 4];
%       L = -2; U = 5;
%       vc = uniformScaleClip(v, L, U)
%
%       % 各维度独立边界
%       L_vec = [-1, -3, 0, -2, -4, -1];
%       U_vec = [2, 5, 3, 4, 6, 2];
%       vc = uniformScaleClip(v, L_vec, U_vec)

    % 确保 v 是列向量或矩阵（每列一个向量）
    if isrow(v)
        v = v(:);   % 转为列向量（若为单行）
    end
    % 若 v 为矩阵，后续按列处理

    % 扩展 L 和 U 为与 v 同样尺寸（若为标量则复制）
    if isscalar(L)
        L = L * ones(size(v));
    elseif isrow(L)
        L = L(:);   % 转为列向量，若尺寸与 v 不匹配则报错
    end
    if isscalar(U)
        U = U * ones(size(v));
    elseif isrow(U)
        U = U(:);
    end

    % 尺寸检查
    if ~isequal(size(v), size(L)) || ~isequal(size(v), size(U))
        error("The sizes of v, L, and U must match, or L/U must be scalars.");
    end

    % 计算全局缩放系数 k（初始为1）
    k = 1.0;

    % 上界越界：v > U
    idxU = (v > U);
    if any(idxU(:))
        ratiosU = U(idxU) ./ v(idxU);   % 均为正数
        k = min(k, min(ratiosU));
    end

    % 下界越界：v < L
    idxL = (v < L);
    if any(idxL(:))
        ratiosL = L(idxL) ./ v(idxL);   % L和v均为负，比值正
        k = min(k, min(ratiosL));
    end

    % 应用缩放
    if k < 1
        v_clipped = k * v;
    else
        v_clipped = v;
    end
end