# 刚体运动姿态控制

## 1.姿态矩阵及误差矩阵
当前姿态矩阵$\mathbf{R}$，目标姿态矩阵$\mathbf{R_d}$，误差姿态矩阵$\mathbf{R_e}$，若

情况1：
$$
\begin{equation}
    \label{eq:1}
    \mathbf{R_d} \mathbf{R_e} = \mathbf{R}.
\end{equation}
$$

该式子以目标姿态矩阵为参考定义误差矩阵$\mathbf{R_e}$。

情况2：
$$
\begin{equation}
    \label{eq:2}
    \mathbf{R} \mathbf{R_e} = \mathbf{R_d}.
\end{equation}
$$
该式子以当前姿态矩阵为参考定义误差矩阵$\mathbf{R_e}$。

$\bf{REMARK\ 1：}$
由于$\mathbf{R}$和$\mathbf{R_d}$都是姿态矩阵，因此$\mathbf{R_e}$也是姿态矩阵。

$\bf{REMARK\ 2：}$
左乘和右乘的理解——右乘的理解很简单，就是在$\mathbf{R}$上再进行旋转变换$\mathbf{R_e}$。将$\mathbf{R_e}$看成基，右乘上坐标向量就是向量在$\mathbf{R_e}$下的分量。再左乘$\mathbf{R}$表示基$\mathbf{R_e}$是基于$\mathbf{R}$做的旋转变换。换句话说，先从惯性系下的基$\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$转换为$\mathbf{R}$，再将惯性系下的基$\mathbf{R}$旋转$\mathbf{R_e}$。因此，可以看出左乘旋转矩阵就是把坐标系在惯性系下旋转，右乘旋转矩阵就是在当前坐标系下旋转。

$\bf{REMARK\ 3：}$
左乘和右乘存在这样的定义，其原因是对于姿态矩阵$\mathbf{R}$，是将其列向量当作基。
$$
\begin{equation}
    \label{eq:3}
    \begin{aligned}
        \begin{bmatrix}
            R_1 & R_2 & R_3
        \end{bmatrix}
        & = R_1 \times x + R_2 \times y + R_3 \times z \\
        & = \mathbf{R} \begin{bmatrix}
            x \\
            0 \\
            0
        \end{bmatrix} + \mathbf{R} \begin{bmatrix}
            0 \\
            y \\
            0
        \end{bmatrix} + \mathbf{R} \begin{bmatrix}
            0 \\
            0 \\
            z
        \end{bmatrix}
    \end{aligned}
\end{equation}
$$
因此，对基进行变换的旋转矩阵的列向量就是新基在老基下的坐标。通过不断的右乘旋转矩阵，就是不断地在前一个基的基础上变换基。而一连串的旋转矩阵相乘的结果，表示最终得到的基在惯性系的基$\begin{bmatrix} 1 & 0 & 0\\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$的坐标。

做姿态控制时，常选用\eqref{eq:2}的旋转矩阵表示法，可以直接获得当前姿态矩阵（参考坐标系）下的误差。

$$
\begin{equation}
    \label{eq:4}
    \mathbf{R_e} = \mathbf{R}^T \mathbf{R_d}.
\end{equation}
$$

## 2.误差矩阵映射为向量

误差矩阵$\mathbf{R_e}$可以通过\eqref{eq:5}映射为旋转向量$\mathbf{e_\theta}$，其模为旋转角度$\theta$，方向为旋转轴$\mathbf{e}$。
$$
\begin{equation}
    \label{eq:5}
    \left[\mathbf{e_\theta}\right]_\times = \log(\mathbf{R_e}).
\end{equation}
$$
其中$\left[\mathbf{e_\theta}\right]_\times$表示反对称矩阵。$\mathbf{e_\theta}$表示误差向量。

$\bf{REMARK\ 4：}$
以$\mathbf{e_\theta}$作为误差设计PID控制器的方法，可以称作基于SO(3)设计方法。SO(3)是什么？SO(3)是三维特殊正交群（Special Orthogonal Group）的数学符号。你可以把它通俗地理解为：所有满足“右手定则”且不改变物体形状的三维旋转矩阵的集合。一个$3\times3$矩阵$\mathbf{R}$属于SO(3)，必须满足：（1）正交性：$R^T R = I$；（2）行列式为$+1$：$\mathrm{det}(\mathbf{R}) = +1$。

SO(3)空间是一个流形（Manifold），通俗说就是一个弯曲的“曲面”，而不是平坦的向量空间。这意味着在 SO(3) 上的两个矩阵相加，结果不再是一个合法的旋转矩阵（就像球面上的两个点相加会飘到球外一样）。

$\bf{REMARK\ 5：}$
为什么要用矩阵对数？矩阵对数（Matrix Logarithm）是矩阵指数（Matrix Exponential）的逆运算。在姿态控制中，它的作用极其明确：把弯曲流形 SO(3) 上的旋转矩阵，映射到平坦的向量空间（即我们熟悉的x, y, z三维轴角向量）中。