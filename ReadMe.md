

```
To display the formula correctly, please view this page under Chrome with MathJax extension installed
```


## Industrial Robot Linear Inverted Pendulum*



**workflow**


>Status Input => Kalman Filter ==>  Linear Inverted Pendulum Control(LQR) ==> Robot Planning ==> Robot & Inverted Pendulum Execution ==> Status Output


## Kalman Filter

本案例中用到卡尔曼滤波的目的是，估算LQR的输入的$\dot{θ}$。因为我们没有倒立摆的摆动加速度测量传感器。而$\dot{x}$，是直接用TCP值计算的，因为编码器的精度很高。

## Linear Inverted Pendulum Control(LQR)


Algorithm: LQR

> 直线一阶倒立摆系统研究_尹贻晓.caj <br>
LQR(Linear quadratic regulator,　线性二次型调节器)的控制对象是现代控制理论中以状态空间的形式给出的线性系统，它的目标函数是对象状态和控制输入的二次型函数。状态反馈控制器Ｋ由权重矩阵Ｑ和Ｒ唯一确定，而线性二次型调节器最优化设计就是要求设计处的状态反馈控制器Ｋ要使得二次型目标函数Ｊ取得最小值，所以参数Ｑ和Ｒ的选择非常重要。

>  基于dSPACE仿真平台的一阶直线倒立摆控制研究_任玲 <br>
本章以系统全状态变量反馈为基本策略，设计LQR控制器(线性二次型最有控制算法)的基本思想为：在满足一定的性能指标要去下，利用状态反馈方法，再消耗能量金小的情况下，使得系统状态的动态性能最佳、状态误差最小。<br>
LQR最优控制算法属于线性反馈的结构，在实际倒立摆控制系统中得到了广泛的应用，具有一定的鲁棒性，动态特性较好。


> 总结 <br>
LQR是能量最优算法，J包含两项＝状态误差累计项＋控制量的动态性能项目（具体见下文）
输入是全状态反馈输入量(即下文的大Ｘ)，输出是倒立摆的控制量Ｆ，这个Ｆ可以让整个J取得最小值。虽然是能量最低，但是通过调试Ｋ参数，可以调试上升时间等。简单说就是针对Ｊ优化，构建汉密尔顿方程Ｈ，然后对Ｈ控制量ｕ求偏导数，并令偏导数＝０，即可求得控制量ｕ(F)，此时的Ｊ也是最小的。



Input: realTime x, $\dot{x}$, θ, $\dot{θ}$ and reference/aimed x0, $\dot{x}0$, θ0, $\dot{θ}0$ (to check later)

Output: F(or call it u)(optimal F to let J minimal)

Notice : F is proportional to acceleration


Ignore air resistance and cart floor friction, we get "cart-pole" model:


$$

\ddot{x} = \frac{(J + ml^2)F + (ml^2+J)mlsin(θ)(\dot{θ})^2-m^2l^2gsin(θ)cos(θ)}{(J+ml^2)(M+m)-m^2l^2cos(θ)}   \ (1)

$$


$$

\ddot{θ} = -\frac{mlcos(θ)F+m^2l^2(\dot{θ})^2sin(θ)cos(θ)-(M+m)mglsin(θ)}{(J+ml^2)(M+m)-m^2l^2cos(θ)} \ (2)

$$


Let's say θ is small and linearize it as:


$\begin{cases}θ \ is \  small \ but  \ not \ zero \\\\sin(θ)\approxθ\\\\cos(θ)\approx1\\\\(\dot{θ})^2\approx0\end{cases}$

back substitute the above assumptions to equation(1) and (2), we  get


$$

\ddot{x} = \frac{(J + ml^2)F -m^2l^2gθ}{(J+ml^2)(M+m)-m^2l^2}   \ (3)

$$


$$

\ddot{θ} = -\frac{mlF-(M+m)mglθ}{(J+ml^2)(M+m)-m^2l^2} \ (4)

$$


if $J=\frac{1}{3}ml^2$, then we can get 

$$

\ddot{x} = \frac{4F-3mgθ}{4M+m}   \ (5)

$$


$$

\ddot{θ} = \frac{3(M+m)gθ-3F}{(4M+m)l} \ (6)

$$

Linear Inverted Pendulum State Function:

$\begin{cases}\dot{X} = A*X+B*U\\\\Y = C*X+D*U\end{cases}$


where:

$\begin{cases}X=[x, \dot{x},θ, \dot{θ}]\\\\\dot{X}=[ \dot{x}, \ddot{x},\dot{θ}, \ddot{θ}]\\\\Y=[x, θ]\\\\U = u \ or \ F\end{cases}$


if we use equations (3) and (4), we can get State Fuctions:

$$
\dot{X}= 
\begin{bmatrix}
    \dot{x} \\\\
    \ddot{x} \\\\
    \dot{θ}  \\\\
    \ddot{θ}
\end{bmatrix} = 

\begin{bmatrix}
    0 & 1 & 0 & 0 \\\\
    0 & 0  & \frac{-m^2l^2g}{(J+ml^2)(M+m)-m^2l^2} & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    0 & 0 & \frac{(M+m)mgl}{(J+ml^2)(M+m)-m^2l^2} & 0
\end{bmatrix} 

\begin{bmatrix}
    x \\\\
    \dot{x} \\\\
    θ \\\\
    \dot{θ}
\end{bmatrix} +

\begin{bmatrix}
    0 \\\\
    \frac{J+ml^2}{(J+ml^2)(M+m)-m^2l^2} \\\\
    0 \\\\
    \frac{-ml}{(J+ml^2)(M+m)-m^2l^2}
\end{bmatrix}F \ (7)
$$


$$
Y=
\begin{bmatrix}
    x \\\\
    θ
\end{bmatrix}=

\begin{bmatrix}
    1 & 0 & 0 & 0 \\\\
    0 & 0 & 1 & 0
\end{bmatrix}

\begin{bmatrix}
    x \\\\
    \dot{x} \\\\
    θ \\\\
    \dot{θ}+ 
\end{bmatrix}  + 

\begin{bmatrix}
    0 \\\\
    0 
\end{bmatrix}F \ (8)
$$

if we use equations (5) and (6), we can get State Fuctions:


$$

\dot{X}= 
\begin{bmatrix}
    \dot{x} \\\\
    \ddot{x} \\\\
    \dot{θ}  \\\\
    \ddot{θ}
\end{bmatrix} = 

\begin{bmatrix}
    0 & 1 & 0 & 0 \\\\
    0 & 0  & \frac{-3mg}{4M+m} & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    0 & 0 & \frac{3(M+m)gθ}{(4M+m)l} & 0
\end{bmatrix} 

\begin{bmatrix}
    x \\\\
    \dot{x} \\\\
    θ \\\\
    \dot{θ}
\end{bmatrix} +

\begin{bmatrix}
    0 \\\\
    \frac{4}{4M+m} \\\\
    0 \\\\
    \frac{-3}{(4M+m)l}
\end{bmatrix}F \ (9)

$$


$$
Y=
\begin{bmatrix}
    x \\\\
    θ
\end{bmatrix}=

\begin{bmatrix}
    1 & 0 & 0 & 0 \\\\
    0 & 0 & 1 & 0
\end{bmatrix}

\begin{bmatrix}
    x \\\\
    \dot{x} \\\\
    θ \\\\
    \dot{θ}+ 
\end{bmatrix}  + 

\begin{bmatrix}
    0 \\\\
    0 
\end{bmatrix}F \ (10)
$$

In program, we use equations (7) and (8).

## Robot Planning

LQR计算出来的是ｕ或者说加速度，需要积分到ｐ(xyz)，然后在笛卡尔直线约束下，下发点位给机器人执行。中间涉及到的是：逆解为J1~J6,然后笛卡尔约束下的下发J1~J6。


