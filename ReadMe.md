

## Industrial Robot Linear Inverted Pendulum*



**workflow**


>Status Input => Kalman Filter ==>  Linear Inverted Pendulum Control(LQR) ==> Robot Planning ==> Robot & Inverted Pendulum Execution ==> Status Output


## Kalman Filter


## Linear Inverted Pendulum Control(LQR)


Algorithm: LQR

Input: x, $\dot{x}$, θ, $\dot{θ}$

Output: F(or call it u)

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
    \dot{x} \\
    \ddot{x} \\
    \dot{θ}  \\
    \ddot{θ}
\end{bmatrix} = 

\begin{bmatrix}
    0 & 1 & 0 & 0 \\
    0 & 0  & \frac{-m^2l^2g}{(J+ml^2)(M+m)-m^2l^2} & 0 \\
    0 & 0 & 0 & 1 \\
    0 & 0 & \frac{(M+m)mgl}{(J+ml^2)(M+m)-m^2l^2} & 0
\end{bmatrix} 

\begin{bmatrix}
    x \\
    \dot{x} \\
    θ \\
    \dot{θ}
\end{bmatrix} +

\begin{bmatrix}
    0 \\
    \frac{J+ml^2}{(J+ml^2)(M+m)-m^2l^2} \\
    0 \\
    \frac{-ml}{(J+ml^2)(M+m)-m^2l^2}
\end{bmatrix}F \ (7)
$$


$$
Y=
\begin{bmatrix}
    x \\
    θ
\end{bmatrix}=

\begin{bmatrix}
    1 & 0 & 0 & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}

\begin{bmatrix}
    x \\
    \dot{x} \\
    θ \\
    \dot{θ}+ 
\end{bmatrix}  + 

\begin{bmatrix}
    0 \\
    0 \\
\end{bmatrix}F \ (8)
$$

if we use equations (5) and (6), we can get State Fuctions:


$$

\dot{X}= 
\begin{bmatrix}
    \dot{x} \\
    \ddot{x} \\
    \dot{θ}  \\
    \ddot{θ}
\end{bmatrix} = 

\begin{bmatrix}
    0 & 1 & 0 & 0 \\
    0 & 0  & \frac{-3mg}{4M+m} & 0 \\
    0 & 0 & 0 & 1 \\
    0 & 0 & \frac{3(M+m)gθ}{(4M+m)l} & 0
\end{bmatrix} 

\begin{bmatrix}
    x \\
    \dot{x} \\
    θ \\
    \dot{θ}
\end{bmatrix} +

\begin{bmatrix}
    0 \\
    \frac{4}{4M+m} \\
    0 \\
    \frac{-3}{(4M+m)l}
\end{bmatrix}F \ (9)

$$


$$
Y=
\begin{bmatrix}
    x \\
    θ
\end{bmatrix}=

\begin{bmatrix}
    1 & 0 & 0 & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}

\begin{bmatrix}
    x \\
    \dot{x} \\
    θ \\
    \dot{θ}+ 
\end{bmatrix}  + 

\begin{bmatrix}
    0 \\
    0 \\
\end{bmatrix}F \ (10)
$$

In program, we use equations (7) and (8).

## Robot Planning