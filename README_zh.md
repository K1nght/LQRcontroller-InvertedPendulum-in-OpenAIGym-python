# LQR控制一阶倒立摆

## Folder structure 
```
├─env            # Gym env implementation of Inverted Pendulum
├─figures        # save images and result gif
│  G.txt         # txt to save G
│  getGH.m       # matlab file to get G and H
│  H.txt         # txt to save H
│  LQR.py        # LQR generalized implementation
│  README.md     # README
│  test_gym.py   # Run LQR in the environment of Inverted Pendulum
│  utils.py      # Some public functions
```

## 动力学方程建立
一阶倒立摆系统的简化模型如图1所示，该模型由一个沿水平方向运动的小车以及与之相连接的单摆构成，其中的一些物理参数如下表所示。
（*图中 $\theta$是摆与竖直向下方向的夹角，代码中`theta`表示摆与竖直向上方向的夹角，即与下式推导中的 $\phi$相同；摆杆质量分布均匀，质心位于摆的中心位置。*）
<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="./figures/system%20model.jpg">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">图1 一阶倒立摆系统模型</div>
</center>

<center>

<table>
   <tr>
        <th> 属性 </th>
        <th> 值 </th>
   </tr>
   <tr>
        <td> 小车质量M </td>
        <td> 0.5 kg </td>
   </tr>
   <tr>
        <td> 小车受到的阻尼b </td>
        <td> 0.1 s^-1</td>
   </tr>
   <tr>
        <td> 摆的质量m </td>
        <td> 0.1 kg </td>
   </tr>
   <tr>
        <td> 摆的1/2长度l </td>
        <td> 0.3 m </td>
   </tr>
   <tr>
        <td> 摆的转动惯量I </td>
        <td> 0.012 kg*m^2 </td>
   </tr>
   <tr>
        <td> 重力加速度g </td>
        <td> 9.8 m/s^2 </td>
   </tr>
   <tr>
        <td> 采样周期tau </td>
        <td> 0.005 s </td>
   </tr>
</table>
</center>

分别对摆、车体建立动力学方程：
$$\begin{equation} 
(I+ml^2)\ddot{\theta}+mgl\sin \theta=-ml\ddot{x}\cos \theta 
\end{equation}$$

$$\begin{equation}
(M+m)\ddot{x}+b\dot{x}+ml\ddot{\theta}\cos \theta - ml\dot{\theta}^2\sin \theta = F 
\end{equation}$$
其中，$\theta$表示摆杆与竖直向下方向的夹角；$x$表示小车的位移

使用完整动力学方程(1)(2)离散化后，作为Gym仿真中的状态转移方程：
1. 假设当前时刻为k，则先通过当前时刻的输入$F(k)$和k-1时刻的$x(k-1), \dot{x}(k-1),\theta(k-1),\dot{\theta}(k-1)$计算当前时刻的$\ddot{x}(k),\ddot{\theta}(k)$:
$$
\begin{cases}
\ddot{\theta}(k)=(-g\sin\theta(k-1) - t\cos\theta(k-1)) / (\frac{I}{m} + l - \frac{ml\cos^2\theta(k-1)}{(M+m)})
\\
\ddot{x}(k)= t - \frac{ml\ddot{\theta}(k)\cos\theta(k-1)}{M+m}
\end{cases}
$$
其中$t=\frac{F(k) + ml[\dot{\theta}(k-1)]^2\sin\theta(k-1) - b\dot{x}(k-1)}{M+m}$

2. 通过当前时刻的$\ddot{x}(k),\ddot{\theta}(k)$计算当前时刻的所有状态量$x(k), \dot{x}(k),\theta(k),\dot{\theta}(k)$：
$$
\begin{cases}
x(k)=x(k-1)+\Delta t\cdot\dot{x}(k-1)
\\
\dot{x}(k)=\dot{x}(k-1)+\Delta t\cdot \ddot{x}(k)
\\
\theta(k)=\theta(k-1)+\Delta t\cdot\dot{\theta}(k-1)
\\
\dot{\theta}(k)=\dot{\theta}(k-1)+\Delta t\cdot\ddot{\theta}(k)
\end{cases}
$$

Gym方程环境程序中需要注意两个地方：
* 方程中$\theta$表示摆杆与竖直向下方向夹角，而程序中`theta`表示摆杆与竖直向下方向夹角，需要修改计算$\ddot{\theta}(k)$时为$(g\sin\theta(k-1) - \dots$，即第一项没有负号；

$$
\begin{cases}
\ddot{\phi}(k)=(-g\sin\phi(k-1) - t\cos\phi(k-1)) / (\frac{I}{m} + l - \frac{ml\cos^2\phi(k-1)}{(M+m)})
\\
\ddot{x}(k)= t - \frac{ml\ddot{\phi}(k)\cos\phi(k-1)}{M+m}
\end{cases}
$$

$$
t=\frac{F(k) + ml[\dot{\phi}(k-1)]^2\sin\phi(k-1) - b\dot{x}(k-1)}{M+m}
$$

$$
\begin{cases}
x(k)=x(k-1)+\Delta t\cdot\dot{x}(k-1)
\\
\dot{x}(k)=\dot{x}(k-1)+\Delta t\cdot \ddot{x}(k)
\\
\phi(k)=\phi(k-1)+\Delta t\cdot\dot{\phi}(k-1)
\\
\dot{\phi}(k)=\dot{\phi}(k-1)+\Delta t\cdot\ddot{\phi}(k)
\end{cases}
$$

* 使用增量方式计算`theta`时，在$\pi$也就是摆杆竖直向下附件需要考虑跳变，程序中`theta`(即图中$\phi$)数值对应的位置图示如下图2所示，
<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="figures\phi数值位置对应图.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">图2 角度位置对应图</div>
</center>

考虑跳变后对$\mathrm{theta}(k)$做如下约束：
$$
\mathrm{theta}(k)=
\begin{cases}
\mathrm{theta}(k)-2\pi, & \mathrm{theta}(k-1) < \pi\ and\ \mathrm{theta}(k) \geq \pi
\\ 
\mathrm{theta}(k)+2\pi, & \mathrm{theta}(k-1) > -\pi\ and\ \mathrm{theta}(k) \leq -\pi
\\ 
\mathrm{theta}(k), & otherwise
\end{cases}
$$

## 非线性动力学方程的线性化
由于上述非线性方程无法直接使用LQR直接进行状态空间建模和求解最优控制问题，因此需要对非线性动力学方程线性化。

令$\theta=\pi+\phi$，即$\phi=\theta-\pi$，选取状态变量$X=[\begin{array}{cccc}x & \dot{x} & \phi & \dot{\phi}\end{array}]^T$，在$X_0=[\begin{array}{cccc}0 & 0 & 0 & 0\end{array}]^T$附近线性化。得到线性化动力学方程：
$$\begin{equation}
(M+m)\ddot{x}+b\dot{x}-ml\ddot{\phi}=u
\end{equation}$$

$$\begin{equation}
(I+ml^2)\ddot{\phi}-mgl\phi=ml\ddot{x}
\end{equation}$$

## 一阶倒立摆系统的状态空间表达式
根据线性化的动力学方程，得到连续域状态空间表达式$\dot{X}=AX+Bu$，其中$X=[\begin{array}{cccc}x & \dot{x} & \phi & \dot{\phi}\end{array}]^T$，A，B矩阵如下式：
$$
A =
\left[\begin{matrix}
0 & 1 & 0 & 0 \\
0 & -\frac{(I+ml^2)b}{p} & \frac{m^2l^2g}{p} & 0\\
0 & 0 & 0 & 1 \\ 
0 & -\frac{mlb}{p} & \frac{mgl(M+m)}{p} & 0
\end{matrix}\right], 
\quad B=
\left[\begin{matrix}
0  \\
\frac{I+ml^2}{p}\\
0\\ 
\frac{ml}{p}
\end{matrix}\right]
\\
where,\ p=I(M+m)+Mml^2
$$

## LQR控制细节
* 得到连续状态空间方程$\dot{X}=AX+Bu$后，可以使用matlab`c2d`函数转变为离散化状态空间$\dot{X}(k+1)=Gx(k)+Hu(k)$
```Matlab
[G,H]=c2d(A,B,Ts) % Ts是采样周期
```

* LQR中$\mathbf{F}_t=[G\ H], \mathbf{f}_t=\mathbf{0}$，通过调参最终确定$\mathbf{C}_t=diag(10\ 15\ 30\ 6\ 1), \mathbf{c}_t=\mathbf{0}$；

* 基于状态反馈的动态LQR控制，按如下方式进行控制预测和实际控制：假设当前为k时刻，已知观测状态$X(k)$，以LQR控制时长T进行控制预测，但实际应用于控制的序列为前t步，即到k+t时刻则基于当时的观测状态$X(k+t)$再进行LQR控制预测之后k+t+1到k+t+T的控制输出，取T=100，t=15。

## 控制结果
由于是对非线性状态方程在**摆杆竖直向上附近线性化**得到的表达式进行LQR控制，因此选择初始状态时摆杆偏离竖直向上方向一个较小的值，经测试当初始状态$\phi(0)\in [-0.2\pi, 0.2\pi]$时，均可稳定收敛，$\phi(0)=0.2\pi$, $X_0=[\begin{array}{cccc}0 & 0 & 0.2\pi & 0\end{array}]^T$的结果如下所示：
<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="./figures/result.gif">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">图3 初始偏离竖直向上方向36°的LQR控制结果</div>
</center>
