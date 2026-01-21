<!-- ---
math:
  enable: true
  engine: mathjax
  mathjax:
    tex-extensions:
      - autoload-all.js
    tags: 'ams'
--- -->

# <center>Stability analysis of aerial vehicle controller
<center>Dzxion</center>

<!-- Abstract——电控系统是目前工业界最常见的控制系统。据我观察，目前大部分资料对电控系统的介绍是描述自身特定的算法框架，并没有给出系统稳定性的分析结果。目前大部分资料并不能从数学的层面上把电控系统的基本性质描述清楚，并且许多从业人员对控制理论的基本概念是不理解的。因此，本文从控制理论的角度，以动力电调作为例子，推导了动力电调开环和闭环稳定性分析结果，以说明控制理论是如何解决问题的。 -->

[toc]

## Introduction

<!-- ### Dynamical Systems

控制理论本质上是研究微分方程/动力系统（dynamical systems）。

$$\dot{x}_1 = f_1(t,x_1,\cdots,x_n,u_1,\cdots,u_p)\\
\dot{x}_2 = f_2(t,x_1,\cdots,x_n,u_1,\cdots,u_p)\\
 \vdots \\
\dot{x}_n = f_n(t,x_1,\cdots,x_n,u_1,\cdots,u_p)$$

上式是由有限个耦合的一阶常微分方程建模的动力系统。其中 $\dot{x}_i$ 表示 $x_i$ 对于时间变量 $t$ 的导数，$u_1,u_2,\cdots,u_p$ 是输入变量。$x_1,x_2,\cdots,x_p$ 是状态变量。我们通常会使用向量将方程写成紧凑的形式。定义如下：

$$x=\left[\begin {array}{c}
x_1 \\
x_2 \\
\vdots \\
\vdots \\
x_n
\end{array}\right],
u=\left[\begin {array}{c}
u_1 \\
u_2 \\
\vdots \\
u_p
\end{array}\right],
f(t,x,u)=\left[\begin {array}{c}
f_1(t,x,u) \\
f_2(t,x,u) \\
\vdots \\
\vdots \\
f_n(t,x,u)
\end{array}\right]$$
将n个一阶微分方程改写为一个n维的一阶向量微分方程
$$\dot{x} = f(t,x,u)\tag{1.1}$$
我们将（1.1）统称为状态空间模型。如果系统不存在输入且是时不变的，方程简化为
$$\dot{x} = f(x)\tag{1.2}$$ -->

<!-- ### Equilibrium Point

状态空间的一个重要概念是平衡点（equilibrium point）。如果状态空间中的点 $x = x^*$ 具有以下特性：每当系统的状态从 $x^*$ 出发，它将在未来所有时间保持在 $x^*$ 时，则称其为（1.2）的平衡点。对于时不变系统（1.2），平衡点是方程的实根
$$f(x)=0$$

### Lyapunov Stability
Lyapunov稳定性定义：
![](微信图片_20250318200401.png)

![](微信图片_20250318202751.png)

Lyapunov稳定性分析方法：
![](微信图片_20250318204823.png)

![](微信图片_20250323090556.png)

### Stabilization

镇定问题的定义，对于系统
$$\dot{x} = f(t,x,u)$$
设计一个反馈控制律
$$u = \gamma(t,x)$$
使得原点 $x = 0$ 是如下闭环系统的渐近稳定平衡点
$$\dot{x} = f(t,x,\gamma(t,x))$$ -->

## Problem Formulation

### Notation

* $x$ 记为飞行器的位置
* $v$ 记为飞行器的速度
* $m$ 记为飞行器的质量
* $T$ 记为飞行器的推力大小
* $R$ 记为飞行器的旋转矩阵，表示机体系在世界系的投影
* $e_3$ 记为世界系z轴基向量
* $g$ 记为重力加速度
* $\Omega$ 记为机体角速度
* $sk(\cdot)$ 记为3维向量到3$\times$3反对称矩阵的映射
* $vex(\cdot)$ 记为3$\times$3反对称矩阵对3维向量的映射
<!-- * $\phi_m$ 记为磁链常数
* $n$ 记为极对数
* $T_e$ 记为电磁扭矩
* $T_L$ 记为电磁扭矩
* $v_q$ 记为$q$轴输入电压
* $v_d$ 记为$d$轴输入电压 -->

### Dynamic
系统动力学表示如下：
$$
\begin{equation}
\begin{aligned}
\dot{x} &= v\\
m\dot{v} &= -TRe_3 + mge_3\\
\dot{R} &= Rsk(\Omega)\\
% \dot{i}_q &= -nw\frac{L_d}{L_q}i_d - \frac{R}{L_q}i_q-\frac{n\phi_m}{L_q}w+\frac{1}{L_q}v_q
% \tag{2.1}
\end{aligned}
% \label{eq:energy-mass}
\end{equation}
$$
<!-- 基于上述模型，做进一步简化，我们令$i_d\equiv0$，方程如下：
$$
\begin{aligned}
\dot{w} &= -\frac{B}{J}w+\frac{K_t}{J}i_q-\frac{1}{J}T_L\\
\dot{i}_q &= - \frac{R}{L_q}i_q-\frac{n\phi_m}{L_q}w+\frac{1}{L_q}v_q\tag{2.2}
\end{aligned}
$$
考虑动力电调的负载扭矩特性，系统可进一步变为
$$
\begin{aligned}
\dot{w} &= -\frac{B}{J}w+\frac{K_t}{J}i_q-\frac{c}{J}w^2\\
\dot{i}_q &= - \frac{R}{L_q}i_q-\frac{n\phi_m}{L_q}w+\frac{1}{L_q}v_q\tag{2.3}
\end{aligned}
$$ -->

## Control Design

飞行器的控制目标：
* 位置误差镇定

定义参考位置 $x_r$ 及其导数 $v_r = \dot{x}_r$，误差位置变量 $\widetilde{x} = x - x_r$，假设速度 $v$ 为虚拟控制变量，定义控制参数$k>0$，控制律设计如下：
$$
\begin{equation}
\begin{aligned}
v = -k\widetilde{x} + v_r\\
\end{aligned}
\end{equation}
% \tag{3.1}
$$
* 速度误差镇定

定义参考速度 $v_r$ 及其导数 $a_r = \dot{v}_r$，误差速度变量 $\widetilde{v} = v - v_r$，假设推力 $T$ 和旋转矩阵 $R$ 为虚拟控制变量，定义控制参数$k>0$，控制律设计如下：
$$
\begin{equation}
\begin{aligned}
(TRe_3)_r = mge_3+k\widetilde{v} - ma_r\\
\end{aligned}
\end{equation}
% \tag{3.1}
$$
* 姿态误差镇定

定义参考速度 $R_d$ 及其导数 $\Omega_r$，误差姿态变量 $\widetilde{R} = R^TR_d$，假设角速度 $\Omega$ 为虚拟控制变量，定义控制参数$k>0$，定义运算：
$$
\begin{equation}
\begin{aligned}
\pi_a\widetilde{R} = \frac{\widetilde{R}-\widetilde{R}^T}{2}\\
\end{aligned}
\end{equation}
% \tag{3.1}
$$
控制律设计如下：
$$
\begin{equation}
\begin{aligned}
\Omega = -kvex(\pi_a\widetilde{R}^T) + \Omega_r\\
\end{aligned}
\end{equation}
% \tag{3.1}
$$
## Stability Analysis

本节中，将给出飞行器闭环系统的稳定性结果。
<!-- 设开环系统的平衡点$(w,i_q,v_q)=(w^*,i_q^*,v_q^∗)=(w^*,i_q^*,v_q^{ref})$。开环系统平衡点由下式求得
$$
\begin{aligned}
0 &= -\frac{B}{J}w^*+\frac{K_t}{J}i_q^*-\frac{c}{J}{w^*}^2\\
0 &= - \frac{R}{L_q}i_q^*-\frac{n\phi_m}{L_q}w^*+\frac{1}{L_q}v_q^*\tag{4.1}
\end{aligned}
$$
虽然标准的镇定问题被定义为平衡点在原点的镇定，但我们可以使用相同的描述来镇定系统在任意点 $x^*$。通过坐标变换
$$w^\delta=w-w^*,i_q^\delta=i_q-i_q^*,v_q^\delta=v_q-v_q^*$$
结合式（2.3）和（4.1）不难得出误差系统方程
$$
\begin{aligned}
\dot{w}^\delta &= -\frac{B}{J}w^\delta+\frac{K_t}{J}i_q^\delta-\frac{c}{J}{w^\delta}^2-\frac{2cw^*}{J}{w^\delta}\\
\dot{i}_q^\delta &= - \frac{R}{L_q}i_q^\delta-\frac{n\phi_m}{L_q}w^\delta\tag{4.2}
\end{aligned}
$$
对误差系统在原点 $(w^\delta,i_q^\delta) = (0,0)$ 线性化可得到
$$
\dot{x} = A_{ol}x
$$
其中，$x=[w^\delta,i_q^\delta]^T$，$
A_{ol} = \left[
\begin{array}{c}
-\frac{B+2cw^*}{J} & \frac{Kt}{J}\\
-\frac{n\phi_m}{Lq} & -\frac{R}{Lq}
\end{array}\right]$ -->

**Proposition 1** *考虑动力学方程（1）以及控制律（2），误差系统的平衡点 $\widetilde{x}=0$ 是（全局）指数稳定的*

**Proof:** 闭环系统的方程为

$$
\dot{\widetilde{x}} = v - v_r
$$
根据控制律（2）可得
$$
\dot{\widetilde{x}} = -k\widetilde{x}
$$
现考虑以下候选李雅普诺夫函数
$$
V \triangleq \frac{1}{2} |\widetilde{x}|^2 = \frac{1}{2} \widetilde{x}^T\widetilde{x}
$$
对 $V$ 求导
$$
\begin{align*}
\dot{V} &= \widetilde{x}^T \dot{\widetilde{x}}\\
        &= -k \widetilde{x}^T\widetilde{x}\\
        &= -k |\widetilde{x}|^2
\end{align*}
$$
根据定理4.10（参考文献1）可知，平衡点 $\widetilde{x} = 0$ 是全局指数稳定

**Proposition 2** *考虑动力学方程（1）以及控制律（3），误差系统的平衡点 $\widetilde{v}=0$ 是（全局）指数稳定的*

**Proof:** 闭环系统的方程为

$$
\dot{\widetilde{v}} = -\frac{1}{m}(TRe_3)+ge_3-a_r
$$
根据控制律（3）可得
$$
\dot{\widetilde{v}} = -\frac{k}{m}\widetilde{v}
$$
现考虑以下候选李雅普诺夫函数
$$
V \triangleq \frac{1}{2} |\widetilde{v}|^2 = \frac{1}{2} \widetilde{v}^T\widetilde{v}
$$
对 $V$ 求导
$$
\begin{align*}
\dot{V} &= \widetilde{v}^T \dot{\widetilde{v}}\\
        &= -k \widetilde{v}^T\widetilde{v}\\
        &= -k |\widetilde{v}|^2
\end{align*}
$$
根据定理4.10（参考文献1）可知，平衡点 $\widetilde{v} = 0$ 是全局指数稳定

**Proposition 3** *考虑动力学方程（1）以及控制律（5），误差系统的平衡点 $\widetilde{R}=I$ 是（全局）指数稳定的*

**Proof:** 闭环系统的方程为

$$
\begin{align*}
\dot{\widetilde{R}} 
                    % &= \dot{(R^TR_d)}\\
                    % &= \dot{R}^TR_d + R^T\dot{R}_d\\
                    &= -sk(\Omega)\widetilde{R}+\widetilde{R}sk(\Omega_d)
\end{align*}
$$
根据控制律（3）可得
$$
\dot{\widetilde{v}} = -\frac{k}{m}\widetilde{v}
$$
现考虑以下候选李雅普诺夫函数
$$
V \triangleq \frac{1}{2} \widetilde{v}^2
$$
对 $V$ 求导
$$
\begin{align*}
\dot{V} &= \widetilde{v} \dot{\widetilde{v}}\\
        &= -\frac{k}{m}\widetilde{v}^2 
\end{align*}
$$
根据定理4.10（参考文献1）可知，平衡点 $\widetilde{v} = 0$ 是全局指数稳定

## Simulation Results

<!-- 在本节中，我们通过仿真验证控制器的有效性。
* **Simulation 1** - 当参数不存在不确定性，开环和闭环响应对比
![](微信图片_20250323145101.png)
![](微信图片_20250323145233.png)
* **Simulation 2** - 当参数存在不确定性，闭环响应对比
![](微信图片_20250323150353.png) -->

<!-- ## Conclusion

本文对动力电调的开环和闭环系统进行稳定性分析。然而稳定性只是最基本的问题，在此之上进一步考虑鲁棒性、扰动抑制等问题，以及在工程中遇到的问题如何抽象成控制目标进行分析。 -->

## Reference

[1] H. Khalil, *Nonlinear Systems 3rd edition.* New Jersey: Prentice Hall, 2002.