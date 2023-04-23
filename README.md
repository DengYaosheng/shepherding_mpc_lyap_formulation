# shepherding_mpc_lyap_formulation
A test and play simu for shepherding formulation using mpc and distance controlled by lyap, and add control barrier func for collision avoidance
此代码实现了一个具有MPC控制的集群控制系统。MPC控制器的目的是优化最终时刻的性能指标，通过调整输入来最小化系统的成本函数。
成本函数是系统的位置和速度的平方和，即：
$J=(x(N+1)TPx(N2+1)+u(i)TRu(i))$
其中 $\mathbf{x}(N_p+1)$ 是最终时刻的状态，$\mathbf{u}(i)$ 是第 $i$ 步的输入，$\mathbf{P}$ 和 $\mathbf{R}$ 分别是状态和输入的权重矩阵。使用二次规划来计算最优输入。
请注意，由于使用了MPC控制器，因此每个sheep agent的速度和位置都将在每个时间步长中计算。这比传统的集群控制系统更为复杂，但可以实现更好的性能。
* Notice this shepherding code, the shepherd relpaced by an external field. No shepherd agent exists.

下面是仿真结果绘图，可以看到羊在避开障碍物的同时形成了编队，而且羊与障碍物之间的距离保持在一定范围内。
![image](https://user-images.githubusercontent.com/69342162/137537731-4f62860d-cade-439b-aed1-22f7ec9a9a67.png)
