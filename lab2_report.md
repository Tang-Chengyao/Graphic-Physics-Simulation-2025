# 图形学物理仿真 Lab2

Lab2 的作业中主要有四个文件：
`CaseFluid.cpp`: 在 Lab0 例子的基础上添加了障碍物的渲染和键盘交互控制障碍物移动的代码，以及部分 UI 控制参数。
`CaseFluid.h`: 声明了 CaseFluid.cpp 中的函数和变量，和 Lab0 例子提供的没有大的改动。
`FluidSimulator.cpp`: 提供了`FluidSimulator.h`中主要模拟逻辑`Simulator::SimulateTimeStep`中几个函数的实现。
`FluidSimulator.h`: 增加了部分 Simulator 的成员变量方便实现。

核心逻辑：
`FluidSimulator.cpp`中的函数实现：
`Simulator::integrateParticles`: 显示欧拉法。
`Simulator::buildHashTable`: 重建空间哈希表，m_hashtableindex: 长度为 m_iNumCells，存储每个网格内的粒子索引在 m_hashtable 中 index 的起始值。m_hashtable: 长度为 m_iNumSpheres，存储指定粒子的索引。
`Simulator::pushParticlesApart`: 每次 pushApart 前先调用 buildHashTable 更新哈希表，然后遍历粒子，用哈希表找出临近的 27 个网格，进行 pushApart。
`Simulator::handleParticleCollisions`: 遍历粒子，处理边界碰撞，同时检测和障碍物球的碰撞。
`Simulator::updateParticleDensity`: 更新网格存储的密度，用于在压强求解中使用 compensateDrift 调整流体密度
`Simulator::transferVelocities`: 粒子 To 网格，网格 To 粒子的速度转换。其中网格 To 粒子才用了 Flip 和 Pic 混合。要注意对于速度的 uvw 分量，网格要加上对应方向的偏移。
`Simulator::solveIncompressibility`: 在网格上求解压强，用 Gauss-Seidel 迭代，加入了 overRelaxation 和 CompensateDrift。

主要问题：
底部高频碰撞无法稳定，可能是由于 pushParticleApart 的方法，每次推开一对小球，但是可能造成推开后的小球和别的小球造成新的重叠。
最近比较忙，粒子的着色和一些代码细节没调节，只写了一个初步的 demo。
