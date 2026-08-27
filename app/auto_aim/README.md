# 装甲板自瞄模块

`app::auto_aim` 是 Robocore 中的装甲板自瞄应用层，迁移自
`TongjiSuperPower/sp_vision_25` 的装甲板链路。

## 模块范围

- 灯条二值化、几何筛选与同色配对
- 可选 OpenCV DNN 数字分类
- 装甲板四点 `solvePnP` 位姿解算
- 云台坐标系到世界坐标系转换
- 11 维 EKF 旋转目标模型、装甲板编号匹配、跳变判断
- detecting/tracking/temp_lost/lost 目标状态机
- 可选 yaw 重投影优化
- 考虑相机延迟和弹道飞行时间的迭代瞄准

本模块明确不包含能量机关识别、预测和控制。

配置位于 `config/sentry.toml` 的 `[auto_aim]` 段。

## 运行链路

```text
相机图像
  -> Detector
  -> Classifier
  -> Solver
  -> Tracker / 11 维 EKF
  -> Aimer / Trajectory
  -> AimCommand
```

`AimCommand.yaw` 和 `AimCommand.pitch` 的单位是弧度，`fire` 只是当前算法给出的开火建议。
仓库目前没有云台串口协议，因此 `task/sentry.cpp` 暂时只生成和记录命令，不直接驱动云台。

## Linux 构建

```bash
cmake -S . -B build
cmake --build build -j
./build/test_auto_aim ./config/sentry.toml
```

运行测试时必须从仓库根目录执行，或者传入正确的配置文件绝对路径。

## 标定参数

`camera_matrix` 和 `distortion` 是相机标定参数；`camera_to_gimbal` 和
`camera_to_gimbal_translation` 是相机到云台的外参。当前配置中的旋转是常用坐标系转换示例，
平移为零；正式上车前必须替换为实测标定结果。
