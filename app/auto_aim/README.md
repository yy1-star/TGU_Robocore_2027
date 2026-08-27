# 装甲板自瞄模块

`app::auto_aim` 是 Robocore 中的装甲板自瞄应用层，迁移自
`TongjiSuperPower/sp_vision_25` 的装甲板链路。

## 模块范围

- 灯条二值化、几何筛选与同色配对
- 可选 OpenCV DNN 数字分类
- 装甲板四点 `solvePnP` 位姿解算
- 云台坐标系到世界坐标系转换
- 旋转目标中心、速度和角速度估计
- 考虑相机延迟和弹道飞行时间的迭代瞄准

本模块明确不包含能量机关识别、预测和控制。

配置位于 `config/sentry.toml` 的 `[auto_aim]` 段。
