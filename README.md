# 飞镖自瞄 · dart_auto_aim

RoboMaster 飞镖自动瞄准上位机视觉模块。基于 bofvision 管线架构，适配飞镖靶（圆形靶面，仅 yaw 轴）。

## 管线架构

```
相机采图 ─→ 检测器 (HSV+圆) ─→ 跟踪器 (EKF) ─→ 归一化偏移 ─→ 串口 ─→ 下位机
              │                    │
              └── PnP 3D位姿 ──────┘
```

## 模块职责

| 模块 | 文件 | 功能 | 输出 |
|------|------|------|------|
| **detector** | `detector/dart_detector.{hpp,cpp}` | HSV 绿色阈值 → 轮廓提取 → 圆度/填充率/面积过滤 | `DartTarget` 列表（像素坐标、半径、bbox） |
| **solver** | `solver/dart_solver.{hpp,cpp}` | PnP IPPE 解算：像素 → 相机系 → 世界系坐标 | 靶心 3D 位置 (m)、距离 (m)、靶面法向 |
| **tracker** | `tracker/dart_tracker.{hpp,cpp}` | 图像空间 EKF（8 状态：[cx, vx, cy, vy, w, vw, h, vh]） + 4 状态机 | 稳定跟踪目标，归一化中心偏移 |
| **aimer** | `aimer/dart_aimer.{hpp,cpp}` | 抛物线弹道 + 空气阻力补偿（**当前未启用**，pitch 由下位机控制） | yaw/pitch 调整量 |
| **common** | `common/dart_types.hpp` | 数据类型：`DartTarget`（检测+位姿+角点）、`DartDetectionResult`（帧结果） | 统一数据结构 |

## 上位机 ↔ 下位机

```
上位机（视觉 + 位姿解算）          下位机（MCU + 电机控制）
┌─────────────────────┐          ┌──────────────────────────┐
│ 相机采图             │          │                          │
│   ↓                  │          │                          │
│ HSV 绿靶检测         │          │                          │
│   ↓                  │  串口    │                          │
│ PnP 解算距离 + yaw   │ ──────→  │ 查表/算弹道 pitch        │
│   ↓                  │          │   ↓                      │
│ EKF 跟踪             │          │ 驱动 yaw 电机 + 发射机构  │
│   ↓                  │          │                          │
│ 归一化偏移 / 距离+yaw │          │                          │
└─────────────────────┘          └──────────────────────────┘
```

**当前协议**：上位机发归一化 yaw 偏移（`float`, [-1, 1]），与区域赛 draft 兼容。  
**待定**：完成后可能改为发（距离 + yaw 角），由下位机根据距离自行调整 pitch 和发射时机。  
**不做弹道补偿**：飞镖 pitch 为固定机械角，弹道由下位机标定曲线控制。

## 为什么不做 pitch

飞镖发射机构只有 yaw 轴可动，pitch 是固定机械角。  
下位机已有弹道标定曲线（距离 → pitch 查表），上位机只提供视觉指向。

## 跟踪器状态机

```
    ┌──────────────────────────────────┐
    │                                  │
    ▼                                  │
  LOST ──检测到──→ DETECTING ──连续N帧──→ TRACKING
    ▲                  │                    │
    │                  │ 丢失                │ 暂时丢失
    │                  ▼                    ▼
    └────超时──── TEMP_LOST ←──────────── TEMP_LOST
                      │                      │
                      └──超时──→ LOST ←──────┘
```

- **LOST**：无目标
- **DETECTING**：检测到但未达稳定阈值（需连续 5 帧）
- **TRACKING**：稳定跟踪，EKF 预测+更新
- **TEMP_LOST**：短暂丢失（容忍 15 帧），EKF 纯预测不更新

## 坐标变换链

```
像素坐标 (u,v)
    │ PnP (IPPE, 相机内参 + 靶面3D模型 14cm×14cm)
    ▼
相机系 3D 坐标 (Xc, Yc, Zc)
    │ R_camera2gimbal + t_camera2gimbal (手眼标定)
    ▼
云台系 3D 坐标 (Xg, Yg, Zg)
    │ R_gimbal2world (IMU 四元数, 每帧更新)
    ▼
世界系 3D 坐标 (Xw, Yw, Zw)  → 距离 = |position_world|
                                    yaw = atan2(Yw, Xw)
```

## 弹道标定方案（后续）

**打靶心法**：多个距离各打 5×5 发，调 pitch 直到命中靶心，记录 (距离, 命中pitch)。  
**拟合**：Python 脚本对 (距离, pitch) 做多项式拟合，得到下位机补偿曲线。

上位机不跑弹道公式。pitch 全权由下位机控制。

## 待定事项

- [ ] **串口协议**：与下位机负责人确认数据格式（距离 + yaw 角 vs 归一化偏移）
- [ ] **波特率/帧头/校验**：需与 MCU 端对齐
- [ ] **YOLO 检测**：预留接口，未来可替换传统 CV 检测
- [ ] **弹道标定**：实物飞镖打好数据后拟合
- [ ] **小电脑部署**：SSH 配置后推送实际测试

## 构建

```bash
# 在 bof_26_vision 项目内
cd bof_26_vision
mkdir -p build && cd build
cmake .. -DBUILD_DART=ON
make dart_auto_aim -j$(nproc)

# 运行
./dart_auto_aim configs/dart.yaml
```

## 依赖

- OpenCV ≥ 4.5（传统 CV 检测 + PnP）
- Eigen 3（EKF + 坐标变换）
- yaml-cpp（配置加载）
- fmt（日志/调试输出）
- bofvision 框架（`io::Camera`, `io::Gimbal`, `tools::Exiter`, `tools::ExtendedKalmanFilter`）
