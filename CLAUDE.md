# Dart Standalone — 飞镖自瞄独立部署

## 项目背景
- 用户：jade，RoboMaster 飞镖视觉自瞄
- 原项目：`~/bof_26_vision/`（同济框架，C++/Eigen/OpenCV，分支 `feat/dart-auto-aim`）
- dart_standalone 是独立精简版，零框架依赖，小电脑直接跑

## 代码管线
```
USB相机 → HSV绿色检测 → 取最大圆 → 像素偏移 → 双串口发云台(CRC16)
```

## 保留的同济框架逻辑
- HSV 绿圆检测：inRange → 形态学开闭 → findContours → 面积/圆度过滤
- CRC16 查表法（多项式 0x1021，初始值 0xFFFF），与下位机完全一致
- 串口协议：12字节 payload（3×int32_t，值×100）+ 2字节 CRC16，115200 8N1
- 双串口：云台（角度命令）+ 底盘（坐标命令）

## 两个下位机
1. **下位机1 云台**（/dev/ttyUSB0）：接收 yaw/pitch/fire 标志
2. **下位机2 底盘**（/dev/ttyACM0）：接收世界坐标（当前简化用归一化偏移代替，后续 PnP 解算）

## 可调参数（全部在 .cpp 顶部）
| 参数 | 默认值 | 作用 |
|------|--------|------|
| SERIAL_PORT_GIMBAL | /dev/ttyUSB0 | 云台串口 |
| SERIAL_PORT_CHASSIS | /dev/ttyACM0 | 底盘串口 |
| H_LOW/H_HIGH | 35/85 | HSV 绿色阈值 |
| YAW_SCALE | 0.5 | yaw 灵敏度 |
| FIRE_TOL | 0.05 | 射击容差 |
| FX/FY/CX/CY | 800/800/320/240 | 相机内参 |

## 用户偏好
- 语言：简体中文
- 风格：直接、简洁，不废话
- **代码不能删原版 dart.cpp**，只能新增或注释
- 数学教学：纯文字，不要 LaTeX，不要 markdown 代码块格式
- 生成的文档/文件放桌面 `/home/jade/Desktop/`
- 飞镖框架逻辑和算法不能丢，同济框架是 baseline

## 当前状态
- dart_standalone 编译通过，可运行
- dart_simple.cpp 在 bof_26_vision 内已编译（框架内简化版）
- 原 dart.cpp 完整保留（完整管线：检测→跟踪→PnP→弹道→串口）
- 三个版本并行：dart.cpp（完整）/ dart_simple.cpp（框架内简化）/ dart_standalone（独立部署）

## 待办
- 相机内参标定（当前 FX/FY/CX/CY 是占位值）
- 从 dart.cpp 移植 PnP 解算到 dart_standalone（世界坐标替换归一化偏移）
- 从 dart.cpp 移植弹道补偿（RK4 弹道求解）
- 小电脑交叉编译配置
