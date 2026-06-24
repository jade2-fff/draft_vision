# dart_easy — 飞镖绿色光源简易自瞄

> HSV 绿色检测 → 像素偏移算角度 → 双串口 CRC16 发送  
> 零框架依赖，只靠 OpenCV

## 管线

```
USB相机 → HSV绿色inRange → 形态学 → 轮廓(面积+圆度) → 取最佳目标
       → 归一化像素偏移 → yaw/pitch 角度 → CRC16 串口 → 下位机
```

## 文件

| 文件 | 职责 |
|------|------|
| `dart_easy.cpp` | 主循环 |
| `dart_detector.h/cpp` | HSV 绿色光源检测 |
| `dart_aimer.h/cpp` | 像素偏移 → 角度指令 + 射击判定 |
| `serial_protocol.h/cpp` | CRC16 协议 + 双串口 |
| `CMakeLists_easy.txt` | CMake 编译 |

## 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

或直接 g++:

```bash
g++ -std=c++17 -O2 \
  dart_easy.cpp serial_protocol.cpp dart_detector.cpp dart_aimer.cpp \
  -o dart_easy $(pkg-config --cflags --libs opencv4)
```

## 串口协议 (对齐下位机 USB_usart)

```
[12字节 payload (3×int32_t, 值×100)] + [2字节 CRC16 (多项式 0x1021)]
```

| 设备 | 路径 | 发送内容 |
|------|------|---------|
| 云台 | `/dev/ttyUSB0` | `(yaw, pitch, fire)` |
| 底盘 | `/dev/ttyACM0` | `(yaw, pitch, 0)` |

波特率 115200 8N1

## 参数调整

```cpp
// HSV (dart_detector.cpp / set_hsv)
detector.set_hsv(35, 85,    // H: 35-85 绿色
                 50, 255,   // S: 50-255
                 50, 255);  // V: 50-255

// 瞄准 (dart_aimer.cpp / set_scales, set_fire_tol)
aimer.set_scales(0.5, 0.3);       // yaw/pitch 缩放
aimer.set_fire_tol(0.05, 0.05);   // 射击容差 (归一化)
```

## 运行

```bash
./dart_easy
# 按 'q' 退出
```
