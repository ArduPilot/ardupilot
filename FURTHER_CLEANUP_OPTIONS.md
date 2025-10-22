# ArduPilot ESP32 Rover - 进一步精简选项

> **当前状态**: 已从 939 MB 精简到 624 MB (节省 315 MB)
>
> **本报告**: 列出可进一步精简的内容（约 73 MB）

---

## 📊 可进一步精简的内容汇总

| 类别 | 大小 | 可节省空间 |
|------|------|-----------|
| **大型可选库** | ~21 MB | 高优先级 |
| **Tools 工具** | ~52 MB | 中优先级 |
| **遥测协议库** | ~0.6 MB | 低优先级 |
| **总计** | **~73 MB** | **可精简到 551 MB** |

---

## 🎯 高优先级精简项（~21 MB）

### 1. AP_OSD (16 MB) ⭐ 最大收益
```
位置: libraries/AP_OSD/
用途: 视频叠加显示 (OSD)
功能: 在 FPV 图传上显示飞行数据
```
**建议**:
- ❌ **删除** - 如果无人船不需要 OSD 屏显
- ✅ **保留** - 如果需要视频叠加飞行数据

**删除命令**:
```bash
rm -rf libraries/AP_OSD/
```

---

### 2. AP_Scripting (4.2 MB)
```
位置: libraries/AP_Scripting/
用途: Lua 脚本引擎
功能: 运行自定义 Lua 脚本扩展功能
```
**建议**:
- ❌ **删除** - 如果不需要 Lua 脚本功能
- ✅ **保留** - 如果需要编写自定义脚本

**删除命令**:
```bash
rm -rf libraries/AP_Scripting/
```

---

### 3. AP_HAL_SITL (338 KB)
```
位置: libraries/AP_HAL_SITL/
用途: 软件在环仿真 HAL
状态: SITL 主体已删除，这是残留
```
**建议**:
- ❌ **建议删除** - ESP32 不需要，已删除 SITL

**删除命令**:
```bash
rm -rf libraries/AP_HAL_SITL/
```

---

### 4. AP_DDS (310 KB)
```
位置: libraries/AP_DDS/
用途: DDS 通信协议 (ROS 2)
功能: 与 ROS 2 系统集成
```
**建议**:
- ❌ **删除** - 如果不使用 ROS 2
- ✅ **保留** - 如果需要与 ROS 2 集成

**删除命令**:
```bash
rm -rf libraries/AP_DDS/
```

---

## 🛠️ 中优先级精简项 - Tools (~52 MB)

### 1. Tools/bootloaders (50 MB) ⭐ 最大收益
```
用途: 其他平台的引导加载程序
包含: STM32/Linux/其他平台 bootloader
```
**建议**:
- ❌ **建议删除** - ESP32 有自己的引导程序

**删除命令**:
```bash
rm -rf Tools/bootloaders/
```

---

### 2. Tools/simulink (918 KB)
```
用途: MATLAB Simulink 集成
```
**建议**:
- ❌ **删除** - 如果不使用 MATLAB
- ✅ **保留** - 如果需要 Simulink 建模

**删除命令**:
```bash
rm -rf Tools/simulink/
```

---

### 3. Tools/IO_Firmware (848 KB)
```
用途: Pixhawk IO 协处理器固件
```
**建议**:
- ❌ **建议删除** - ESP32 不使用 IO 协处理器

**删除命令**:
```bash
rm -rf Tools/IO_Firmware/
```

---

### 4. Tools/AP_Periph (414 KB)
```
用途: CAN 外设节点固件
```
**建议**:
- ❌ **删除** - 如果不需要 CAN 外设节点
- ✅ **保留** - 如果需要构建 AP_Periph 固件

**删除命令**:
```bash
rm -rf Tools/AP_Periph/
```

---

### 5. Tools/Linux_HAL_Essentials (322 KB)
```
用途: Linux HAL 相关工具
```
**建议**:
- ❌ **建议删除** - 已删除 Linux HAL

**删除命令**:
```bash
rm -rf Tools/Linux_HAL_Essentials/
```

---

### 6. Tools/ros2 (273 KB)
```
用途: ROS 2 集成工具
```
**建议**:
- ❌ **删除** - 如果不使用 ROS 2
- ✅ **保留** - 如果需要 ROS 2 集成

**删除命令**:
```bash
rm -rf Tools/ros2/
```

---

## 📡 低优先级精简项 - 遥测协议库 (~0.6 MB)

### 特殊遥测协议
```
libraries/AP_Radio/          293 KB  - Cypress radio
libraries/AP_Frsky_Telem/    162 KB  - FrSky 遥测
libraries/AP_Hott_Telem/      20 KB  - Graupner HoTT
libraries/AP_LTM_Telem/       16 KB  - LTM 遥测
libraries/AP_Devo_Telem/      12 KB  - Walkera Devo
libraries/AP_IBus_Telem/      36 KB  - FlySky iBus
```

**建议**:
- 如果只使用标准的 MAVLink，可以删除这些专有遥测协议
- 保留常用的即可

---

## 🔧 其他可选功能库

### 可选但可能有用
```
libraries/AP_Beacon/         139 KB  - 信标定位系统
libraries/AP_ExternalAHRS/   272 KB  - 外部 AHRS
libraries/AC_Avoidance/      268 KB  - 避障（可能船用有用）
libraries/AP_Button/          20 KB  - 按钮输入
libraries/AP_KDECAN/          17 KB  - KDE CAN 电调
libraries/AP_Stats/           12 KB  - 统计信息
libraries/AP_TempCalibration/ 13 KB  - 温度校准
```

**建议**:
- 评估后再决定是否删除
- AC_Avoidance 对船可能有用（避障）

---

## 📋 一键精简脚本（激进版）

**⚠️ 警告**: 执行前请确认不需要这些功能

```bash
#!/bin/bash
# 激进精简脚本 - 额外节省 ~73 MB

cd f:/opensource/usv_esp32/esp32s3rover/ardupilot

echo "===== 删除大型可选库 ====="
rm -rf libraries/AP_OSD/              # 16 MB
rm -rf libraries/AP_Scripting/        # 4.2 MB
rm -rf libraries/AP_HAL_SITL/         # 338 KB
rm -rf libraries/AP_DDS/              # 310 KB

echo "===== 删除 Tools 中不需要的 ====="
rm -rf Tools/bootloaders/             # 50 MB
rm -rf Tools/simulink/                # 918 KB
rm -rf Tools/IO_Firmware/             # 848 KB
rm -rf Tools/AP_Periph/               # 414 KB
rm -rf Tools/Linux_HAL_Essentials/    # 322 KB
rm -rf Tools/ros2/                    # 273 KB

echo "===== 删除专有遥测协议 ====="
rm -rf libraries/AP_Radio/            # 293 KB
rm -rf libraries/AP_Frsky_Telem/      # 162 KB
rm -rf libraries/AP_Hott_Telem/       # 20 KB
rm -rf libraries/AP_LTM_Telem/        # 16 KB
rm -rf libraries/AP_Devo_Telem/       # 12 KB

echo "===== 完成 ====="
du -sh .
```

---

## 💡 推荐精简方案

### 保守方案（推荐）⭐
```bash
# 只删除明确不需要的（~51 MB）
rm -rf libraries/AP_HAL_SITL/         # 仿真残留
rm -rf Tools/bootloaders/             # 其他平台引导
rm -rf Tools/IO_Firmware/             # Pixhawk IO
rm -rf Tools/Linux_HAL_Essentials/    # Linux 工具

精简后: ~573 MB
```

### 中等方案
```bash
# 保守方案 + 可选大库（~72 MB）
# 添加删除:
rm -rf libraries/AP_OSD/              # 如果不需要屏显
rm -rf libraries/AP_Scripting/        # 如果不需要脚本
rm -rf Tools/simulink/
rm -rf Tools/AP_Periph/
rm -rf Tools/ros2/

精简后: ~552 MB
```

### 激进方案
```bash
# 使用上面的一键脚本
精简后: ~551 MB
节省: ~388 MB (从原始 939 MB)
```

---

## ⚠️ 注意事项

1. **编译依赖**: 删除库后可能影响编译，需要在板型配置中禁用相关功能
2. **功能完整性**: 删除前确认不需要该功能
3. **可恢复性**: 所有删除都可以从 Git 恢复
4. **测试编译**: 删除后务必测试编译是否成功

---

## 🔍 如何决定是否删除

### 问自己以下问题：

**AP_OSD (16 MB)**:
- ❓ 需要在 FPV 视频上显示飞行数据吗？
- ❓ 有视频图传系统吗？

**AP_Scripting (4.2 MB)**:
- ❓ 需要编写自定义 Lua 脚本吗？
- ❓ 需要在运行时扩展功能吗？

**ROS 2 相关 (AP_DDS + Tools/ros2)**:
- ❓ 需要与 ROS 2 系统集成吗？
- ❓ 使用 ROS 2 进行导航或控制吗？

**如果答案都是"否"，可以安全删除！**

---

**检查时间**: 2025-10-22
**当前大小**: 624 MB
**可精简到**: 551 MB (保守) / 551 MB (激进)
