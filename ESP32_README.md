# ArduPilot ESP32 精简版

这是 ArduPilot 的 ESP32 专用精简版本，移除了 ChibiOS、Linux 等非 ESP32 平台的代码。

## 🎯 项目特点

- ✅ **仅支持 ESP32/ESP32-S3**：移除了 ChibiOS、Linux、QURT 等其他平台代码
- ✅ **编译成功**：已验证可以成功编译 Rover 固件
- ✅ **精简高效**：移除了测试框架和不必要的工具
- ✅ **保留核心功能**：所有 ESP32 必需的库和功能完整保留

## 📦 已删除的内容

- ChibiOS 操作系统 (~381MB)
- Linux 平台支持
- QURT 平台支持
- 测试框架（gtest, gbenchmark, gsoap, CrashDebug）
- 开发工具（20+ 个针对其他平台的工具）

## 📂 项目结构

```
ardupilot/
├── libraries/          # 核心库（包含 AP_HAL_ESP32）
├── Rover/             # Rover 固件源码
├── Tools/             # 构建工具（仅保留 ESP32 必需）
├── modules/           # ⚠️ 未包含在仓库中（需单独获取）
└── build/             # 构建输出（.gitignore 忽略）
```

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/oceangis/ardupilot_esp32.git
cd ardupilot_esp32
```

### 2. 获取依赖模块

由于 `modules/` 目录太大（~3GB），未包含在仓库中。需要手动获取：

#### 方法 A：使用 git submodule（推荐）

```bash
# 初始化并更新所有子模块
git submodule update --init --recursive
```

#### 方法 B：从 ArduPilot 官方仓库复制

```bash
# 克隆官方 ArduPilot
git clone --recursive https://github.com/ArduPilot/ardupilot.git ardupilot-master

# 复制 modules 目录
cp -r ardupilot-master/modules ./

# 或只复制必需的模块
mkdir -p modules
cp -r ardupilot-master/modules/esp_idf modules/
cp -r ardupilot-master/modules/mavlink modules/
cp -r ardupilot-master/modules/waf modules/
cp -r ardupilot-master/modules/DroneCAN modules/
cp -r ardupilot-master/modules/lwip modules/
cp -r ardupilot-master/modules/littlefs modules/
cp -r ardupilot-master/modules/Micro-XRCE-DDS-Client modules/
cp -r ardupilot-master/modules/Micro-CDR modules/
```

### 3. 配置 ESP-IDF 环境

```bash
# 导出 ESP-IDF 环境变量
cd modules/esp_idf
./install.sh esp32s3
source export.sh
cd ../..
```

或者使用快捷方式：
```bash
source modules/esp_idf/export.sh
```

### 4. 配置编译

```bash
# 配置 ESP32-S3 板子
./waf configure --board=esp32s3devkit

# 可用的板子：
# - esp32s3devkit (默认，有完整传感器配置)
# - esp32s3empty (仿真模式)
# - esp32buzz
# - esp32diy
# 等等...查看 libraries/AP_HAL_ESP32/hwdef/
```

### 5. 编译 Rover 固件

```bash
# 编译 Rover
./waf rover

# 固件输出位置：
# build/esp32s3devkit/esp-idf_build/ardupilot.bin
```

### 6. 烧录固件

```bash
# 自动烧录
./waf rover --upload

# 或手动烧录
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash 0x0 build/esp32s3devkit/esp-idf_build/ardupilot.bin
```

## 🔧 自定义引脚配置

修改硬件引脚配置：

```bash
# 编辑板子的硬件定义文件
nano libraries/AP_HAL_ESP32/hwdef/esp32s3devkit/hwdef.dat
```

可配置项：
- PWM 输出引脚（电机/舵机）
- UART 串口（GPS、遥测）
- I2C、SPI 总线
- RC 接收机输入
- ADC 模拟输入
- WiFi 设置

修改后需重新编译：
```bash
./waf configure --board=esp32s3devkit
./waf rover
```

## 📊 固件信息

- **固件大小**: ~1.9 MB
- **可用分区**: 3 MB
- **剩余空间**: 1.2 MB (38%)
- **编译时间**: ~10 分钟（首次编译）

## 🛠️ 故障排除

### 问题1：找不到 ESP-IDF 工具链

```bash
# 确保已导出 ESP-IDF 环境
source modules/esp_idf/export.sh
```

### 问题2：modules 目录缺失

```bash
# 获取所有子模块
git submodule update --init --recursive
```

### 问题3：编译错误

```bash
# 清理并重新配置
./waf clean
./waf configure --board=esp32s3devkit
./waf rover
```

## 📝 修改记录

### v1.0 - 初始版本
- ✅ 成功编译 ESP32-S3 Rover 固件
- ✅ 移除 ChibiOS、Linux、QURT 平台代码
- ✅ 移除测试框架和不必要工具
- ✅ 修复 LogStructure.h 的 ChibiOS 宏定义
- ✅ 保留所有 ESP32 必需库

## 🔗 相关链接

- [ArduPilot 官方](https://ardupilot.org/)
- [ArduPilot ESP32 文档](libraries/AP_HAL_ESP32/README.md)
- [ESP-IDF 文档](https://docs.espressif.com/projects/esp-idf/)
- [ESP32-S3 引脚图](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/index.html)

## 📄 许可证

ArduPilot 采用 GPLv3 许可证。详见 [COPYING.txt](COPYING.txt)

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

**维护者**: oceangis
**最后更新**: 2025-10-22
