# ChronosFluid (ESP32-S3-EYE)

基于 `esp-bsp/examples/display` 改造的 ESP32-S3-EYE 项目，包含：

- 表盘模式（指针 + 数字时间）
- 流体/倾斜交互模式（依赖 IMU）
- 串口时间设置（`date MMDDHHMMYYYY`）

## 目录结构

```text
ChronosFluid_upload_20260416/
├─ CMakeLists.txt
├─ sdkconfig.bsp.esp32_s3_eye
├─ partitions_display_large_app.csv
├─ main/
│  ├─ main.c
│  ├─ watch_face.c
│  ├─ watch_face.h
│  ├─ watch_zh_patch_font.c
│  ├─ app_video.c
│  └─ app_video.h
├─ send_date_cmd.py
└─ build_watch.bat
```

## 环境

- 芯片：ESP32-S3
- 开发板：ESP32-S3-EYE
- ESP-IDF：v5.4.x
- 图形库：LVGL

## 构建与烧录

在本目录执行：

```bash
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye build
idf.py -p COM3 flash
idf.py -p COM3 monitor
```

也可使用脚本（按你本机路径）：

```bash
build_watch.bat
```

## 设置系统时间

方式 1：串口直接发送：

```text
date MMDDHHMMYYYY
```

示例：

```text
date 040217302026
```

表示：`2026-04-02 17:30`

方式 2：用脚本发送（默认发送电脑当前时间）：

```bash
python send_date_cmd.py --port COM3
```

##  教学步骤

ChronosFluid 烧录教程
1. 准备工作
在开始之前，请确保你已经准备好：

✅ 安装了 ESP-IDF v5.4.x
✅ 拥有 ESP32-S3-EYE 开发板
✅ 用 USB 线连接好开发板到电脑
2. 构建项目
进入项目目录

Bash

cd /workspace/ChronosFluid
编译构建

Bash

idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye build
Windows 用户：也可以直接运行批处理脚本：


Bash

build_watch.bat
3. 烧录到开发板
查找串口
首先确定你的开发板串口号：

Windows：设备管理器 → 端口 (COM 和 LPT)
Linux：ls /dev/ttyUSB* 或 ls /dev/ttyACM*
macOS：ls /dev/tty.usb*
执行烧录

Bash

idf.py -p <你的串口> flash
示例：


Bash

idf.py -p COM3 flash
4. 监控串口输出（可选）

Bash

idf.py -p <你的串口> monitor
```


