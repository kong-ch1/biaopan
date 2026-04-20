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

1. 准备工作
首先确保你已经：
安装了 ESP-IDF v5.4.x
拥有 ESP32-S3-EYE 开发板
连接好开发板到电脑（通过 USB）

2. 构建项目
在项目目录下执行以下命令：
进入项目目录：```text
datecd /workspace/ChronosFluid
构建项目：```text
dateidf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye build
或者使用 Windows 批处理脚本：
```text
datebuild_watch.bat

3. 烧录到开发板
```text
dateidf.py -p <你的串口> flash
如```text
dateidf.py -p COM3 flash


```


