# ESP32-S3-EYE Watch Face + Fluid Mode

## 项目简介

这是一个基于 `esp-bsp/examples/display` 改造的 `ESP32-S3-EYE` 项目，包含两个可切换功能：

- `表盘模式`：显示时间与状态信息
- `流体模式`：根据板载 IMU 倾斜方向驱动液体粒子流动

目标是保留 BSP 原有工程结构，同时提供可直接编译、烧录、演示的版本。

## 看这个项目先看哪里

如果你是第一次接手，按下面顺序看最省时间：

1. `main/main.c`：程序入口
2. `main/watch_face.c`：核心逻辑（模式切换、表盘、流体、按键、IMU）
3. `main/watch_face.h`：对外接口
4. `sdkconfig.bsp.esp32_s3_eye`：当前板卡默认配置

## 目录说明（给阅读者）

```text
examples/display/
├─ main/                      # 业务代码（最重要）
│  ├─ main.c                  # app_main 入口
│  ├─ watch_face.c            # 表盘 + 流体 + 按键 + IMU
│  ├─ watch_face.h            # 头文件
│  ├─ watch_zh_patch_font.c   # 中文字体补丁
│  ├─ lvgl_demo_ui.c          # 原始 LVGL 演示代码（辅助参考）
│  └─ images/                 # 图片资源
├─ videos/                    # 演示视频素材
│  ├─ watch_face/             # 表盘演示图片
│  └─ fluid_sim/              # 流体演示视频
├─ CMakeLists.txt             # 工程构建入口（ESP-IDF/CMake）
├─ pytest_display.py          # 自动化测试脚本（pytest-embedded）
├─ sdkconfig.bsp.*            # 各开发板预设配置
├─ sdkconfig.defaults.esp32   # 默认配置模板
└─ README.md                  # 项目说明
```

## 当前功能

- 自定义表盘界面
- 两套表盘主题风格
- `MENU` 键切换 `表盘模式` / `流体模式`
- `PLAY` 键切换表盘风格
- `PLAY` 长按重置流体
- `UP / DOWN` 在流体模式下旋转重力方向
- 自动探测 IMU：`QMA6100P` / `BMI270` / `ICM42670`
- 基于 LVGL Canvas 的轻量流体渲染
- 支持串口设置系统时间

## 硬件与环境

- 芯片：`ESP32-S3`
- 开发板：`ESP32-S3-EYE`
- SDK：`esp-idf v5.4`
- 图形库：`LVGL`
- 串口示例：`COM3`

## 功能分类规则

- `表盘功能`：`APP_MODE_WATCH`
  - 绘制与刷新主要在 `build_watch_face_ui()`、`tick_cb()`
- `流体功能`：`APP_MODE_FLUID`
  - 绘制与仿真主要在 `build_fluid_ui()`、`fluid_sim_step()`、`fluid_timer_cb()`
- `MENU` 键：切换 `表盘模式 / 流体模式`
- `PLAY` 键：切换主题；流体模式长按重置粒子

## 流体实现说明

当前版本的流体模拟采用的是**轻量粒子流体方案**，不是严格的 `FIP/FLIP` 网格求解器。

实现思路：

1. 用一组二维粒子表示液体
2. 每帧根据 IMU 倾斜方向计算屏幕内重力向量
3. 对粒子施加重力、阻尼并更新位置
4. 对过近粒子做推开与简单碰撞修正
5. 对边界做反弹与速度衰减
6. 最后将粒子绘制到 LVGL Canvas 中

这样做的优点是：

- 结构简单
- 易于继续迭代
- 更适合 `ESP32-S3-EYE` 上实时运行

## 按键说明

- `MENU`：切换表盘模式 / 流体模式
- `PLAY`：切换表盘主题
- `PLAY 长按`：重置流体粒子
- `UP`：流体模式下重力旋转 `+10°`
- `DOWN`：流体模式下重力旋转 `-10°`

## 构建与烧录

在 `examples/display` 目录下执行：

```bash
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye build
idf.py -p COM3 -b 115200 flash
```

如果需要串口监视：

```bash
idf.py -p COM3 monitor
```

## 设置时间

项目支持通过串口输入以下命令设置系统时间：

```text
date MMDDHHMMYYYY
```

例如：

```text
date 040217302026
```

表示设置为 `2026-04-02 17:30`。

## 主要文件

- `main/main.c`：应用入口
- `main/watch_face.c`：表盘、按键、IMU、流体逻辑
- `main/watch_face.h`：应用入口声明
- `main/watch_zh_patch_font.c`：中文字体补丁
- `CMakeLists.txt`：工程构建入口，定义项目名与主组件
- `pytest_display.py`：示例自动化测试，校验程序启动日志
- `sdkconfig.bsp.esp32_s3_eye`：板级默认配置

## 视频放置规范

- 表盘视频放到：`videos/watch_face/`
- 流体视频放到：`videos/fluid_sim/`
- 目录中的 `.gitkeep` 仅用于让 Git 记录空目录；放入真实视频后可保留也可删除

## 当前状态

当前仓库中的这个版本是一个**稳定基线版本**：

- 已可编译
- 已可烧录到 `ESP32-S3-EYE`
- 表盘与流体模式均可运行
- 已保存到本地 Git 提交

如果后续继续演进，可以进一步升级为：

- 更接近 `metaball / density field` 的液面渲染
- 更真实的 `PBF / FIP / FLIP` 风格流体约束求解
- 更强的水面连续感与甩动感

## 备注

当前远程仓库默认指向上游 `espressif/esp-bsp`。如果你要把本项目上传到你自己的 GitHub，建议：

1. 新建你自己的 GitHub 仓库
2. 将本地仓库远程改为你的仓库，或新增一个新的 remote
3. 再执行 `git push`
