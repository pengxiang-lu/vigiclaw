# VigiClaw

## 简介

![工作流程图.drawio](assets/工作流程图.drawio.png)

​	VigiClaw 是一个基于 OpenClaw 的机械臂仿真与控制项目，使用的仿真环境是ManiSkill。用于把Openclaw接入到ManiSkill仿真环境中，然后用自然语言指令控制机械臂完成任务。VigiClaw名称是Vigilant Openclaw的缩写，代表警觉的Openclaw。这个项目也可以使用其他Agent框架如codex。论文名称：**VigiClaw: Action-Triggered State Verification for Robust Long-Horizon Robotic Manipulation**。

## 环境要求

- Linux 或其他支持 SAPIEN/ManiSkill 的系统
- Python `>=3.10,<3.13`
- 可用的 GPU/图形环境，因为开启仿真。
- ManiSkill `>=3.0.1`

## 安装

### 使用 uv（推荐）

```bash
git clone https://github.com/<your-name>/VigiClaw.git
cd VigiClaw

uv venv
uv sync
source .venv/bin/activate
```

### 使用 pip

```bash
git clone https://github.com/<your-name>/VigiClaw.git
cd VigiClaw

python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

视觉 Skill 脚本还需要额外依赖，按实际使用的脚本安装：

```bash
pip install requests numpy Pillow opencv-python ultralytics
```

### 安装YOLO 环境

本项目的物体检测和视觉定位 Skill 使用 Ultralytics YOLO，推荐使用名为 `yolo26` 的 Conda 环境：

```bash
conda create -n yolo26 python=3.10 -y
conda activate yolo26
pip install requests numpy Pillow opencv-python ultralytics
```

如果 `yolo26` 环境已经存在，只需要执行：

```bash
conda activate yolo26
pip install requests numpy Pillow opencv-python ultralytics
```

运行视觉脚本前，请确保已经激活该环境：

```bash
conda activate yolo26
cd skill-direct/scripts
conda run -n yolo26 python object_detect.py
```

### 安装OpenClaw

这里默认安装完成OpenClaw。

## OpenClaw 任务列表

项目通过 `openclaw_task` 注册了以下 OpenClaw 任务：

| 任务名称                     | 任务简介                                                     | 是否可用 |
| ---------------------------- | ------------------------------------------------------------ | -------- |
| `OpenclawPickCube`           | 抓取红色方块，并将其提起到指定高度。                         | √        |
| `OpenclawPickCup`            | 从两个颜色不同但形状相同的杯子中，抓取指定目标杯子并提起。   | √        |
| `OpenclawPickBanana`         | 抓取香蕉，并将其提起到指定高度。                             | √        |
| `OpenclawMoveApple`          | 抓取苹果，按照随机指定的方向移动一定距离后放下。             | √        |
| `OpenclawPutBananaOnPlate`   | 抓取香蕉并将其放到盘子上。                                   | √        |
| `OpenclawStackCubes`         | 按顺序将红色方块堆叠到绿色方块上、蓝色方块上，最后将黄色方块堆叠到蓝色方块上。 | √        |
| `OpenclawDailyScene`         | 在包含棒球、香蕉、杯子、苹果和马克笔的日常场景中，根据指令抓取工具（默认指马克笔）。 | √        |
| `OpenclawPutTogether`        | 将红、绿、蓝三个方块分别放入对应颜色的碗中。                 | √        |
| `OpenclawStackCubesDisturb`  | 完成红、绿、蓝方块的堆叠，并将蓝色方块堆叠到红色方块上；任务过程中可能发生方块干扰。 | √        |
| `OpenclawPutTogetherDisturb` | 将方块放入对应颜色的碗中；任务过程中可能出现方块回移或碗移动等干扰。 | √        |
| `OpenclawOpenCabinet`        | 打开柜子，并抓取柜子内部的物体。                             | ×        |
| `OpenclawPushCube`           | 推动目标方块，按照随机指定的方向移动一定距离。               | √        |
| `OpenclawObstacleAvoid`      | 绕过蓝色障碍物，抓取红色方块并将其提起。                     | ×        |

## 使用流程

### 服务端：

查看 OpenClaw 自定义任务：

```bash
PYTHONPATH=/home/lpx/python-project/VigiClaw uv run python -c "import mani_skill.envs; import openclaw_task; from mani_skill.utils.registration import REGISTERED_ENVS; print('\\n'.join(sorted(k for k in REGISTERED_ENVS if k.startswith('Openclaw'))))"
```

仿真服务端启动时会导入 `openclaw_task`，自动注册工程内的自定义任务。

默认服务地址：

```text
http://127.0.0.1:8000
```

在第一个终端启动仿真服务端，仿真服务端支持通过 `--task` 选择要运行的 OpenClaw 任务，默认任务为 `OpenclawMoveApple`：

```bash
cd openclaw_control
python server.py --task OpenclawPickCube	# 这里选择传入 OpenClaw 自定义任务中的一个任务即可
```

也可以通过命令行设置 episode 轮数：

```bash
python server.py --task OpenclawPickCube --episode_nums 5
```

任务完成 5 个 episode 后，仿真服务端会自动退出。

### 客户端：

然后启动OpenClaw或其他Agent框架,先输入：

```bash
请学习一下 绝对地址。	
```

这里的绝对地址就是工程目录下的skill文件夹，可以选择skill-dmp或者skill-direct，这两个技能的区别主要在action，skill-direct直接给关键点控制，skill-dmp会使用先前学习到的示教参数生成连续轨迹。

然后输入：

```
我开启了服务器，请完成任务。
```

就可以看到机械臂完成任务的过程。

![异常检测](assets/异常检测.png)

## DMP 轨迹采集、学习与执行

采集、权重学习和本地回放脚本位于 `openclaw_control/`。执行任务时分为两个 skill：

| skill 目录 | 动作脚本 | 权重资源 |
| --- | --- | --- |
| `skill-direct/` | `scripts/direct_action.py`，直接发送单步 TCP 目标 | `assets/yolo_seg.pt`，用于视觉感知 |
| `skill-dmp/` | `scripts/dmp_action.py`，由 DMP 权重生成并执行轨迹 | `weights/*_weights.npy`，用于 DMP；`assets/yolo_seg.pt` 用于视觉感知 |

两个目录各有独立的 `SKILL.md`，并包含感知、健康检查、任务查询和重置脚本。
原 `skill/` 已拆分；根据是否使用 DMP 选择对应 skill。
旧 `openclaw_control/dmp/` 已移除，新脚本共用 `dmp_core.py` 中的 DMP 逻辑。
YOLO 分割权重放在各 skill 的 `assets/yolo_seg.pt`，视觉脚本按自身路径定位资源。
下面的命令在项目根目录运行。

### 1. 在独立仿真中采集

在项目根目录运行：

```bash
python openclaw_control/dmp_collect_trajectory.py \
  --task OpenclawPickCube --seed 0 --output pick
```

采集会自行启动独立仿真，不需要 HTTP 服务。`--output` 必填，只传名称
（也可带 `.npz` 后缀），目录会在保存时自动创建。上述命令保存到
`openclaw_control/trajectory/pick_trajectory.npz`，与启动命令的工作目录无关。
不接受目录路径；已带 `_trajectory` 的名称不会重复添加后缀。同名文件会被覆盖。

操作流程：

1. 等待仿真启动，先熟悉控制并调整好开始位姿；按 `Z` 前不会记录轨迹。
2. 按 `Z` 开始录制，使用移动、旋转和夹爪按键完成示范。
3. 夹爪开合时自动分段；需要额外的阶段边界时按 `X` 手动分段。`X` 不会暂停录制。
4. 动作完成后及时按 `C`，保存轨迹并退出。`Esc` 退出且不保存本次采集；再次按 `Z` 会清空当前记录并重新开始录制，不会重置仿真场景。

| 按键 | 操作 |
| --- | --- |
| `2` / `8` | X 正向 / 负向移动 |
| `6` / `4` | Y 正向 / 负向移动 |
| `7` / `1` | Z 正向 / 负向移动 |
| `9` / `3` | Roll 正向 / 负向旋转 |
| `U` / `J` | Pitch 正向 / 负向旋转 |
| `I` / `K` | Yaw 正向 / 负向旋转 |
| 空格 | 切换夹爪开合：1 打开，-1 闭合 |
| `Z` | 开始 / 清空并重新录制 |
| `X` | 手动分段，继续录制 |
| `C` | 保存并退出 |
| `Esc` | 不保存并退出 |

每次移动按键事件调整目标位置 0.01 米，旋转按键事件调整目标角度 0.05 弧度。
轨迹记录的是世界坐标系下的实际 TCP 位姿（XYZ 和 Roll、Pitch、Yaw），夹爪记录控制指令。

**采样方式与停顿：** 录制期间每个仿真控制步保存一个轨迹点，控制频率为 50 Hz
（每步对应 0.02 秒仿真时间），不是只在按键时记录。即使不按键或机械臂已经静止，
也会持续采样；例如静止 3 秒仿真时间会留下约 150 个相同或接近的点。
夹爪切换和按 `C` 保存时还会额外记录当前状态。
当前采集和学习逻辑不自动去重或压缩静止区间，停顿会进入 DMP 学习并影响回放时序。
因此建议先规划操作，再按 `Z`；录制中尽量减少无意的等待，保留抓取、释放等操作所需的停顿。

### 2. 学习权重

```bash
uv run python openclaw_control/dmp_generate_weight.py \
  --input pick
```

`--input pick` 自动读取 `openclaw_control/trajectory/pick_trajectory.npz`，输出 `skill-dmp/weights/pick_weights.npy`。
也支持完整采集文件名或文件路径。输出名称由 `--input` 自动决定：去掉扩展名和末尾的 `_trajectory`，再添加 `_weights.npy`。
例如 `pick_obj_trajectory.npz` 输出为 `skill-dmp/weights/pick_obj_weights.npy`，无需指定 `--output`。
输出目录固定为 `skill-dmp/weights/`，与启动命令的工作目录无关。
支持新的 `.npz` 采集文件，也兼容旧采集脚本保存的分段字典和普通 `(N, 7)` 的 `.npy` 轨迹。
新权重保存各段模型参数、采样频率和原始示范轨迹，供回放绘图对比。

### 3. 在仿真中回放并修改中间点

```bash
uv run python openclaw_control/dmp_replay.py \
  --weights pick --task OpenclawPickCube --seed 0 \
  --midpoint 0 -0.20 0.05 0.30 \
  --goal -0.10 0.05 0.45
```

中间点表示“第几段轨迹的终点”，索引从 **0** 开始。若有 N 段，则中间点索引为
`0..N-2`；最后一段用 `--goal`。可重复传入 `--midpoint` 来修改多个点。
不指定的中间点沿用原示范。单段轨迹只能修改起点和终点，可在采集时用 `X` 添加分段。

`--start`、`--goal` 支持 `X Y Z`、`X Y Z ROLL PITCH YAW` 或完整七维动作；
`--midpoint` 在这些数值之前加索引。只传 XYZ 时保留原点姿态和夹爪状态。
所有坐标为世界坐标，位置单位为米，角度单位为弧度。
夹爪状态在段内保持不变，在分段连接处切换；七维目标可显式改变边界处夹爪状态。

回放默认采用示范起点。`--time-scale 2` 将运动时长加倍，
`--headless` 无窗口运行。可用下面的命令只生成轨迹，便于检查：

```bash
uv run python openclaw_control/dmp_replay.py --weights pick_obj_weights.npy \
  --generate-only --save-trajectory generated.npy
```

回放默认先用 plot 窗口显示原始示范与生成轨迹对比：
仅显示三维路径及 XYZ、RPY、夹爪状态的 2×4 总览（Figure 1）。
关闭图窗后再启动仿真，默认不保存图片。
横轴为采样点序号，曲线表示计划轨迹，不是仿真中实际执行后的 TCP 轨迹。
`--generate-only` 同样显示对比图，关闭图窗后退出；
`--plot-output comparison.png` 可显式额外保存两张图片，`--no-plot` 跳过绘图。
无图形界面运行时可使用 `--headless --no-plot`。
旧权重缺少示范数据时会优先在 `openclaw_control/trajectory/` 查找同名 `<名称>_trajectory.npz`，并兼容权重同目录的旧采集文件，
也可用 `--trajectory openclaw_control/trajectory/<名称>_trajectory.npz` 指定原始采集文件。

### 4. 通过服务端执行 DMP

先启动更新后的服务端，再运行：

```bash
uv run python skill-dmp/scripts/dmp_action.py --weights pick_obj_weights.npy \
  --midpoint 0 -0.20 0.05 0.30 --goal -0.10 0.05 0.45
```

默认从 `/arrival_status` 返回的实际 `tcp_action` 起步，也可传 `--start`。
权重参数支持名称（`--weights pick` 自动读取 `skill-dmp/weights/pick_weights.npy`）、`skill-dmp/weights/` 中的文件名或任意权重文件路径。
`--dry-run` 只生成不发送，配合显式 `--start` 使用时不需要服务端。
`--save-trajectory <文件名>` 将生成结果统一保存到 `openclaw_control/trajectory/`，自动创建目录；`--server-url` 可修改服务地址。
夹爪切换默认保持 0.5 秒，并等待到位；终点同样等待到位。
`--gripper-hold` 和 `--arrival-timeout` 可调整保持及超时时间。
发送目标频率取自权重，实际速度也受服务端处理耗时影响。
服务端拒绝动作、连接出错或到位超时后停止发送后续轨迹点。

原单点动作脚本已更名为 `direct_action.py`：

```bash
uv run python skill-direct/scripts/direct_action.py -0.30 0 0.30 -3.14 0 0 1
```

更新后的服务端在 `/state` 和 `/arrival_status` 中新增 `tcp_action`，
原 `current_action` 字段继续表示目标动作；已启动的旧服务端需重启才能使用实际 TCP 起点。

无仿真依赖的核心流程与 HTTP 行为测试：

```bash
python -m unittest discover -s tests -v
```

## 开源协议

项目协议 Apache License Version 2.0。
