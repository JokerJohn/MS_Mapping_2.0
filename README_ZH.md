# MS-Mapping 项目概览

## 1. 项目简介

MS-Mapping 是一个基于 ROS 的多会话激光雷达 SLAM 框架，适用于需要多次采集与重复定位的大规模场景。系统融合激光里程计、GNSS 观测与闭环检测，以保持全局一致的位姿图，并提供地图管理、可视化及结果导出的配套工具。

### 项目初衷

- 在 **lio-sam_6axis** 项目中，许多用户需要带 GNSS 的闭环优化，但原始 LIO 不够稳定，常导致建图失败。
- **ms-mapping** 虽具备多会话鲁棒能力，但偏学术研究，缺乏 GNSS 融合，像初始化等环节不够便捷。
- 因此本项目定位为纯工程化系统：紧扣 GNSS 构建高鲁棒、高精度建图，只保留在实际工作中被证明实用的功能，而不追逐论文里“酷炫”但不落地的特性。

目前我日常建图均依赖这一套系统，除了极端环境，几乎不会遇到建图失败。

## 2. 核心能力

- **多会话定位**：支持在已优化地图上快速复定位，可按需求选择基线模式（Map-to-Frame / Frame-to-Frame）。
- **传感器融合**：融合激光里程计、GNSS 约束（XYZ / 姿态先验 / XY 平面）及可选 IMU 因子（Fast-LIO）。
- **稳健闭环**：同时支持单会话与跨会话闭环，配合 ICP、Scan Context、视觉等多种检测策略及可配置阈值。
- **灵活初始化**：初值来源可为 YAML 配置、RViz `initialpose`、RTK GNSS，并内置回退策略与航向扫描。
- **地图管理**：提供重建全局地图、提取局部子图、导出轨迹/协方差/点云的配套工具链。
- **运行监控**：输出详尽 ROS 日志，并在 RViz 中实时展示位姿图、GNSS 约束、闭环边及局部/全局点云。

## 3. 系统架构

```
ROS
 ├── ms_mapping（核心位姿图节点）
 │   ├── pose_slam(): 实时 LiDAR SLAM 主循环
 │   ├── InitSystem(): 多源初始化流程
 │   ├── OptimizeGraph(): ISAM2 增量优化
 │   └── VisualizationThread(): RViz 发布（闭环、GNSS、地图）
 │
 ├── cloud_process
 │   ├── ICP / GICP 配准工具
 │   ├── 局部 / 全局子图构建
 │   └── Scan Context 与特征提取辅助
 │
 ├── data_saver
 │   ├── 位姿 / 点云序列化
 │   ├── 因子图持久化（g2o、TUM、ROS bag）
 │   └── 离线地图重建脚本
 │
 └── fastlio（可选后端）
     ├── IMU 预处理
     ├── 点云去畸变
     └── 高频激光里程计
```

## 4. 模块说明

| 模块 | 功能 |
| --- | --- |
| `ms_mapping` | 核心位姿图 SLAM 节点，订阅激光里程计、GNSS、IMU，管理关键帧、优化与可视化线程。 |
| `cloud_process` | 点云预处理、ICP/GICP 配准、闭环检测辅助及子图生成。 |
| `data_saver` | 数据离线导出（TUM、g2o、ROS bag）及历史地图重建。 |
| `fastlio`（可选） | Fast-LIO 前端，实现 IMU 融合与去畸变点云。 |
| `launch/` | 可直接运行的 launch 文件（含 bag 回放与 RViz 配置）。 |
| `config/yaml/` | 针对不同传感器的参数模板（如 Robosense、Pandar、Mid360），含多会话默认配置。 |
| `config/rviz/` | RViz 可视化布局，展示位姿图边、GNSS 约束、点云及诊断信息。 |
| `scripts/` | 各类辅助脚本（bag 格式转换、轨迹可视化、栅格地图生成等）。 |

## 5. 初始化模块

`ms_mapping` 内的 `InitSystem()` 负责多源初始化流程：

- 当 `initial_pose_type=0` 时，从 YAML 读取初始位姿并校验配置；
- 支持 RViz `/initialpose` 交互式姿态输入，并可重复发布旧会话地图，协助手动对齐；
- 在 `initial_pose_type=2` 下融合 GNSS 数据，包含航向回退估计、航向搜索（yaw sweep）及附近局部地图提取；
- 对每个候选初值执行多次（并行）ICP 匹配，筛选最优结果后再插入图节点与约束，确保图优化的稳定启动。

该模块确保无论是新建会话还是针对历史地图复定位，都能获得一致、可靠的起始状态。

## 6. 使用流程

1. **数据准备**：采集或获取包含激光雷达、GNSS、IMU 话题的 ROS bag。
2. **参数配置**：根据传感器与基线模式（F2F / M2F）选择或修改 YAML（`pgo`、`common`、`lio` 等部分）。
3. **启动运行**：使用提供的 launch（如 `ms_robosense.launch`）启动 ms_mapping、辅助节点、RViz 及（可选）bag 回放。
4. **实时监控**：在 RViz 中查看关键帧、闭环边、GNSS 约束（绿色）以及局部/全局地图叠加。
5. **在线优化与保存**：位姿图实时优化，可通过服务 `/save_map` 导出地图、轨迹或重新开始会话。
6. **多会话流程**：后续会话加载历史地图（`map_directory`），依赖 GNSS/RVIZ 初始化，闭环与 GNSS 约束确保全局一致性。

## 7. 参数要点

- `pgo/baseline`：`0` 表示 F2F，`1` 表示 M2F 复定位。
- `pgo/useMultiMode`：启用多会话逻辑与旧图加载。
- `pgo/historyKeyframeSearchTimeDiff`：闭环候选的时间间隔阈值，回放同一时段数据时可调小。
- `common/useGPS` 与 `common/gnss_constraint_type`：控制 GNSS 融合模式（XYZ / 姿态先验 / XY）。
- `common/initial_pose_type`：选择初值来源（YAML=0、RViz=1、GNSS=2）。
- `lio/*`：Fast-LIO 前端参数（若启用）。

## 8. 可视化

- **闭环约束**：黄色/青色边表示单会话/跨会话闭环。
- **GNSS 因子**：绿色节点/连线，便于与闭环区分。
- **局部地图**：初始化与闭环阶段发布的局部点云。
- **全局地图**：发布在 `/pgo_map` 的关键帧地图，用于多会话对齐。

## 9. 日志与诊断

- 控制台日志使用彩色输出（如绿色 “ADD GPS Factor”），便于跟踪 GNSS 因子添加。
- 每 10 帧输出一次 `t1–t5` 统计，便于评估各阶段耗时。
- 警告信息具备节流，便于关注异常（如 GNSS 数据 NaN、闭环失败）。
- `/save_map` 服务会输出状态并在 `save_directory/sequence/` 下保存结果。

## 10. 快速上手

```bash
# 克隆 & 编译
cd ~/catkin_ws/src
git clone <仓库地址>
catkin build ms_mapping

# 使用示例配置运行
roslaunch ms_mapping ms_robosense.launch

# （可选）触发地图保存
rosservice call /save_map
```

## 11. 参考资料

- `config/yaml/*.yaml`：面向不同传感器的参数模板及多会话预设。
- `scripts/convert_custom_bag_to_standard.py`：数据规范化脚本。
- `scripts/visualize_pose_graph.py`：离线查看位姿图结果。
- 相关论文及 Fast-LIO、Scan Context 官方文档。

## 12. 保存数据输出

调用 `/save_map`（或运行 `data_saver` 逻辑）后，会在 `save_directory/sequence/` 目录下生成：

- `pose_graph_3d_result.txt`：优化后位姿与协方差（TUM 格式）。
- `data_time.txt`：关键帧对应的阶段耗时统计（`t1–t5`）。
- `key_point_frame/`：关键帧点云（降采样）用于地图重建或多会话匹配。
- `logs/`：运行日志与参数快照（如 `runtime_params.yaml`）。
- 可选导出的对齐 ROS bag（`*_result.bag`），包含点云与里程计。
- 若启用 `common/save_intensity_image` 或 `common/save_grid_map`，还会生成 `intensity/` 图像与 `grid_map/` 栅格地图。

这些成果便于离线分析、地图拼接或多会话复定位。

## 13. 脚本亮点

`scripts/` 目录下常用脚本包括：

- `convert_custom_bag_to_standard.py`：统一不同数据源的 topic 命名。
- `visualize_pose_graph.py`：快速可视化位姿图与闭环边。
- `generate_gridmap.py`：依据关键帧生成占据或强度栅格地图。
- `compare_pointcloud_bags.py`：对比不同 bag 的点云覆盖情况与密度。
- `inspect_pointcloud_fields.py`：检查点云字段是否满足处理要求。

充分利用这些工具，可加速多会话数据的准备、验证与展示。

## 14. 许可与贡献

许可信息请参考仓库中的 License 文件。欢迎通过 Pull Request 参与贡献，提交前请遵循代码风格与测试要求。
