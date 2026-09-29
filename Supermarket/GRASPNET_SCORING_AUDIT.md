# GraspNet scoring audit

本文件是对当前 GraspNet -> RealMan/RM65 -> CuRobo 链路的专项诊断。仓库中未找到上一轮同名审计文件，因此本文件为新建审计，不覆盖其他用户改动。结论只使用代码和现有日志；没有保留的 RGB-D 帧、原始候选快照或受控消融结果的地方，明确标为 `NOT MEASURED`，不把推测写成事实。

## GraspNet 候选全部无法满足评分要求的专项诊断

### 0. 结论先行

当前线上链路没有一个独立的“最终评分公式/最终分数阈值”。`THRESH_GOOD=0.7` 位于 GraspNet 训练 loss 的正负样本定义中，不是推理时的 `score > threshold`；线上只有模型 score 排序、碰撞检测、NMS、mask/几何筛选和 CuRobo 两段规划。因此“当前评分要求过严”目前不能计算，正确状态是 `INVALID/NOT IMPLEMENTED`，不能据此判定模型分数不够。

在可追溯的 50 次 UI 触发中，主要可见瓶颈是：

1. **GraspNet 碰撞检测与 NMS 的合并阶段**：50 次中 36 次（72%）在该阶段后为 0 个候选；由于没有记录 raw decode 数，无法给出候选级 rejection ratio。
2. **目标 mask 中心过滤**：88 个输入中保留 69 个，淘汰 19 个，rejection ratio 为 21.6%。
3. **脚本内 pre-grasp CuRobo 规划**：69 个输入中淘汰 1 个，rejection ratio 为 1.45%。

这不等于 GraspNet 网络本身坏了。当前 UI 每次都传 `--open y --align-base-z y`，GraspNet 实际看到的是合成圆柱点云，之后又把原始网络旋转重新构造成参考 Z/approach 约束姿态；这是一种强后处理/任务建模条件。更关键的是，脚本内使用 RealMan base 转换后的 CuRobo 预检可以通过，而同一左臂姿态随后由驻留式 planner 以 `frame=left_base` 重新解释并报 `IK_FAIL`，存在高置信度的跨进程 frame 语义不一致风险。

因此当前根因判断是 **Mixed，但优先级为接口/姿态后处理 > 机器人执行接口 > 碰撞/ROI 过滤 > 网络质量**。没有证据支持“全部是模型泛化能力不足”。

### 1. 候选死亡漏斗

以下数字来自 `/tmp/supermarket_grasp_ui_grasp.log` 的 50 个异质触发块（右臂 45 次、左臂 5 次；均为 `--open y --align-base-z y --select-best 3`）。这些是跨多次运行的累计数，不是同一帧的单次流水线；raw stage 没有日志，不能用后级数字反推。

| Stage | Input | Pass | Reject | Reject ratio | 证据/状态 |
| --- | ---: | ---: | ---: | ---: | --- |
| GraspNet raw decoded | 未记录 | 未记录 | 未知 | `NOT MEASURED` | `pred_decode` 输出后没有打印长度/全量分布 |
| score threshold | 未实现 | 未实现 | 未知 | `INVALID` | 没有运行时 `score > threshold` |
| workspace / ROI | 未单独记录 | 未单独记录 | 未知 | `NOT MEASURED` | 直接脚本只做深度范围和 mask/bbox 中心检查 |
| GraspNet collision + NMS | 未知 raw | **88** | 未知 | 候选级不可算 | `detect_grasps` 先碰撞再 NMS，日志只打印合并后的结果 |
| 运行级 zero-after-collision+NMS | 50 次 | 14 次非零 | 36 次为零 | **72.0%（运行级）** | 不能等同于候选级淘汰率 |
| target center in object mask | 88 | **69** | 19 | **21.6%** | `Candidates with centers inside selected object mask` |
| cylinder interior / approach | 69 | 69 | 0 | 0% | 当前 `align-base-z y` 且指定 approach，原始 approach test 被延后 |
| cylinder surface | 69 | 69 | 0 | 0% | UI 为 `surface=any` |
| explicit orientation gate | 69 | 69 | 0 | 0% | `rejected orientation=0` |
| pre-grasp CuRobo | 69 | **68** | 1 | **1.45%** | `pregrasp_ik=1` |
| grasp CuRobo (script-side) | 68 | 68 | 0 | 0% | `grasp_ik=0` |
| joint-limit / singularity | 未单独记录 | 未单独记录 | 未知 | `NOT MEASURED` | CuRobo 状态没有按原因拆分 |
| NVBlox/environment collision | 当前脚本未接入 | 未知 | 未知 | `NOT MEASURED` | 驻留 planner 默认 `enable_nvblox=false` |
| final score threshold | 未实现 | 未实现 | 未知 | `INVALID` | 只有 `select-best` 排序，不是阈值 |
| final export | 68 个通过脚本预检 | 约 13 次非空发布 | 其余无发布 | 运行级统计 | `select-best=3` 只是输出上限 |

这里的“GraspNet collision-free”不是原始候选数，也不是纯碰撞结果：`realsense_grasp_detection.py` 在检测器之后立即做 NMS，再打印 88。因此不能把 88 当成“碰撞通过数”去和 raw 数相减。

### 2. Rejection ratio 排名

按目前能可靠量化的观测排序：

| Rank | Stage | 观测结果 | 置信度 | 解释 |
| --- | --- | --- | --- | --- |
| 1 | collision + NMS 合并阶段 | 36/50 次运行得到 0；候选级分母缺失 | 中 | 这是最大运行级瓶颈，但无法区分碰撞和 NMS 各自贡献 |
| 2 | object-mask center | 19/88 = 21.6% | 高 | 3D center 投影回图像后要求落在最终 mask 内 |
| 3 | pre-grasp CuRobo | 1/69 = 1.45% | 高 | 只统计进入脚本 CuRobo 的候选；不能解释 36 次零输出 |

width、TF、joint margin、singularity、environment collision、final-score 没有单独计数，不能伪造排名。若按“候选级”而不是“运行级”，第一名必须等 raw decode 计数补齐后重算。

### 3. GraspNet 原始输出统计

#### 3.1 可获得的 score 统计

当前日志只打印通过 collision/NMS、mask、脚本 CuRobo 后的 `CuRobo accepted score`，不是 raw score 分布。68 个值为：

| Statistic | Value |
| --- | ---: |
| count | 68 |
| min | 0.0705 |
| median | 0.5177 |
| mean | 0.49535 |
| P90 | 0.81109 |
| P95 | 0.89475 |
| max | 1.1746 |

分数并非全部很低，且超过 0.7 的值确实存在；但这些值已经经过多级筛选，不能代表网络 raw 分布。`pred_decode` 用 `grasp_score * grasp_tolerance / GRASP_MAX_TOLERANCE`，所以 score 超过 1 是实现允许的，并非异常。

保存的不同历史快照也显示同样现象：`left_click_grasp_predictions.npy` 的 score 为约 1.288、1.024、0.930，`right_click_grasp_predictions.npy` 为约 0.692、0.675、0.400；它们是选出的最终快照，不是 raw Top-K。

#### 3.2 width/depth/translation

代码层面：

- width 在 `pred_decode` 中先乘 1.2，再 clamp 到 `[0, 0.1] m`；没有单独的 width reject stage。
- depth 类别只取 4 个离散值 `(class+1)*0.01 m`，即 0.01/0.02/0.03/0.04 m。
- 当前保存快照 width 约在 0.0695--0.1000 m，未见因 width 超限而被淘汰。
- 当前 50 次 UI 日志的最终候选没有保存完整的 raw translation 分布；一次左臂实例的 RealMan base final position 为 `[0.077294, -0.464961, 0.367780] m`，相应 pre-grasp 由 0.08 m 沿 approach 反向构造。

要求中的 raw `candidate count / score min/max/mean/median/P90/P95 / width 分布 / depth 分布 / translation range / approach histogram / rotation matrix Top 20` 均需要在 `pred_decode` 后增加结构化快照才能获得，当前状态为 `NOT MEASURED`。现有 Open3D 代码可以画点云与 candidate axes，但本批日志没有保存窗口截图或帧 bundle，因此不能声称“视觉上全部合理”或“视觉上全部错误”。

#### 3.3 pose convention 和人工后处理

`pred_decode` 以 `approaching = -grasp_top_view_xyz` 作为 rotation 第 0 列（虚拟夹爪 +X approach），第 1 列为 jaw closing 方向，第 2 列为叉乘得到的 up 轴；可视化也标注虚拟夹爪 +X 为 approach。当前 `filter_grasps_by_approach` 同样读取 `rotation_matrices[:, :, 0]`，这一点在代码内是一致的。

但当前 UI 的 `--align-base-z y` 会在 `build_final_grasp_transforms` 中根据 reference-Z 和指定 approach 重建最终 orientation，原始网络 rotation 不再原样送入 CuRobo；同时还可能应用：

- 原始/对称 branch：绕模型 +X 旋转 180 度的等价分支；
- 左右臂 angle orbit；
- cylinder axis centering、surface constraint、grasp-height fraction；
- `T_grasp_model_tcp` 的模型 TCP 偏移。

所以必须把“raw pose 质量”和“任务约束重建后的 pose 质量”分开评估。当前证据只能判定 **raw pose 质量为 QUESTIONABLE/未充分观测，后处理改写是高风险接口条件**，不能判定 raw 网络本身 BAD。

### 4. 输入点云质量

#### 4.1 实际点数

日志中的 `Selected-object points` 为 50 个目标 mask 的有效深度点数：

| Statistic | Value |
| --- | ---: |
| count | 50 frames |
| min | 4,921 |
| median | 16,560 |
| mean | 16,500 |
| max | 22,976 |
| GraspNet sampled input | 20,000（不足时有放回重复采样） |

因此不是“目标只有几百点”的情况，点数本身通常足够。但这不是完整点云质量证明：现有日志没有保存与每次 mask 配对的 raw depth，因此无法计算 NaN/Inf/zero ratio、深度 histogram、断层/飞点、scene/background 比例。

#### 4.2 GraspNet 到底看了什么

当前两条代码路径不同：

1. `realsense_click_grasp.py` 的 `make_selected_cloud` 用 `mask & depth_valid` 生成 `scene_cloud`，GraspNet sampled cloud 也从同一个目标区域抽样；碰撞检测只看到目标物点云。这会让“手指与目标接触”更容易被近似 collision detector 当作碰撞，且完全看不到货架/邻近物体。
2. `grounded_sam2_ros2/graspnet_bridge.py` 的 object cloud 用 SAM mask，但 `scene_cloud` 是整幅图所有有效深度点；这条路径的 collision 语义更接近场景碰撞，不能和上一条的候选数量直接比较。

当前 UI 还总是传 `--open y`，把测得的 object cloud 替换为 synthetic cylinder。故本次日志不能用来证明网络对真实商品点云的泛化；这是高影响的输入/任务建模条件。

#### 4.3 RGB/depth/mask/intrinsics 状态

| Check | Status | 证据/限制 |
| --- | --- | --- |
| depth `>0`、`0.15--1.50 m` | PASS（代码级） | 直接脚本在投影前过滤 |
| NaN/Inf | RISK | 直接脚本主要依赖 depth>0/范围；ROS bridge 有 `isfinite`，但当前 UI 帧未保留统计 |
| RGB/depth alignment | PASS（代码/接口级） | RealSense 用 align-to-color；ROS topic 为 aligned depth |
| mask 有效点数 | PASS/RISK | 4,921--22,976；但 mask 形状/边缘质量未保存 |
| 背景/货架混入 | 路径相关 | direct UI 目标-only；bridge scene cloud 为全场景 |
| 目标正面薄层/断层 | `NOT MEASURED` | 无配对帧，不能从候选日志推断 |

### 5. 坐标与单位检查

| Quantity | Current unit/convention | Status |
| --- | --- | --- |
| RealSense z16 depth | integer raw * `depth_scale` -> m | PASS（direct/bridge 代码均有路径） |
| frame-bundle depth | `depth.astype(float32) * depth_scale` | RISK：必须确认 bundle 的 depth 是否仍为 raw integer |
| point cloud xyz | m, optical +X right, +Y down, +Z forward | PASS（投影公式明确） |
| GraspNet translation/width/height/depth | m | PASS；depth 为 0.01 m 离散值 |
| voxel size | 0.01 m | PASS |
| collision threshold | dimensionless IoU-like ratio, default 0.01 | PASS/RISK：语义不是米，容易误解 |
| collision approach distance | 0.05 m | PASS |
| pre-grasp distance | 0.08 m | PASS |
| TCP/hand-eye translation | m | RISK：`tcp_transform_verified=false` |
| CLI `--angle` | degrees; internal RPY/solver | PASS（转换处明确） |
| joint input | RealMan degrees -> CuRobo radians | PASS（脚本预检）；resident planner must match |
| stale Open3D JSON | 1850x1016 old camera config | FAIL for current 640x480 use |

没有在当前代码中发现明确的 mm/cm 直接当 m 的单点错误；但 `frame-bundle` 的 depth_scale 和两个 planner 的 frame 名称仍需在同一时刻做数值校验。

### 6. 相机内参与 ROI

直接 RealSense 路径从当前 aligned depth profile 读取 intrinsics；frame bundle 要求 `fx/fy/ppx/ppy/depth_scale` 并检查 RGB/depth shape。投影代码为 `x=(u-ppx)z/fx`、`y=(v-ppy)z/fy`，没有发现 resize 后未同步修改内参的代码证据。

但日志只记录了 mask 像素数，没有记录每帧 `fx/fy/cx/cy/depth_scale`、RGB/depth shape 或 timestamp。故运行时数值一致性应标为 `RISK`，不能仅凭源代码升级为 PASS。

ROI/mask 的确产生了可量化损失：88 个候选中 19 个中心投影落在最终 mask 外。当前 mask 是 bbox 内深度 seed 的连通区域；bbox 太紧、深度容差 0.04 m、遮挡或只保留正面薄层都可能造成漏边，但没有配对图像无法区分哪一种。

### 7. checkpoint、模型和版本

| Item | Observed value | Status |
| --- | --- | --- |
| checkpoint | `/home/lh/Supermarket/graspnet-baseline/checkpoint-rs.tar` | PASS（文件存在） |
| checkpoint size/mtime | 12,468,415 bytes，2026-08-10 | 记录值 |
| epoch | 18 | PASS（运行日志打印） |
| network | local `GraspNet`, input feature dim 0, 300 views, 12 angles, 4 depths | PASS（代码） |
| input points | 20,000 | PASS |
| backbone/training provenance | 未在代码或 checkpoint 元数据中完整记录 | `NOT MEASURED` |
| official vs fine-tuned | 来源无法确认 | `RISK` |
| `pred_decode`/`GraspGroup`/collision detector | 同一 local `graspnet-baseline` tree | 兼容性较高 |
| `graspnetAPI` exact package version | 未固定/未记录 | `RISK` |

没有发现字段顺序或 rotation decode 已经错位的直接证据，但 checkpoint 来源和环境版本不具备可复现记录，仍需补充 hash、git commit 和依赖版本。

### 8. 夹爪尺寸、width 和 GraspNet collision

当前 `ModelFreeCollisionDetector` 使用的近似尺寸为 `finger_width=0.01 m`、`finger_length=0.06 m`，voxel size 默认 `0.01 m`；调用时 `approach_dist=0.05 m`、`collision_thresh=0.01`。代码没有 finger height/thickness、真实 TCP jaw offset 或真实夹爪 CAD 的统一配置。

真实夹爪尺寸在当前 Supermarket/robot 配置中没有找到可验证的同名参数，因此“模型夹爪和实机是否一致”为 **未找到/高风险**，不能把 detector 的 collision-free 等同于真实可抓取。

width 由网络 clamp 到最大 0.1 m，当前保存快照在约 0.0695--0.1000 m，日志中没有 width reject stage。也就是说，当前“候选全死”不是被一个显式 opening-width 阈值淘汰；更需要确认真实 RM65 夹爪有效开口、指尖厚度和 `T_grasp_model_tcp` 是否对应同一个模型。

碰撞还有两个语义风险：

- direct UI 的 scene cloud 只有目标物，手指接触目标本来是必要动作，却可能被 detector 判作 collision；
- synthetic cylinder 模式用合成物体几何，不能反映真实货架/邻近物的碰撞。

当前没有完成 `collision_thresh={-1,0.005,0.01,0.02}` 或不同 voxel/approach 的受控离线敏感性测试，结果为 `NOT MEASURED`。不得根据 36 次 zero 直接永久放宽阈值。

### 9. 姿态变换、TF 和人工 rotation 修正

当前主变换链是：

```text
T_base_tcp_capture @ T_tcp_camera @ T_camera_grasp @ T_grasp_model_tcp
```

单位是 m/rad，代码对 rotation 正交性做检查。`build_final_grasp_transforms` 再按 reference-Z、approach、cylinder constraint 重建最终姿态，pre-grasp 为 `final - approach * 0.08`；这个方向与 GraspNet +X approach 的定义一致。

代码中确实存在人为姿态后处理，而不是简单原样透传：

- `candidate_rotation @ symmetry` 尝试绕模型 +X 的 180 度对称 branch；
- `align-base-z` 改写姿态基准；
- 左右臂使用相反 angle sign；
- cylinder 轴心/表面/高度约束移动 translation 和 orientation。

因此“网络原始 pose 正常、后处理后失败”是可行路径，必须保存 raw/final 两套矩阵做逐候选对照。

#### 左臂 frame 证据

脚本日志对一个左臂 candidate 同时打印：

```text
RealMan base grasp = [ 0.077294 -0.464961  0.367780 ] m
CuRobo driver_base = [-0.367780 -0.464961  0.077294 ] m
```

脚本内用 `realman_base_to_curobo_transform` 后报告 `pregrasp_ik=0, grasp_ik=0`；但 ROS 消息在 `frame=left_base` 发布 RealMan base 数值，驻留 planner 随后也按 `left_base` 收到该消息并出现 `IK_FAIL`。`wrist_camera_tf.py` 明确指出 ROS/CuRobo `left_base` 是 driver_base，需要左臂转换。这个“消息 frame 名称与数值所属 frame 不一致”的证据等级为 **高**，是当前最优先修复的接口问题。

右臂/左臂 `T_grasp_model_tcp` 在 `grasp_runtime.json` 中存在非零偏移，且当前标记 `tcp_transform_verified=false`。右臂的 `-0.10 m` 偏移可能是有效标定变更，但在没有实测验证的情况下不能假定正确。

### 10. IK、pre-grasp、CuRobo 与 NVBlox

#### 10.1 脚本内 IK 不是 resident planner 的同一条件

`filter_grasps_by_curobo_ik` 对每个 candidate 做两段 `plan_curobo_pose`：当前关节 -> pre-grasp，再用 pre-grasp endpoint -> final。它使用脚本内创建的 MotionGen、placeholder world、`interpolation_dt=0.02`，并且只记录 orientation/pregrasp_ik/grasp_ik 汇总，不记录底层 `MotionGenStatus` 的位置越界、姿态不可达、关节限位、seed 不收敛或奇异性分类。

驻留 `/curobo_realman_test` planner 则重新读当前状态、按 ROS frame 转换 target、直接对当前 -> goal 做一次 `plan_single`，默认 `interpolation_dt=0.008`，并可选择 NVBlox。它不是脚本两段规划的复现。因此“GraspNet 阶段 IK 通过但 CuRobo 最后 IK 失败”并不矛盾：两者输入 pose frame、起始关节、路径分段、world 和参数均可能不同。

#### 10.2 失败分类的现状

| Failure class requested | Current evidence |
| --- | --- |
| position outside workspace | resident planner 有 0.90 m 距离上限；候选级状态未保留 |
| orientation unreachable | 脚本有 orientation gate 计数，但本批为 0；resident `IK_FAIL` 未拆分 |
| joint limit | 未单独记录 |
| solver/seed convergence | 未单独记录 |
| left/right base frame error | 左臂存在高置信度数值/名称不一致证据 |
| singularity | 未单独记录 |
| NVBlox/world collision | 当前 resident 默认关闭；开启时受 unknown policy 影响 |

验证结果 `test_003` 显示 NVBlox `unknown_is_collision=true` 可导致 `START_IN_COLLISION`，`test_004` 改为 false 后规划成功。这是条件性风险，不是当前 UI 日志中已证明的主因。

#### 10.3 pre-grasp 方向和距离

代码为 `pregrasp = final - approach * distance`，且 synthetic-cylinder interior filter 也以同一公式检查 pre-grasp 在圆柱外、向内运动。当前 UI 因 `align-base-z y` 会跳过 raw camera approach gate，改在 base reference-Z 后由 CuRobo 检查；默认距离为 0.08 m。现有日志只有 1/69 pregrasp failure，未做 0.05/0.08/0.10/0.12 m 消融，因此不能声称距离过大是主因。

### 11. 当前最终评分阈值可达性

当前仓库没有找到类似下面的线上实现：

```python
final_score = w1 * grasp_score + w2 * clearance_score + ...
qualified = final_score > threshold
```

实际存在的分数路径是：`pred_decode` 生成 GraspNet score -> sort -> collision -> NMS -> `select-best`。`THRESH_GOOD=0.7` 只在 loss 中标记训练 grasp quality，不能作为线上 qualified threshold。

因此：

- final score 原始项、权重、归一化范围：`NOT IMPLEMENTED`；
- 理论最大值与当前阈值比较：不可计算；
- “阈值是否过严”：状态 **INVALID/NOT IMPLEMENTED**，不是 `TOO STRICT`；
- 不能把 0.45 左右的 accepted mean 与 0.7 直接比较，因为当前没有 0.7 的最终分数门槛。

同时，若未来新增 composite score，必须先把 m、rad、bool、GraspNet score、clearance 等项分别归一化，再讨论 hard reject/soft score；当前没有证据表明已有“一票否决过多”的 final-score 组合。

### 12. 原始 pose 可视化结论

本次没有可复用的 paired RGB-D/mask frame bundle，也没有保存 Top 20 raw/collision-free/after-IK 三组窗口或截图；抓取节点最后会清理临时 frame bundle。因此无法诚实给出“视觉上原始一定合理/一定错误”的二值结论。

能下的代码级结论是：

1. GraspNet +X/closing/up 轴 decode 和 direct filter 的轴定义是一致的。
2. UI 强制 synthetic cylinder、align-base-z、approach、angle 和 TCP offset，确实会改写 raw rotation/translation。
3. 脚本日志中的若干 final pose 在脚本内能完成两段 CuRobo，而相同左臂消息在 resident planner 中 `IK_FAIL`，因此当前最有证据的解释是“接口/最终姿态条件不一致”，而不是已证明的 raw pose BAD。

结论标签：**Raw GraspNet pose quality = QUESTIONABLE（数据不足）；转换后/跨进程一致性 = RISK；机器人约束失败 = 已观测但条件不一致。**

### 13. Top near-miss candidates

由于日志只保留通过后的 score 和 aggregate reject count，没有 candidate ID、rank、底层 MotionGenStatus 或失败 pose，无法建立要求中的 Top 50 逐候选矩阵。当前能复原的近失效样本如下：

| Near miss | Evidence | Difference to success |
| --- | --- | --- |
| 左臂同一 candidate | 脚本打印 RealMan base 与 driver_base 两套坐标并接受；驻留 planner 对 `frame=left_base` 报 `IK_FAIL` | 不是网络 score 差，而是跨节点 frame/规划条件不同 |
| 50 次中的 1 个 pre-grasp reject | `Pose/CuRobo filter: kept 68/69`, `pregrasp_ik=1` | 只差 pre-grasp 段，具体状态未记录 |
| exact-time smoke candidate | GraspNet candidate_count=6、selected score=1.143469；后续因目标低于 workspace 校验而未触发 planning | GraspNet 合法不等于 robot workspace 合法 |
| accepted low-score tail | accepted score min=0.0705 | 仍通过脚本 IK，说明 score 不是当前硬门槛；不能称 final-score near miss |
| mask near misses | 19 个中心投影在 mask 外 | 可能是 bbox/mask/深度边缘问题，缺少帧无法进一步归因 |

### 14. 参数敏感性与消融

本次没有执行会改变候选数量的受控离线消融，以下全部为 `NOT MEASURED`：

| Ablation | Required comparison | Current result |
| --- | --- | --- |
| score threshold | current / -10% / -20% | 无线上 threshold，无法执行 |
| NMS | ON / OFF | 代码固定执行，日志无法分离 |
| collision | `0.005/0.01/0.02` 或 `-1` | 未跑；不要直接改默认值 |
| voxel/approach | 0.005/0.01 m，0.03/0.05 m | 未跑 |
| pre-grasp distance | 0.05/0.08/0.10/0.12 m | 未跑 |
| grasp IK only | 去掉 pre-grasp，仅离线诊断 | 未跑 |
| CuRobo | 只做 IK/不做 path | 未跑 |
| final score | 去掉 final threshold | 当前没有该模块 |
| symmetry | raw branch vs 180-degree +X branch | 代码会尝试 branch，但未统计逐 branch 成功率 |

建议的安全诊断顺序是：固定一帧 RGB-D/mask 和随机种子，保存 raw decode；只在 plan-only/不发机器人命令的模式下依次开关 NMS、collision、pre-grasp 和对称 branch；同时把每个 candidate 的 `MotionGenStatus`、target frame 和所有中间矩阵写成 JSON。没有这组对照，不应以“放宽阈值后有候选”作为修复结论。

### 15. 失败原因矩阵（当前可观测粒度）

要求的逐候选表：

| Rank | Score | Collision | IK | PreIK | Joint | Approach | CuRobo | Final | Main reject |
| --- | ---: | --- | --- | --- | --- | --- | --- | --- | --- |
| Top 50 raw | 未记录 | 未记录 | 未记录 | 未记录 | 未记录 | 未记录 | 未记录 | 未实现 | `NOT MEASURED` |
| Aggregate 88 post collision+NMS | 已部分记录 | 已过合并阶段 | 未进入/部分进入 | 未进入/部分进入 | 未记录 | current approach deferred | 部分 | 未实现 | 只能做阶段汇总 |
| 69 entered script CuRobo | 68 accepted scores | 已过 | 68 final path pass | 68 pass, 1 reject | 未拆分 | 0 orientation reject | 68 pass | 未实现 | pregrasp 1 |

这张表的缺口不是算法猜测，而是日志 schema 缺少 raw candidate ID、阶段前后索引和 solver status。下一轮若要回答“每一个候选为什么死”，必须先补日志，而不是只增加最终 print。

### 16. 根因排序

#### Root Cause 1: 左臂消息 frame 与数值所属 frame 不一致

- **证据**：脚本把 RealMan base 转成 CuRobo driver_base 后可通过；发布端却把 RealMan base 数值标为 `left_base`；resident planner 对该消息 `IK_FAIL`；`wrist_camera_tf.py` 明确要求左臂 driver-base 转换。
- **影响**：同一位置/姿态在两个 planner 中不是同一坐标，能直接造成“GraspNet 阶段 IK 通过、最终 CuRobo IK 失败”。
- **置信度**：高。

#### Root Cause 2: UI 使用 synthetic cylinder + 强制姿态重建，raw GraspNet 质量被后处理条件混合

- **证据**：50 次日志均为 `--open y --align-base-z y`；source 明确在 `align-base-z` 后重建 approach/orientation。
- **影响**：网络对真实商品的点云泛化和最终机械臂姿态可行性被混在一起，raw candidate 不能直接等价为 final TCP pose。
- **置信度**：高（设计事实）；对失败比例的数值贡献：未单独消融，不能量化。

#### Root Cause 3: collision + NMS 后候选大量归零

- **证据**：36/50 次运行在合并阶段后为 0；detector 使用 1 cm voxel、0.01 threshold、5 cm approach，且 direct scene cloud 只含目标。
- **影响**：运行级最大可见瓶颈；可能是 collision 过敏、目标接触被判 collision 或 raw/NMS 分布问题。
- **置信度**：中；候选级 raw 分母和 NMS/collision 单独消融缺失。

#### Root Cause 4: object mask center filter

- **证据**：19/88 = 21.6% 被投影 mask 拒绝。
- **影响**：bbox/深度连通区域边缘的候选直接丢失，可能减少本来可用的侧向抓取。
- **置信度**：中高；具体是 mask 漏边还是网络 center 偏移，需帧可视化。

#### Root Cause 5: TCP/真实夹爪参数未验证

- **证据**：`tcp_transform_verified=false`；真实 finger height/thickness/opening 在当前配置中未找到；detector 只用 1 cm/6 cm 近似尺寸。
- **影响**：collision-free、TCP pose 和实机几何可能不一致。
- **置信度**：中高；需要标定板/夹爪 CAD 对照。

#### Root Cause 6: resident CuRobo/NVBlox 条件与脚本预检不同

- **证据**：脚本两段 plan、placeholder world、dt=.02；resident 单段 plan、dt=.008、可启用 NVBlox；NVBlox unknown policy 的独立验证可改变结果。
- **影响**：预检通过不保证最终 planner 通过。
- **置信度**：中；左臂 frame 问题证据高，NVBlox 对当前 UI 是否启用仍需现场确认。

#### Root Cause 7: GraspNet 网络/权重本身

- **证据**：accepted score 分布并非全低，存在 >1 和 >0.7；点数通常 4,921--22,976；没有 raw pose 统计和可视化。
- **影响**：不能排除 checkpoint 场景泛化问题，但现有数据不足以将其排第一。
- **置信度**：低到中。

### 17. 三类问题归类

| Category | Findings |
| --- | --- |
| A. GraspNet 本身 | raw 分布/真实商品可视化缺失；checkpoint provenance 未确认；当前没有证明“网络输出全部异常” |
| B. GraspNet -> Robot interface | 左臂 frame label/value mismatch（高）；`T_grasp_model_tcp` 未验证；synthetic-cylinder 和 align-base-z 改写 pose；真实夹爪尺寸未找到 |
| C. Robot execution constraints | CuRobo 两条路径条件不同；pre-grasp 1/69；resident IK_FAIL；NVBlox unknown policy 条件风险；最终评分模块未实现 |

综合结论：**Mixed，主责在 B + C；A 尚未被证明是主因。**

### 18. 最终回答

为什么现有 GraspNet 生成的 grasp 几乎全部无法通过当前评分？

严格按现有证据，答案不是“GraspNet 模型不好”，而是多个条件叠加：

1. 运行时没有可计算的 final-score threshold，所谓“评分不满足”不能归因于阈值。
2. UI 使用合成圆柱和强制 reference-Z/approach 后处理，raw pose 与最终 TCP pose 不是同一个对象。
3. collision+NMS 合并阶段在 72% 的触发运行中归零；mask 又淘汰 21.6% 的可见候选，这是候选数量下降的主要来源，但缺少 raw 分母和消融，不能把原因细分成 detector 或 NMS。
4. 脚本内 CuRobo 只淘汰 1/69 pre-grasp candidate，说明“GraspNet 阶段 IK 全部失败”与数据不符；随后 resident planner 的左臂 `IK_FAIL` 有明确 frame 语义不一致证据。
5. 实机夹爪尺寸、TCP 偏移和 resident world/NVBlox 条件未统一验证，导致 collision-free/IK-valid 不能等价于可执行。

相对贡献的可量化部分只有：运行级 zero-after-collision+NMS 72%、mask candidate rejection 21.6%、script pre-grasp rejection 1.45%。其余贡献目前应写为 `UNMEASURED`，不能编造比例。

### 19. 上位机完整流程、额外耗时和两次 IK 的关系

#### 19.1 实际流程

当前 ROS2 上位机不是“调用一次 GraspNet”这么短的路径，而是串行完成：

```text
Qwen/目标确认
  -> 读取 RealMan 当前关节和 TCP
  -> 获取 aligned RGB-D 与相机内参
  -> bbox/深度 seed/连通 mask
  -> 生成目标点云（当前 UI 还可能替换为 synthetic cylinder）
  -> 启动外部 realsense_click_grasp.py
  -> GraspNet forward/pred_decode
  -> collision detector + NMS
  -> mask/cylinder/approach 后处理
  -> 创建/预热脚本内 CuRobo
  -> current -> pre-grasp -> final 两段 PLAN_ONLY
  -> 保存 npy，并通过 ROS 发布
  -> resident CuRobo planner 重新读取状态并再次规划
```

`supermarket_grasp_ros2/grasp_node.py` 每次触发都会启动外部抓取子进程；`realsense_click_grasp.py` 又会在该进程中加载 checkpoint、创建 MotionGen/CUDA graph 并执行两段规划。手工“一个终端一个模块”时，常见情况是相机、Qwen 或 CuRobo 已经是 resident/warm 状态，且只跑了其中一段，不能和这条完整串行链直接比较。

#### 19.2 已观测的时间来源

现有日志能确认以下开销存在：

| Component | Observed timing/behavior | Why it matters |
| --- | --- | --- |
| Qwen | 单次推理约 11.66--12.85 s；UI 默认可能重复确认 | 目标确认本身就可能贡献十几秒，且在 GraspNet 前串行 |
| subprocess/import | 每次触发重新启动 Python；日志有 import/cuda warning 后才到 checkpoint | 解释器、模块、CUDA context 不复用 |
| GraspNet checkpoint | 每个抓取子进程重新 `torch.load`，日志打印 `Loaded checkpoint epoch 18` | 模型加载/显存初始化不是零成本 |
| CuRobo resident warmup | `/tmp/supermarket_grasp_ui_curobo.log` 中启动后约 11.2 s 才 ready | 首次手工测试若已预热，看起来会比完整启动快 |
| ROS/DDS/状态读取 | 抓取节点等待相机、Qwen 结果、当前 TCP/关节和 planner 响应 | 服务同步和消息往返会叠加等待 |
| two-stage planning | 脚本对每个候选做 current->pre-grasp，再做 pre-grasp->final | 这是完整流程的必要检查，不是单次 GraspNet forward |

一次左臂日志从抓取子进程 `RUNNING` 到 checkpoint load 约 24.9 s，随后才出现 accepted/publish；这说明现场的“额外十几秒”不能只归咎于 CuRobo solver，进程/模型/CUDA 初始化同样在关键路径。不同机器和缓存状态会改变绝对数，但串行阶段不会消失。

#### 19.3 为什么手工分终端会感觉更快

手工测试通常隐含了以下差异：

- 已经启动并预热了 RealSense、Qwen 或 resident CuRobo；
- 直接给定 bbox/点云，跳过 Qwen 的识别和 `RECHECK`；
- 只看 GraspNet 输出，跳过 mask、synthetic-cylinder、两段 CuRobo 或 ROS 发布；
- 只测右臂或单次目标，不包含上位机的服务同步、失败重试和日志/文件 I/O。

所以“每个终端单独看都快”不代表总链路应该是各单项冷启动时间的简单相加；完整流程把所有冷启动和串行等待都放到了用户按下触发后的关键路径。

#### 19.4 GraspNet 阶段 IK 判断有什么用

它的合理用途是一个 **早期可行性预筛**：在候选仍然较多时，先排除明显超出当前机器人模型/姿态约束的目标，减少后续 ROS planner 和真机接口压力。它不是执行许可，也不是“GraspNet 已经证明 CuRobo 一定能走”。

当前它的条件与最终 planner 不同：

| Item | GraspNet script-side prefilter | Resident CuRobo execution path |
| --- | --- | --- |
| Input pose | 脚本内刚由 camera -> RealMan -> driver_base 计算的 pose | ROS 消息重新解释的 pose；左臂当前有 frame label/value mismatch 风险 |
| Start state | 一次读取的 current joints | planner callback 重新读取，最多约 0.5 s 新鲜度约束 |
| Path | current -> pre-grasp，再 pre-grasp -> final | 默认 current -> goal 单段 `plan_single` |
| World | 脚本 placeholder world，未接 NVBlox | 可选 NVBlox/环境 world；unknown policy 会改变结果 |
| MotionGen | 脚本内创建/预热，`dt=.02` | resident 配置，`dt=.008` 等参数可能不同 |
| Output | `pregrasp_ik`/`grasp_ik` aggregate bool | 真实 `MotionGenStatus`，可能是 `IK_FAIL`、collision 或其他状态 |

因此出现“GraspNet 阶段 IK 通过、最后 CuRobo IK 失败”是完全可能的，当前左臂日志已经给出这种实例。要让预筛真正有判定意义，至少需要：

1. 统一消息 frame：发布 `left_driver_base` 数值就使用对应 frame 名，或在发布前只做一次 RealMan -> CuRobo 转换；
2. 预筛和 resident planner 使用同一套 `rm65.yml`、world、collision/NVBlox policy 和 pose target；
3. 统一使用 pre-grasp -> final 的分段规划，或明确最终 planner 只接收 pre-grasp 并在同一节点完成 final；
4. 逐候选记录 target frame、start joints、pre/final pose 和 `MotionGenStatus`，而不是只打印 `IK_FAIL`。

在这些条件统一前，GraspNet 阶段 IK 的价值仅限于“在它自己的假设下减少明显坏候选”，不能作为最终可执行性的证明。

### 20. 已实施修改

本轮按根因优先级完成了以下代码修改：

1. `supermarket_grasp_ros2/grasp_node.py`：脚本导出的 RealMan-base 姿态在 ROS 发布边界对左臂只转换一次为 CuRobo driver_base，并以 `left_base` 发布；候选 JSON 同时保留 source/target frame 和两套位置。
2. `curobo_realman_test/planner_node.py`：订阅 `/{arm}/grasp/pregrasp_pose`，与同时间戳的 final target 匹配时执行 `current -> pre-grasp -> final`；只发 final target 时保留 direct fallback。PLAN_ONLY 和执行模式都使用相同的分段结果。
3. `realsense_click_grasp.py`：脚本侧 CuRobo 默认 `interpolation_dt=0.008`、`max_attempts=4`、`time_dilation_factor=0.25`，与 resident planner 默认值一致。
4. `realsense_grasp_detection.py`/`realsense_click_grasp.py`：增加 raw count、score/width/depth/pose Top-20、输入点云内参、collision/NMS/top-k 阶段统计和逐候选 CuRobo JSON 记录。默认行为不变，输出旁生成 `*_diagnostics.json`。
5. 增加只读诊断开关：`--score-threshold`、`--disable-nms`、已有的 `--collision-thresh -1`，以及显式 `--skip-curobo`。这些开关不会自动放宽默认筛选。

ROS2 Python 文件已用 `/usr/bin/python3` 做语法检查；GraspNet 诊断统计用 `grasp` 环境进行了 mock candidate 测试，NMS ON/OFF 的候选数和 JSON 统计均通过。尚未启动真实 ROS/CuRobo/机械臂进程，因此 resident planner 的实际 `PLAN_OK_STAGED` 仍需在 `execute:=false` 下现场验证。

### 21. 最终终端摘要

```text
========== GraspNet Failure Diagnosis ==========

Raw candidates:
  NOT LOGGED (pred_decode 后没有 raw count/Top-K snapshot)
After NMS:
  88 aggregate candidates after collision+NMS across 50 runs
Collision free:
  88 logged post-combined results; 36/50 runs ended at zero (72.0% run-level)
IK valid:
  68 script-side final-path candidates; resident planner is a different condition
Pregrasp valid:
  68/69 script-side (1 reject, 1.45%)
CuRobo valid:
  68/69 script-side; resident left_base mismatch produced IK_FAIL
Final qualified:
  NOT IMPLEMENTED as a score threshold; 68 passed script prefilter before select-best cap

Largest rejection stage:
  Collision + NMS combined (36/50 runs zero; candidate denominator unknown)
Second largest rejection stage:
  Object-mask center filter (19/88 = 21.6%)
Third largest rejection stage:
  Script pre-grasp CuRobo (1/69 = 1.45%)

Raw GraspNet pose quality:
  QUESTIONABLE / NOT MEASURED (no raw Top-20 frame or pose snapshot)

TF correctness:
  RISK (left RealMan-base values published under left_base; TCP transforms unverified)

Point-cloud quality:
  RISK (4,921--22,976 target points observed; paired NaN/Inf/background stats absent)

Current final-score threshold:
  INVALID (no runtime final-score formula/threshold; THRESH_GOOD is training-only)

Main root cause:
  Interface and final-pose/frame-condition mismatch
Secondary root cause:
  Collision+NMS zero-output plus mask filtering; real gripper/TCP not verified

Conclusion:
  Mixed, primarily Interface + Robot constraints; GraspNet itself not proven as primary cause

Report:
  GRASPNET_SCORING_AUDIT.md
===============================================
```

本摘要中的 `NOT LOGGED/NOT MEASURED` 是当前诊断的边界。下一步应先固定一帧并补齐 raw candidate/TF/frame/status 日志，再做 NMS、collision、pre-grasp distance 和 position-only/full-pose IK 的离线消融；不应先降低阈值或关闭碰撞检查。
