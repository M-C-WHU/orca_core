# 安全校准指南

## 问题分析

原始校准脚本 (`calibrate.py`) 使用以下参数，导致机械手损坏：
- `calibration_current: 300` mA - 电流过大
- `calibration_step_size: 0.15` rad - 步长过大
- 没有安全限制，导致舵机转过了机械极限

## 解决方案

### 1. 安全校准脚本 (`safe_calibrate.py`)

```bash
# 基本使用
sudo uv run python scripts/safe_calibrate.py orca_core/models/v2/orcahand_right/config.yaml

# 自定义参数
sudo uv run python scripts/safe_calibrate.py \
  --current 100 \          # 校准电流 (mA)
  --step-size 0.03 \       # 步长 (rad)
  --force-wrist \          # 强制重新校准手腕
  orca_core/models/v2/orcahand_right/config.yaml

# 跳过安全检查（不推荐）
sudo uv run python scripts/safe_calibrate.py \
  --no-check \
  orca_core/models/v2/orcahand_right/config.yaml
```

### 2. 安全配置文件 (`config_safe.yaml`)

永久降低校准力度：
```bash
# 使用安全配置文件
sudo uv run python scripts/calibrate.py orca_core/models/v2/orcahand_right/config_safe.yaml
```

安全参数对比：
| 参数 | 原始值 | 安全值 | 变化 |
|------|--------|--------|------|
| `calibration_current` | 300 mA | 150 mA | -50% |
| `calibration_step_size` | 0.15 rad | 0.05 rad | -67% |
| `calibration_num_stable` | 10 | 15 | +50% |
| `calibration_threshold` | 0.01 | 0.005 | -50% |

### 3. 全局安全参数

在 `safe_calibrate.py` 中定义的全局变量，方便调整：

```python
# 安全校准参数
SAFE_CALIBRATION_CURRENT = 150          # 单位：mA
SAFE_WRIST_CALIBRATION_CURRENT = 200    # 单位：mA
SAFE_CALIBRATION_STEP_SIZE = 0.05       # 单位：rad
SAFE_CALIBRATION_STEP_PERIOD = 0.001    # 单位：秒
SAFE_CALIBRATION_NUM_STABLE = 15        # 稳定检测次数
SAFE_CALIBRATION_THRESHOLD = 0.005      # 单位：rad

# 安全限制
MAX_ROTATION_LIMIT = 6.0                # 单位：rad (约344°)
MIN_ROTATION_LIMIT = 0.0                # 单位：rad
```

## 诊断损坏

如果怀疑机械手已损坏：

```bash
# 运行诊断脚本
sudo uv run python scripts/diagnose_damage.py
```

诊断脚本会检查：
1. 电机通信是否正常
2. 电机电流是否异常
3. 校准数据是否合理
4. 是否可以小幅度移动

## 校准最佳实践

### 校准前：
1. **物理检查**：确保机械手没有物理障碍
2. **电源检查**：确保电源稳定
3. **连接检查**：确保USB连接可靠
4. **手动检查**：手动移动关节，确保没有卡住

### 校准中：
1. **密切观察**：全程观察机械手状态
2. **随时中断**：准备好按 Ctrl+C
3. **逐步增加**：先使用低电流，逐步增加

### 校准后：
1. **验证结果**：检查校准数据是否合理
2. **测试移动**：小幅度测试各个关节
3. **保存配置**：备份校准数据

## 故障排除

### 问题1：校准过程中舵机不移动
- 检查电流设置是否过低
- 检查扭矩是否启用
- 检查物理连接

### 问题2：校准过早停止
- 增加 `calibration_num_stable`
- 降低 `calibration_threshold`
- 检查是否有物理阻力

### 问题3：校准范围异常
- 检查 `MAX_ROTATION_LIMIT` 设置
- 检查机械极限是否被越过
- 考虑手动设置限制

### 问题4：通信超时
- 降低 `calibration_step_size`
- 增加 `calibration_step_period`
- 检查USB连接质量

## 高级调整

### 针对不同关节调整参数
某些关节可能需要不同的参数：

```python
# 在 safe_calibrate.py 中添加关节特定参数
JOINT_SPECIFIC_PARAMS = {
    'wrist': {'current': 200, 'step_size': 0.03},
    'thumb_cmc': {'current': 120, 'step_size': 0.04},
    'index_pip': {'current': 100, 'step_size': 0.02},
}
```

### 自适应校准
根据实时反馈调整参数：
- 监测电流变化，如果突然增加则停止
- 监测位置变化率，如果异常则调整步长
- 实现软停止，而不是硬停止

## 紧急停止

如果校准过程中出现问题：
1. **立即按 Ctrl+C**
2. **断开电源**
3. **运行诊断脚本**
4. **手动检查机械结构**

## 总结

安全校准的关键是：
1. **低电流**：从150mA开始，逐步增加
2. **小步长**：使用0.05 rad或更小的步长
3. **多检查**：增加稳定检测次数
4. **有限制**：设置最大旋转角度限制
5. **可监控**：全程观察，随时可中断

记住：**宁可校准不完整，也不要损坏机械手**。可以多次校准，逐步找到最佳参数。