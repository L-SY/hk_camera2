# 双相机拼接融合参数指南

## 概述

本文档详细介绍了双相机拼接系统中的高级融合算法及其参数配置。新的融合系统提供了多种融合模式和可调参数，可以根据实际应用场景优化拼接效果。

## 主要融合参数

### 基础控制参数

#### `enable_blending` (bool)
- **默认值**: `true`
- **描述**: 是否启用重叠区域融合
- **用法**: 
  - `true`: 启用融合算法处理重叠区域
  - `false`: 简单覆盖模式，右图直接覆盖左图
- **适用场景**: 如果发现融合后图像模糊，可以设置为`false`尝试不融合的效果

#### `blend_strength` (double, 0.0-1.0)
- **默认值**: `1.0`
- **描述**: 融合强度控制
- **用法**:
  - `0.0`: 完全使用左图
  - `0.5`: 左右图各占50%
  - `1.0`: 根据具体融合模式计算权重
- **建议值**: 0.3-0.8，根据实际效果调整

### 融合模式选择

#### `blend_mode` (string)
- **默认值**: `"linear"`
- **描述**: 融合算法模式
- **可选值**:

##### 1. `"none"` - 无融合模式
- **效果**: 右图直接覆盖左图，无过渡
- **优点**: 处理速度最快，无模糊
- **缺点**: 可能有明显的拼接缝隙
- **适用**: 两相机曝光一致，重叠区域较小

##### 2. `"linear"` - 线性融合模式
- **效果**: 在重叠区域进行简单的线性插值
- **优点**: 算法简单，处理速度快
- **缺点**: 可能造成重叠区域轻微模糊
- **适用**: 一般场景，对速度有要求

##### 3. `"weighted"` - 距离加权融合
- **效果**: 基于像素到边界的距离进行加权融合
- **优点**: 过渡更自然，减少缝隙
- **缺点**: 计算复杂度较高
- **适用**: 对质量要求高的场景

##### 4. `"feather"` - 羽化融合模式
- **效果**: 在边缘创建羽化过渡效果
- **优点**: 最自然的过渡效果
- **缺点**: 处理时间最长
- **适用**: 对拼接质量要求极高的场景

### 高级控制参数

#### `blend_feather_size` (int, 像素)
- **默认值**: `50`
- **描述**: 羽化边缘大小（仅在feather模式下有效）
- **用法**:
  - `0`: 无羽化效果
  - `50`: 50像素的羽化边缘
  - `100`: 100像素的羽化边缘
- **建议值**: 根据重叠区域大小调整，通常为重叠宽度的10-30%

#### `blend_overlap_priority` (string)
- **默认值**: `"center"`
- **描述**: 重叠区域优先级处理
- **可选值**:
  - `"left"`: 左图优先，在重叠区域主要保留左图
  - `"right"`: 右图优先，在重叠区域主要保留右图
  - `"center"`: 中心平衡，正常融合处理
- **适用**: 当一个相机效果明显更好时使用

#### `blend_gamma_correction` (double, 0.5-2.0)
- **默认值**: `1.0`
- **描述**: 伽马校正系数
- **用法**:
  - `< 1.0`: 增强暗部细节
  - `= 1.0`: 不进行伽马校正
  - `> 1.0`: 压缩亮部，增强对比度
- **建议值**: 0.8-1.2

## 参数配置示例

### 高质量配置（推荐）
```yaml
enable_blending: true
blend_mode: "weighted"
blend_strength: 0.6
blend_feather_size: 30
blend_overlap_priority: "center"
blend_gamma_correction: 0.9
```

### 高速度配置
```yaml
enable_blending: true
blend_mode: "linear"
blend_strength: 0.5
blend_feather_size: 20
blend_overlap_priority: "center"
blend_gamma_correction: 1.0
```

### 无融合配置（最快速度）
```yaml
enable_blending: false
blend_mode: "none"
blend_strength: 1.0
blend_feather_size: 0
blend_overlap_priority: "center"
blend_gamma_correction: 1.0
```

### 羽化配置（最高质量）
```yaml
enable_blending: true
blend_mode: "feather"
blend_strength: 0.7
blend_feather_size: 60
blend_overlap_priority: "center"
blend_gamma_correction: 0.85
```

## 实时参数调整

所有融合参数都支持实时动态调整，无需重启节点：

```bash
# 切换到无融合模式
ros2 param set /hk_camera_stitching_node enable_blending false

# 调整融合强度
ros2 param set /hk_camera_stitching_node blend_strength 0.3

# 切换融合模式
ros2 param set /hk_camera_stitching_node blend_mode "weighted"

# 调整羽化大小
ros2 param set /hk_camera_stitching_node blend_feather_size 80

# 设置重叠优先级
ros2 param set /hk_camera_stitching_node blend_overlap_priority "left"

# 调整伽马校正
ros2 param set /hk_camera_stitching_node blend_gamma_correction 0.8
```

## 性能影响

不同融合模式的相对性能（处理时间）：

1. `"none"`: 100% (基准)
2. `"linear"`: ~120%
3. `"weighted"`: ~180%
4. `"feather"`: ~250%

## 故障排除

### 问题1: 重叠区域模糊
**解决方案**:
- 设置 `enable_blending: false` 尝试无融合模式
- 或降低 `blend_strength` 到 0.2-0.4
- 或切换到 `blend_mode: "none"`

### 问题2: 拼接缝隙明显
**解决方案**:
- 启用融合: `enable_blending: true`
- 使用 `blend_mode: "weighted"` 或 `"feather"`
- 增加 `blend_feather_size` 到 50-100

### 问题3: 处理速度慢
**解决方案**:
- 使用 `blend_mode: "linear"` 或 `"none"`
- 减小 `blend_feather_size`
- 考虑禁用融合: `enable_blending: false`

### 问题4: 颜色不匹配
**解决方案**:
- 调整 `blend_gamma_correction`
- 使用 `blend_overlap_priority` 优先保留效果更好的相机
- 检查相机的白平衡和曝光设置

## 最佳实践

1. **初始配置**: 从线性模式开始，逐步尝试更高级的模式
2. **参数调整**: 小步长调整参数，观察实时效果
3. **性能平衡**: 根据硬件性能和质量需求选择合适的融合模式
4. **场景适配**: 不同光照条件可能需要不同的融合参数
5. **实时监控**: 观察处理时间，确保满足实时性要求

通过合理配置这些参数，可以在处理速度和拼接质量之间找到最佳平衡点。 