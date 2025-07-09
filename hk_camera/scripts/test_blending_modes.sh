#!/bin/bash

# 双相机拼接融合模式测试脚本
# 用法: ./test_blending_modes.sh [mode]
# 模式: none, linear, weighted, feather

NODE_NAME="/hk_camera_stitching_node"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== 双相机拼接融合模式测试工具 ===${NC}"

# 检查节点是否运行
if ! ros2 node list | grep -q "hk_camera_stitching_node"; then
    echo -e "${RED}错误: hk_camera_stitching_node 节点未运行${NC}"
    echo "请先启动节点: ros2 launch hk_camera dual_camera_stitching.launch.py"
    exit 1
fi

# 显示当前参数状态
show_current_status() {
    echo -e "${YELLOW}当前融合参数状态:${NC}"
    echo -n "融合启用: "; ros2 param get $NODE_NAME enable_blending | grep -o "True\|False"
    echo -n "融合模式: "; ros2 param get $NODE_NAME blend_mode | grep -o '".*"' | tr -d '"'
    echo -n "融合强度: "; ros2 param get $NODE_NAME blend_strength | grep -o "[0-9.]*"
    echo -n "羽化大小: "; ros2 param get $NODE_NAME blend_feather_size | grep -o "[0-9]*"
    echo -n "重叠优先级: "; ros2 param get $NODE_NAME blend_overlap_priority | grep -o '".*"' | tr -d '"'
    echo -n "伽马校正: "; ros2 param get $NODE_NAME blend_gamma_correction | grep -o "[0-9.]*"
    echo ""
}

# 设置无融合模式
set_none_mode() {
    echo -e "${GREEN}设置无融合模式 (最快速度)${NC}"
    ros2 param set $NODE_NAME enable_blending false
    ros2 param set $NODE_NAME blend_mode "none"
    echo "✓ 无融合模式已设置"
}

# 设置线性融合模式
set_linear_mode() {
    echo -e "${GREEN}设置线性融合模式 (平衡速度)${NC}"
    ros2 param set $NODE_NAME enable_blending true
    ros2 param set $NODE_NAME blend_mode "linear"
    ros2 param set $NODE_NAME blend_strength 0.5
    ros2 param set $NODE_NAME blend_gamma_correction 1.0
    echo "✓ 线性融合模式已设置"
}

# 设置加权融合模式
set_weighted_mode() {
    echo -e "${GREEN}设置加权融合模式 (高质量)${NC}"
    ros2 param set $NODE_NAME enable_blending true
    ros2 param set $NODE_NAME blend_mode "weighted"
    ros2 param set $NODE_NAME blend_strength 0.6
    ros2 param set $NODE_NAME blend_overlap_priority "center"
    ros2 param set $NODE_NAME blend_gamma_correction 0.9
    echo "✓ 加权融合模式已设置"
}

# 设置羽化融合模式
set_feather_mode() {
    echo -e "${GREEN}设置羽化融合模式 (最高质量)${NC}"
    ros2 param set $NODE_NAME enable_blending true
    ros2 param set $NODE_NAME blend_mode "feather"
    ros2 param set $NODE_NAME blend_strength 0.7
    ros2 param set $NODE_NAME blend_feather_size 60
    ros2 param set $NODE_NAME blend_overlap_priority "center"
    ros2 param set $NODE_NAME blend_gamma_correction 0.85
    echo "✓ 羽化融合模式已设置"
}

# 交互式菜单
interactive_menu() {
    while true; do
        echo ""
        echo -e "${BLUE}请选择融合模式:${NC}"
        echo "1) 无融合 (none) - 最快速度，可能有缝隙"
        echo "2) 线性融合 (linear) - 平衡速度和质量"  
        echo "3) 加权融合 (weighted) - 高质量，基于距离"
        echo "4) 羽化融合 (feather) - 最高质量，最自然过渡"
        echo "5) 显示当前状态"
        echo "6) 自定义参数调整"
        echo "0) 退出"
        echo ""
        read -p "请输入选择 (0-6): " choice
        
        case $choice in
            1) set_none_mode ;;
            2) set_linear_mode ;;
            3) set_weighted_mode ;;
            4) set_feather_mode ;;
            5) show_current_status ;;
            6) custom_adjustment ;;
            0) echo "退出"; exit 0 ;;
            *) echo -e "${RED}无效选择，请重试${NC}" ;;
        esac
        
        echo ""
        show_current_status
    done
}

# 自定义参数调整
custom_adjustment() {
    echo -e "${YELLOW}自定义参数调整:${NC}"
    echo "1) 调整融合强度 (0.0-1.0)"
    echo "2) 调整羽化大小 (像素)"
    echo "3) 设置重叠优先级 (left/right/center)"
    echo "4) 调整伽马校正 (0.5-2.0)"
    echo "5) 启用/禁用融合"
    read -p "请选择要调整的参数 (1-5): " param_choice
    
    case $param_choice in
        1)
            read -p "输入融合强度 (0.0-1.0): " strength
            ros2 param set $NODE_NAME blend_strength $strength
            ;;
        2)
            read -p "输入羽化大小 (像素): " feather
            ros2 param set $NODE_NAME blend_feather_size $feather
            ;;
        3)
            read -p "输入重叠优先级 (left/right/center): " priority
            ros2 param set $NODE_NAME blend_overlap_priority $priority
            ;;
        4)
            read -p "输入伽马校正值 (0.5-2.0): " gamma
            ros2 param set $NODE_NAME blend_gamma_correction $gamma
            ;;
        5)
            read -p "启用融合? (true/false): " enable
            ros2 param set $NODE_NAME enable_blending $enable
            ;;
        *)
            echo -e "${RED}无效选择${NC}"
            ;;
    esac
}

# 主程序逻辑
if [ "$#" -eq 0 ]; then
    # 无参数，显示交互式菜单
    show_current_status
    interactive_menu
else
    # 有参数，直接设置模式
    case $1 in
        "none")
            set_none_mode
            ;;
        "linear")
            set_linear_mode
            ;;
        "weighted")
            set_weighted_mode
            ;;
        "feather")
            set_feather_mode
            ;;
        "status")
            show_current_status
            ;;
        *)
            echo -e "${RED}无效的模式参数${NC}"
            echo "用法: $0 [none|linear|weighted|feather|status]"
            echo "或者不带参数运行交互式菜单"
            exit 1
            ;;
    esac
    
    echo ""
    show_current_status
fi

echo -e "${BLUE}测试完成！您可以通过查看 /stitched_image topic 来观察不同融合模式的效果${NC}" 