#!/usr/bin/env python3
"""检查校准状态和机械手位置"""

import sys
import numpy as np
from orca_core.hardware_hand import OrcaHand

def main():
    config_path = "orca_core/models/v2/orcahand_right/config.yaml"
    
    print("=== 检查机械手校准状态 ===")
    
    try:
        # 创建机械手实例
        hand = OrcaHand(config_path)
        
        # 连接
        print("连接机械手...")
        hand.connect()
        
        # 检查校准数据
        print("\n=== 校准数据 ===")
        if hasattr(hand, '_calibration_data'):
            calib = hand._calibration_data
            print(f"校准数据存在: {len(calib)} 个关节")
            for joint, data in calib.items():
                print(f"  {joint}: {data}")
        else:
            print("警告: 没有找到校准数据属性")
        
        # 检查电机限制
        print("\n=== 电机限制 ===")
        if hasattr(hand, '_motor_limits'):
            limits = hand._motor_limits
            for motor_id, (lower, upper) in enumerate(limits, 1):
                if lower is not None and upper is not None:
                    print(f"  电机 {motor_id}: [{lower:.3f}, {upper:.3f}] rad")
                else:
                    print(f"  电机 {motor_id}: 未校准")
        
        # 读取当前位置
        print("\n=== 当前位置 ===")
        try:
            motor_pos = hand.get_motor_pos()
            joint_pos = hand.get_joint_pos()
            
            print("电机位置 (rad):")
            for i, pos in enumerate(motor_pos, 1):
                print(f"  电机 {i}: {pos:.3f}")
            
            print("\n关节位置 (rad):")
            for joint_name, pos in joint_pos.items():
                print(f"  {joint_name}: {pos:.3f}")
                
        except Exception as e:
            print(f"读取位置时出错: {e}")
        
        # 断开连接
        hand.disconnect()
        print("\n=== 检查完成 ===")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())