#!/usr/bin/env python3
"""简单的机械手测试脚本"""

import sys
import time
from common import create_hand, connect_hand, shutdown_hand

def main():
    config_path = "orca_core/models/v2/orcahand_right/config.yaml"
    
    print("=== 简单机械手测试 ===")
    
    try:
        # 创建机械手实例
        hand = create_hand(config_path, use_mock=False)
        
        # 连接
        print("连接机械手...")
        connect_hand(hand)
        
        # 测试1: 读取电机位置
        print("\n=== 测试1: 读取电机位置 ===")
        try:
            motor_pos = hand.get_motor_pos()
            print(f"读取到 {len(motor_pos)} 个电机位置:")
            for i, pos in enumerate(motor_pos, 1):
                print(f"  电机 {i}: {pos:.3f} rad")
        except Exception as e:
            print(f"读取电机位置失败: {e}")
        
        # 测试2: 读取电机电流
        print("\n=== 测试2: 读取电机电流 ===")
        try:
            motor_current = hand.get_motor_current()
            print(f"读取到 {len(motor_current)} 个电机电流:")
            for i, cur in enumerate(motor_current, 1):
                print(f"  电机 {i}: {cur:.1f} mA")
        except Exception as e:
            print(f"读取电机电流失败: {e}")
        
        # 测试3: 检查校准状态
        print("\n=== 测试3: 检查校准状态 ===")
        try:
            # 尝试访问校准相关属性
            if hasattr(hand, 'calibration'):
                print(f"校准对象: {hand.calibration}")
            else:
                print("警告: 没有找到 calibration 属性")
                
            # 检查是否有 _motor_limits 属性
            if hasattr(hand, '_motor_limits'):
                limits = hand._motor_limits
                print(f"电机限制: {limits}")
            else:
                print("警告: 没有找到 _motor_limits 属性")
                
        except Exception as e:
            print(f"检查校准状态失败: {e}")
        
        # 测试4: 尝试小幅度移动一个电机
        print("\n=== 测试4: 尝试小幅度移动 ===")
        try:
            # 获取当前位置
            current_pos = hand.get_motor_pos()
            print(f"电机1当前位置: {current_pos[0]:.3f} rad")
            
            # 尝试小幅度移动（如果当前位置不是极限）
            if abs(current_pos[0]) < 5.0:  # 不在极限附近
                print("尝试小幅度移动电机1...")
                # 这里需要根据实际情况调整
                # hand._set_motor_pos([1], [current_pos[0] + 0.1])
                print("移动测试跳过（需要具体实现）")
            else:
                print("电机1在极限位置，跳过移动测试")
                
        except Exception as e:
            print(f"移动测试失败: {e}")
        
        # 断开连接
        shutdown_hand(hand)
        print("\n=== 测试完成 ===")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())