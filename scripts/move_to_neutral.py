#!/usr/bin/env python3
"""将机械手移动到中性位置"""

import sys
import time
import numpy as np
from common import create_hand, connect_hand, shutdown_hand

def main():
    config_path = "orca_core/models/v2/orcahand_right/config.yaml"
    
    print("=== 将机械手移动到中性位置 ===")
    
    try:
        # 创建机械手实例
        hand = create_hand(config_path, use_mock=False)
        
        # 连接
        print("连接机械手...")
        connect_hand(hand)
        
        # 获取校准数据
        if not hasattr(hand, 'calibration'):
            print("错误: 没有找到校准数据")
            return 1
            
        calib = hand.calibration
        print(f"校准状态: {calib.calibrated}")
        
        # 计算中性位置（每个电机的中间位置）
        neutral_positions = []
        motor_ids = list(range(1, 18))  # 电机1-17
        
        for motor_id in motor_ids:
            limits = calib.motor_limits_dict.get(motor_id)
            if limits and limits[0] is not None and limits[1] is not None:
                # 计算中间位置
                neutral = (limits[0] + limits[1]) / 2.0
                neutral_positions.append(neutral)
                print(f"电机 {motor_id}: 限制 [{limits[0]:.3f}, {limits[1]:.3f}] -> 中性 {neutral:.3f} rad")
            else:
                print(f"电机 {motor_id}: 未校准，使用默认位置 0.0")
                neutral_positions.append(0.0)
        
        # 获取当前位置
        current_pos = hand.get_motor_pos()
        print(f"\n当前位置:")
        for i, pos in enumerate(current_pos, 1):
            print(f"  电机 {i}: {pos:.3f} rad")
        
        # 设置控制模式
        print("\n设置控制模式...")
        hand.set_control_mode("position")  # 位置控制模式
        
        # 启用扭矩
        print("启用扭矩...")
        hand.enable_torque()
        
        # 缓慢移动到中性位置
        print("\n缓慢移动到中性位置...")
        steps = 50  # 分50步移动
        for step in range(steps + 1):
            # 计算插值位置
            alpha = step / steps
            target_pos = current_pos * (1 - alpha) + np.array(neutral_positions) * alpha
            
            # 创建位置字典
            target_dict = {}
            for i, motor_id in enumerate(motor_ids):
                target_dict[motor_id] = target_pos[i]
            
            # 设置位置
            hand._set_motor_pos(target_dict)
            
            # 等待
            time.sleep(0.02)
            
            if step % 10 == 0:
                print(f"  进度: {step/steps*100:.0f}%")
        
        print("移动到中性位置完成")
        
        # 保持位置几秒钟
        print("保持位置5秒...")
        time.sleep(5)
        
        # 禁用扭矩
        print("禁用扭矩...")
        hand.disable_torque()
        
        # 断开连接
        shutdown_hand(hand)
        print("\n=== 完成 ===")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        
        # 尝试安全关闭
        try:
            hand.disable_torque()
            shutdown_hand(hand)
        except:
            pass
            
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())