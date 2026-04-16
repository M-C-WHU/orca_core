#!/usr/bin/env python3
"""诊断机械手损坏情况"""

import sys
import time
from common import create_hand, connect_hand, shutdown_hand

def main():
    config_path = "orca_core/models/v2/orcahand_right/config.yaml"
    
    print("=== 机械手损坏诊断 ===")
    print("注意: 此脚本仅用于诊断，不会移动机械手")
    
    try:
        # 创建机械手实例
        hand = create_hand(config_path, use_mock=False)
        
        # 连接
        print("连接机械手...")
        try:
            connect_hand(hand)
        except Exception as e:
            print(f"连接失败: {e}")
            print("可能的原因:")
            print("1. 机械手电源未打开")
            print("2. USB连接断开")
            print("3. 舵机损坏")
            return 1
        
        # 诊断1: 检查电机通信
        print("\n=== 诊断1: 电机通信检查 ===")
        try:
            motor_pos = hand.get_motor_pos()
            print(f"成功读取 {len(motor_pos)} 个电机位置")
            
            # 检查是否有异常位置
            abnormal_motors = []
            for i, pos in enumerate(motor_pos, 1):
                if pos == 0.0 or abs(pos) > 10.0:  # 异常值
                    abnormal_motors.append((i, pos))
            
            if abnormal_motors:
                print("警告: 以下电机位置异常:")
                for motor_id, pos in abnormal_motors:
                    print(f"  电机 {motor_id}: {pos:.3f} rad")
            else:
                print("所有电机位置正常")
                
        except Exception as e:
            print(f"读取电机位置失败: {e}")
            print("可能舵机通信已损坏")
        
        # 诊断2: 检查电机电流
        print("\n=== 诊断2: 电机电流检查 ===")
        try:
            motor_current = hand.get_motor_current()
            print(f"成功读取 {len(motor_current)} 个电机电流")
            
            # 检查是否有异常电流
            high_current_motors = []
            for i, cur in enumerate(motor_current, 1):
                if abs(cur) > 50.0:  # 电流过高
                    high_current_motors.append((i, cur))
            
            if high_current_motors:
                print("警告: 以下电机电流异常高:")
                for motor_id, cur in high_current_motors:
                    print(f"  电机 {motor_id}: {cur:.1f} mA")
                print("可能舵机卡住或损坏")
            else:
                print("所有电机电流正常")
                
        except Exception as e:
            print(f"读取电机电流失败: {e}")
        
        # 诊断3: 检查校准数据
        print("\n=== 诊断3: 校准数据检查 ===")
        try:
            if hasattr(hand, 'calibration'):
                calib = hand.calibration
                print(f"校准状态: {calib.calibrated}")
                
                if calib.calibrated:
                    print("校准数据:")
                    for motor_id, limits in calib.motor_limits_dict.items():
                        if limits[0] is not None and limits[1] is not None:
                            range_size = limits[1] - limits[0]
                            print(f"  电机 {motor_id}: [{limits[0]:.3f}, {limits[1]:.3f}] rad (范围: {range_size:.3f} rad)")
                            
                            # 检查范围是否合理
                            if range_size > 6.5:  # 超过360度
                                print(f"    警告: 范围过大，可能已转过极限")
                            elif range_size < 0.1:  # 范围太小
                                print(f"    警告: 范围过小，可能未正确校准")
                else:
                    print("机械手未校准")
            else:
                print("未找到校准数据")
                
        except Exception as e:
            print(f"检查校准数据失败: {e}")
        
        # 诊断4: 尝试小幅度移动（可选）
        print("\n=== 诊断4: 小幅度移动测试 ===")
        response = input("是否尝试小幅度移动测试? (yes/no): ")
        
        if response.lower() in ['yes', 'y', '是']:
            try:
                # 获取当前位置
                current_pos = hand.get_motor_pos()
                print(f"电机1当前位置: {current_pos[0]:.3f} rad")
                
                # 尝试极小幅度移动
                if abs(current_pos[0]) < 5.0:
                    print("尝试极小幅度移动电机1 (+0.01 rad)...")
                    # 启用扭矩
                    hand.enable_torque(motor_ids=[1])
                    time.sleep(0.1)
                    
                    # 设置新位置
                    new_pos = {1: current_pos[0] + 0.01}
                    hand._set_motor_pos(new_pos)
                    time.sleep(0.5)
                    
                    # 检查新位置
                    new_pos_read = hand.get_motor_pos()
                    print(f"移动后位置: {new_pos_read[0]:.3f} rad")
                    
                    if abs(new_pos_read[0] - (current_pos[0] + 0.01)) < 0.02:
                        print("移动测试成功")
                    else:
                        print("移动测试失败 - 位置未按预期变化")
                    
                    # 禁用扭矩
                    hand.disable_torque()
                else:
                    print("电机1在极限位置，跳过移动测试")
                    
            except Exception as e:
                print(f"移动测试失败: {e}")
                print("可能舵机已损坏或卡住")
        else:
            print("跳过移动测试")
        
        # 断开连接
        shutdown_hand(hand)
        
        print("\n=== 诊断总结 ===")
        print("建议:")
        print("1. 如果电机通信正常但位置异常，可能需要手动复位")
        print("2. 如果电流异常高，检查是否有物理卡住")
        print("3. 如果校准范围过大，说明已转过机械极限")
        print("4. 下次校准使用 safe_calibrate.py 脚本")
        
    except Exception as e:
        print(f"诊断过程中出错: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())