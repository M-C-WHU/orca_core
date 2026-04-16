#!/usr/bin/env python3
"""安全的机械手校准脚本 - 可调节力度和速度"""

import argparse
import sys
import time
from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand

# ==================== 安全校准参数 ====================
# 这些参数可以全局调整，避免损坏机械手
SAFE_CALIBRATION_CURRENT = 150  # 单位：mA - 降低电流以减少力度
SAFE_WRIST_CALIBRATION_CURRENT = 200  # 单位：mA - 手腕校准电流
SAFE_CALIBRATION_STEP_SIZE = 0.05  # 单位：rad - 减小步长，更精细
SAFE_CALIBRATION_STEP_PERIOD = 0.001  # 单位：秒 - 增加步间间隔
SAFE_CALIBRATION_NUM_STABLE = 15  # 增加稳定检测次数
SAFE_CALIBRATION_THRESHOLD = 0.005  # 单位：rad - 更严格的稳定阈值

# 安全限制 - 防止过度转动
MAX_ROTATION_LIMIT = 6.0  # 单位：rad - 最大允许旋转角度
MIN_ROTATION_LIMIT = 0.0  # 单位：rad - 最小允许旋转角度

def safe_calibrate_hand(hand, force_wrist=False, use_safe_params=True):
    """执行安全的校准程序"""
    
    print("=== 安全校准模式 ===")
    print(f"使用安全参数: {use_safe_params}")
    
    if use_safe_params:
        print(f"校准电流: {SAFE_CALIBRATION_CURRENT} mA")
        print(f"手腕校准电流: {SAFE_WRIST_CALIBRATION_CURRENT} mA")
        print(f"步长: {SAFE_CALIBRATION_STEP_SIZE} rad")
        print(f"步间间隔: {SAFE_CALIBRATION_STEP_PERIOD} s")
        print(f"最大旋转限制: {MAX_ROTATION_LIMIT} rad")
    
    # 保存原始配置
    original_config = {
        'calibration_current': hand.config.calibration_current,
        'wrist_calibration_current': getattr(hand.config, 'wrist_calibration_current', 300),
        'calibration_step_size': hand.config.calibration_step_size,
        'calibration_step_period': hand.config.calibration_step_period,
        'calibration_num_stable': hand.config.calibration_num_stable,
        'calibration_threshold': hand.config.calibration_threshold,
    }
    
    try:
        # 应用安全参数
        if use_safe_params:
            hand.config.calibration_current = SAFE_CALIBRATION_CURRENT
            if hasattr(hand.config, 'wrist_calibration_current'):
                hand.config.wrist_calibration_current = SAFE_WRIST_CALIBRATION_CURRENT
            hand.config.calibration_step_size = SAFE_CALIBRATION_STEP_SIZE
            hand.config.calibration_step_period = SAFE_CALIBRATION_STEP_PERIOD
            hand.config.calibration_num_stable = SAFE_CALIBRATION_NUM_STABLE
            hand.config.calibration_threshold = SAFE_CALIBRATION_THRESHOLD
        
        # 执行校准
        print("开始安全校准...")
        hand.calibrate(force_wrist=force_wrist)
        print("安全校准完成")
        
        # 验证校准结果
        if hasattr(hand, 'calibration') and hand.calibration.calibrated:
            print("\n=== 校准验证 ===")
            calib = hand.calibration
            for motor_id, limits in calib.motor_limits_dict.items():
                if limits[0] is not None and limits[1] is not None:
                    # 检查是否在安全范围内
                    if limits[0] < MIN_ROTATION_LIMIT or limits[1] > MAX_ROTATION_LIMIT:
                        print(f"警告: 电机 {motor_id} 的极限 [{limits[0]:.3f}, {limits[1]:.3f}] 超出安全范围")
                    else:
                        print(f"电机 {motor_id}: 安全范围 [{limits[0]:.3f}, {limits[1]:.3f}] rad")
                else:
                    print(f"电机 {motor_id}: 未完全校准")
        
        return True
        
    except Exception as e:
        print(f"安全校准过程中出错: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 恢复原始配置
        if use_safe_params:
            hand.config.calibration_current = original_config['calibration_current']
            if hasattr(hand.config, 'wrist_calibration_current'):
                hand.config.wrist_calibration_current = original_config['wrist_calibration_current']
            hand.config.calibration_step_size = original_config['calibration_step_size']
            hand.config.calibration_step_period = original_config['calibration_step_period']
            hand.config.calibration_num_stable = original_config['calibration_num_stable']
            hand.config.calibration_threshold = original_config['calibration_threshold']

def manual_safety_check(hand):
    """手动安全检查 - 在开始校准前确认"""
    print("\n=== 手动安全检查 ===")
    
    # 检查当前电机位置
    try:
        motor_pos = hand.get_motor_pos()
        print("当前电机位置:")
        for i, pos in enumerate(motor_pos, 1):
            print(f"  电机 {i}: {pos:.3f} rad")
            
            # 检查是否在合理范围内
            if pos < MIN_ROTATION_LIMIT or pos > MAX_ROTATION_LIMIT:
                print(f"  警告: 电机 {i} 位置 {pos:.3f} rad 超出预期范围!")
                
    except Exception as e:
        print(f"读取位置失败: {e}")
    
    # 检查电机电流
    try:
        motor_current = hand.get_motor_current()
        print("\n当前电机电流:")
        for i, cur in enumerate(motor_current, 1):
            print(f"  电机 {i}: {cur:.1f} mA")
    except Exception as e:
        print(f"读取电流失败: {e}")
    
    print("\n=== 安全提示 ===")
    print("1. 确保机械手没有物理障碍")
    print("2. 确保所有关节可以自由移动")
    print("3. 准备好随时按 Ctrl+C 停止")
    print("4. 校准过程中密切观察机械手状态")
    
    response = input("\n是否继续校准? (yes/no): ")
    return response.lower() in ['yes', 'y', '是']

def main() -> int:
    parser = argparse.ArgumentParser(
        description="运行安全的ORCA机械手校准程序"
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="即使手腕已校准也重新校准",
    )
    parser.add_argument(
        "--unsafe",
        action="store_true",
        help="使用原始配置参数（不推荐）",
    )
    parser.add_argument(
        "--current",
        type=int,
        default=SAFE_CALIBRATION_CURRENT,
        help=f"校准电流 (mA)，默认: {SAFE_CALIBRATION_CURRENT}",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=SAFE_CALIBRATION_STEP_SIZE,
        help=f"校准步长 (rad)，默认: {SAFE_CALIBRATION_STEP_SIZE}",
    )
    parser.add_argument(
        "--no-check",
        action="store_true",
        help="跳过手动安全检查",
    )
    
    args = parser.parse_args()
    
    # 更新全局参数
    global SAFE_CALIBRATION_CURRENT, SAFE_CALIBRATION_STEP_SIZE
    SAFE_CALIBRATION_CURRENT = args.current
    SAFE_CALIBRATION_STEP_SIZE = args.step_size
    
    hand = create_hand(args.config_path, use_mock=args.mock)
    
    try:
        connect_hand(hand)
        
        # 手动安全检查
        if not args.no_check and not manual_safety_check(hand):
            print("校准已取消")
            return 1
        
        # 执行安全校准
        success = safe_calibrate_hand(
            hand, 
            force_wrist=args.force_wrist,
            use_safe_params=not args.unsafe
        )
        
        if success:
            print("\n✅ 安全校准成功完成")
            return 0
        else:
            print("\n❌ 安全校准失败")
            return 1
            
    except KeyboardInterrupt:
        print("\n⚠️  校准被用户中断")
        return 1
    except Exception as e:
        print(f"\n❌ 校准过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    # 打印安全参数
    print("安全校准参数:")
    print(f"  校准电流: {SAFE_CALIBRATION_CURRENT} mA")
    print(f"  手腕校准电流: {SAFE_WRIST_CALIBRATION_CURRENT} mA")
    print(f"  步长: {SAFE_CALIBRATION_STEP_SIZE} rad")
    print(f"  步间间隔: {SAFE_CALIBRATION_STEP_PERIOD} s")
    print(f"  最大旋转限制: {MAX_ROTATION_LIMIT} rad")
    print()
    
    raise SystemExit(main())