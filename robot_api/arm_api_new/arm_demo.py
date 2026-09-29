from realman_arm_api_api2 import RealmanArmClient, ArmApiError, ArmPose
import time


def main() -> None:
    try:
        with RealmanArmClient(model="RM65", ip="169.254.128.19") as arm_l:
            print("API version:", arm_l.api_version())
            print("Socket state:", arm_l.socket_state())

            arm_l.power_on(block=False)
            print("Power:", arm_l.get_power_state())

            state = arm_l.get_state()
            print("Current state:", state)
            print("Current joints:", state.joints)
            print("Current pose:", state.pose.as_list())

            # # Joint move
            arm_l.movej([0, 0, 0, 0, 0, 180], v=20, r=0, trajectory_connect=0, block=True)

            # # Linear move
            # target_pose = ArmPose(0.0, -0.867, 0.0, -1.57, 0.0, 3.14)   
            
            #arm_l.movel(target_pose, v=10, trajectory_connect=0, r=0, block=True)
            
            # # move_home
            #arm_l.move_home( block=True)
            
            # # movej_p
            #arm_l.movej_p(target_pose, v=20, r=0, block=True)
             
            # # lift_control'

            #arm_l.control_lift(action="to", height=500, speed=20, block=True)

            # arm_l.lift_control(action="up", height=0.1, speed=100, block=True, timeout=3)
            # time.sleep(1)   
            # arm_l.lift_control(action="down", height=0.1, speed=100, block=True, timeout=3)
            # time.sleep(1)

            # #大寰夹爪
            
            # arm_l.control_gripper_dh(action, *, speed=50, force=50, position=0, port=1, 
            #         device=1, baudrate=115200, block=True, auto_prepare=True, tool_voltage_type=3)

            # Gripper因时夹爪
            # arm_l.gripper_release(speed=500, block=True, timeout=3)
            # arm_l.gripper_pick(speed=500, force=200, block=True, timeout=3)
            # arm_l.set_tool_voltage(3)

            # 2) 配置末端 RS485 Modbus
            # arm_l.set_modbus_mode(port=1, baudrate=115200, timeout=3)

            # 3) 初始化/激活钧舵夹爪
            # device 要改成你的实际从站地址
            # 钧舵手册示例常用 09，但你的设备不一定就是 9
            # arm_l.control_gripper_jd("init", port=1, device=9)

            # # 4) 轮询激活状态
            # for _ in range(20):
            #     state = arm_l.get_gripper_status_jd(port=1, device=9)
            #     print("active=", state.active, "activated=", state.activated, "gsta=", state.gsta_text)
            #     if state.activated:
            #         break
            #     time.sleep(0.2)

            # 5) 激活完成后再开合
            # arm_l.control_gripper_jd("init", speed=128, force=128, port=1, device=9, block=True, timeout=3)
            # arm_l.control_gripper_jd("open", speed=128, force=128, port=1, device=9, block=True, timeout=3)
            # time.sleep(3)
            # arm_l.control_gripper_jd("close", speed=128, force=128, port=1, device=9, block=True, timeout=3)
           #time.sleep(0.5)

            # 6) 读状态
            # state = arm_l.get_gripper_status_jd(port=1, device=9)
            # print("holding=", state.holding, "gobj=", state.gobj_text, "pos=", state.current_position)

    except ArmApiError as exc:
        print(f"Robot API error: {exc}")
    except Exception as exc:
        print(f"Unhandled error: {exc}")


if __name__ == "__main__":
    main()
