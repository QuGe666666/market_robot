
#!/usr/bin/env python3  # 指定解释器路径，方便在Linux/Unix下直接运行本文件
# -*- coding: utf-8 -*-  # 指定文件编码为utf-8，确保中文注释和字符串不会乱码

"""
motor_service.py
================

"""
"""
motor_service.py
================
工业级“电机服务核心”：
- 串口只在这里独占打开
- 运动任务互斥：同一时刻只能一个（位置/速度）
- STOP 最高优先级：随时打断
- telemetry 周期采样缓存（WS/HTTP 从缓存取）
- 错误日志：持久缓存 + 可清空 + WS 推送可显示

"""




import time  # 导入标准库time，提供sleep、time等函数，用于计时、延时、获取当前时间戳
import threading  # 导入标准库threading，提供多线程支持，包括锁、事件、线程对象
from dataclasses import dataclass  # 导入dataclasses库，@dataclass用于简化数据结构定义
from typing import Optional, Dict, Any, List  # 导入typing库，提供类型注解，提升代码可读性和IDE提示
from collections import deque  # 导入collections库的deque，双端队列，适合高效日志缓存

from leesn_control import KTechMotor, LiftAxis  # 导入自定义模块ktech_lift中的KTechMotor（电机底层通信类）和LiftAxis（升降轴高层封装类）




MODE_IDLE = "idle"  # 字符串常量，表示当前无运动任务，系统处于空闲状态
MODE_SPEED = "speed"  # 字符串常量，表示当前处于速度控制模式
MODE_POSITION = "position"  # 字符串常量，表示当前处于位置控制模式




@dataclass  # 自动生成__init__、__repr__等方法，简化数据结构定义
class Telemetry:
    ts: float  # 采样时间戳，float类型，单位为秒，记录本次采样的时间
    pos_mm: Optional[float]  # 当前升降轴的线性位置，单位mm，None表示无效
    speed_dps: Optional[float]  # 当前电机速度，单位度每秒（deg/s），None表示无效
    temp_c: Optional[int]  # 当前电机温度，单位摄氏度，None表示无效
    encoder: Optional[int]  # 当前编码器原始值，通常为绝对值或增量值，None表示无效
    iq_raw: Optional[int]  # 电机电流或功率原始值，具体含义依赖底层协议，None表示无效

    angle_raw_001deg: Optional[int]  # 电机多圈角度原始值，单位0.01度（int），None表示无效
    angle_deg: Optional[float]  # 电机多圈角度，单位度（float），None表示无效
    invert_dir: Optional[bool]  # 方向反转标志，True表示方向取反，None表示未知

    low_mm: Optional[float]  # 软限位下限，单位mm，None表示未设置
    high_mm: Optional[float]  # 软限位上限，单位mm，None表示未设置

    pulley_ratio: Optional[float]  # 皮带轮传动比，float，None表示未知
    screw_lead_mm: Optional[float]  # 丝杆导程，单位mm/圈，float，None表示未知
    motor_gear_ratio: Optional[float]  # 电机减速比，float，None表示未知
    mm_per_rev_motor: Optional[float]  # 电机每转对应的线性位移，单位mm，float，None表示未知
    mm_per_deg: Optional[float]  # 每度对应的线性位移，单位mm/deg，float，None表示未知
    deg_per_mm: Optional[float]  # 每mm对应的角度，单位deg/mm，float，None表示未知

    mode: str  # 当前运动模式，字符串，取值见MODE_IDLE等
    busy: bool  # 当前是否有运动任务在运行，True为忙，False为空闲
    last_error: str  # 最近一次错误信息，字符串，空字符串表示无错误
    estop: bool  # 急停锁存状态，True表示急停锁存中，False表示正常



class MotorService:
    def __init__(
        self,
        port: str,  # 串口端口号，字符串类型，例如 'COM3' 或 '/dev/ttyUSB0'
        motor_id: int = 1,  # 电机ID，int类型，默认为1
        baudrate: int = 115200,  # 串口波特率，int类型，默认为115200
        pulley_in: float = 32.0,  # 输入端皮带轮齿数，float类型
        pulley_out: float = 24.0,  # 输出端皮带轮齿数，float类型
        screw_lead_mm: float = 8.0,  # 丝杆导程，单位mm，float类型
        motor_gear_ratio: float = 8.0,  # 电机减速比，float类型
        invert_dir: bool = False,  # 方向反转标志，bool类型
        timeout_s: float = 0.20,  # 串口超时时间，单位秒，float类型
        retries: int = 5,  # 串口通信重试次数，int类型
        dir_switch_delay_s: float = 0.20,  # 换向延迟，单位秒，float类型
        telemetry_period_s: float = 0.2,  # 采样周期，单位秒，float类型
        error_log_size: int = 80,  # 错误日志最大条数，int类型
        auto_release_brake: bool = True,  # 是否自动释放刹车，bool类型

    ):
        self.port = str(port)  # 保存串口端口号，确保为字符串
        self.motor_id = int(motor_id)  # 保存电机ID，确保为整数
        self.baudrate = int(baudrate)  # 保存波特率，确保为整数
        self.telemetry_period_s = float(telemetry_period_s)  # 保存采样周期，确保为浮点数
        self.auto_release_brake = bool(auto_release_brake)

        # 创建底层电机通信对象KTechMotor，独占串口资源
        self.motor = KTechMotor(
            port=self.port,  # 串口端口
            motor_id=self.motor_id,  # 电机ID
            baudrate=self.baudrate,  # 波特率
            timeout_s=float(timeout_s),  # 超时时间
            retries=int(retries),  # 重试次数
            dir_switch_delay_s=float(dir_switch_delay_s),  # 换向延迟
            debug=False,  # 关闭调试输出
        )

        # 创建升降轴高层对象LiftAxis，负责机械参数和运动学换算
        self.lift = LiftAxis(
            motor=self.motor,  # 传入底层电机对象
            pulley_in=float(pulley_in),  # 输入皮带轮齿数
            pulley_out=float(pulley_out),  # 输出皮带轮齿数
            screw_lead_mm=float(screw_lead_mm),  # 丝杆导程
            motor_gear_ratio=float(motor_gear_ratio),  # 电机减速比
            invert_dir=bool(invert_dir),  # 方向反转
        )

        # 互斥锁：用于运动任务的互斥，防止多线程同时操作
        self._motion_lock = threading.Lock()  # 线程锁，保护运动相关操作
        # 取消事件：用于异步任务的中断
        self._cancel_event = threading.Event()  # 事件对象，控制任务取消
        # 运动线程：保存当前运动任务的线程对象
        self._motion_thread: Optional[threading.Thread] = None  # 线程对象或None
        # 当前运动模式：idle/speed/position
        self._mode = MODE_IDLE  # 初始为idle

        # 错误日志相关
        self._last_error = ""  # 最近一次错误信息，字符串
        self._error_log = deque(maxlen=int(error_log_size))  # 错误日志队列，最大长度error_log_size
        # ========== E-STOP (latching) ==========
        self._estop_lock = threading.Lock()   # 急停状态锁（避免多线程同时改）
        self._estop_latched = False           # True=急停锁存中，False=正常

        # ========== speed guard ==========
        self._speed_guard_stop = threading.Event()
        self._speed_guard_thread = threading.Thread(target=self._speed_guard_loop, daemon=True)
        self._speed_guard_thread.start()

        self._speed_cmd_mm_s = 0.0           # 最近一次速度命令（mm/s）
        self._speed_block_dir = 0            # 方向锁死：+1 禁止上升，-1 禁止下降，0 不锁
        # telemetry缓存，保存最近一次采样数据，初始化时各项为None或默认值
        self._telemetry = Telemetry(
            ts=time.time(),  # 当前时间戳
            pos_mm=None, speed_dps=None, temp_c=None, encoder=None, iq_raw=None,  # 运动状态相关
            angle_raw_001deg=None, angle_deg=None, invert_dir=self.lift.invert_dir,  # 角度与方向
            low_mm=self.lift.limit_low_mm, high_mm=self.lift.limit_high_mm,  # 软限位
            pulley_ratio=self.lift.pulley_ratio,  # 皮带轮传动比
            screw_lead_mm=self.lift.screw_lead_mm,  # 丝杆导程
            motor_gear_ratio=self.lift.motor_gear_ratio,  # 电机减速比
            mm_per_rev_motor=self.lift.mm_per_rev_motor,  # 电机每转线性位移
            mm_per_deg=self.lift.mm_per_deg,  # 每度线性位移
            deg_per_mm=self.lift.deg_per_mm,  # 每mm角度
            mode=self._mode, busy=False, last_error="",  # 运动模式、忙标志、错误
            estop=self.get_estop(),

        )

        # telemetry线程相关
        self._tele_stop = threading.Event()  # 事件对象，控制采样线程停止
        self._tele_thread = threading.Thread(target=self._telemetry_loop, daemon=True)  # 采样线程，守护线程

        self._init_driver()  # 初始化电机驱动，清除错误并启动
        self._tele_thread.start()  # 启动采样线程

    # -------------------------
    # 错误日志
    # -------------------------
    def _log_error(self, where: str, err: Exception | str):  # 内部方法，记录错误日志，where为出错位置，err为异常对象或字符串
        msg = str(err)  # 将异常对象或错误信息转为字符串，便于记录
        entry = {"ts": time.time(), "where": str(where), "error": msg}  # 构造日志字典，包含时间戳、位置、错误内容
        self._error_log.append(entry)  # 将日志加入错误日志队列
        self._last_error = msg  # 更新最近一次错误信息

    def get_error_log(self) -> List[Dict[str, Any]]:  # 获取全部错误日志，返回字典列表
        return list(self._error_log)  # 将deque转为list返回

    def clear_error_log(self):  # 清空错误日志和最近错误
        self._error_log.clear()  # 清空错误日志队列
        self._last_error = ""  # 清空最近一次错误信息

    # -------------------------
    def _init_driver(self):
        try:
            # 不要在服务启动时向驱动器写入报警清除寄存器。
            # 某些驱动器会把该写操作当作报警复位/蜂鸣确认，导致上电瞬间报警声。
            # 需要清除硬件报警时由运维人员单独执行，避免启动流程产生运动相关副作用。
            # self.motor.clear_error()
            #self.motor.run()

            # # ✅ Web 服务启动后默认松闸（只做一次，不做维护逻辑）
            # if getattr(self, "auto_release_brake", True):
            #     try:
            #         # 有些设备需要 run 后稍等一下再松闸（保险）
            #         time.sleep(0.05)
            #         self.motor.brake_release()
            #     except Exception as e:
            #         # 松闸失败不让服务启动直接崩，但要记日志
            #         self._log_error("brake_release_on_startup", e)

            self._last_error = ""
        except Exception as e:
            self._log_error("_init_driver", e)


    # -------------------------
    # telemetry loop
    # -------------------------
    def _telemetry_loop(self):  # 采样线程主循环，定期采集电机和升降轴状态
        while not self._tele_stop.is_set():  # 如果未收到停止信号则持续循环
            try:  # 捕获采样过程中的所有异常，保证线程健壮
                angle_raw = self.motor.read_multi_turn_angle_raw()  # 读取电机多圈原始角度，单位0.01度
                angle_deg = float(angle_raw) / 100.0  # 转换为度，float类型
                pos_mm = self.lift._deg_to_mm(angle_deg)  # 角度转线性位置，单位mm
                s2 = self.motor.read_state2()  # 读取电机状态2，包含速度、温度、编码器、电流等

                self._telemetry = Telemetry(  # 构造新的采样数据对象，保存所有状态
                    ts=time.time(),  # 当前时间戳
                    pos_mm=float(pos_mm),  # 线性位置
                    speed_dps=float(s2.speed_dps),  # 速度，度每秒
                    temp_c=int(s2.temperature_c),  # 温度，摄氏度
                    encoder=int(s2.encoder),  # 编码器原始值
                    iq_raw=int(s2.iq_or_power_raw),  # 电流/功率原始值

                    angle_raw_001deg=int(angle_raw),  # 多圈角度原始值
                    angle_deg=float(angle_deg),  # 多圈角度，度
                    invert_dir=bool(self.lift.invert_dir),  # 方向反转标志

                    low_mm=self.lift.limit_low_mm,  # 软限位下限
                    high_mm=self.lift.limit_high_mm,  # 软限位上限

                    pulley_ratio=float(self.lift.pulley_ratio),  # 皮带轮传动比
                    screw_lead_mm=float(self.lift.screw_lead_mm),  # 丝杆导程
                    motor_gear_ratio=float(self.lift.motor_gear_ratio),  # 电机减速比
                    mm_per_rev_motor=float(self.lift.mm_per_rev_motor),  # 电机每转线性位移
                    mm_per_deg=float(self.lift.mm_per_deg),  # 每度线性位移
                    deg_per_mm=float(self.lift.deg_per_mm),  # 每mm角度

                    mode=self._mode,  # 当前运动模式
                    busy=self.is_busy(),  # 是否忙
                    last_error=self._last_error,  # 最近一次错误
                    estop=self.get_estop(),

                )

            except Exception as e:  # 捕获采样异常
                self._log_error("_telemetry_loop", e)  # 记录错误日志
                t = self._telemetry  # 取上一次采样数据，避免丢失
                self._telemetry = Telemetry(  # 用上一次数据构造新对象，保持数据结构完整
                    ts=time.time(),
                    pos_mm=t.pos_mm, speed_dps=t.speed_dps, temp_c=t.temp_c, encoder=t.encoder, iq_raw=t.iq_raw,
                    angle_raw_001deg=t.angle_raw_001deg, angle_deg=t.angle_deg, invert_dir=bool(self.lift.invert_dir),
                    low_mm=self.lift.limit_low_mm, high_mm=self.lift.limit_high_mm,
                    pulley_ratio=t.pulley_ratio, screw_lead_mm=t.screw_lead_mm, motor_gear_ratio=t.motor_gear_ratio,
                    mm_per_rev_motor=t.mm_per_rev_motor, mm_per_deg=t.mm_per_deg, deg_per_mm=t.deg_per_mm,
                    mode=self._mode, busy=self.is_busy(), last_error=self._last_error,
                    estop=self.get_estop(),
                )

            time.sleep(self.telemetry_period_s)  # 休眠指定采样周期，单位秒

    # -------------------------
    def get_telemetry(self) -> Dict[str, Any]:  # 获取最新一次采样数据，返回字典
        t = self._telemetry  # 取当前telemetry对象
        return {  # 返回一个字典，包含所有采样字段
            "ts": t.ts,  # 时间戳
            "pos_mm": t.pos_mm,  # 线性位置
            "speed_dps": t.speed_dps,  # 速度
            "temp_c": t.temp_c,  # 温度
            "encoder": t.encoder,  # 编码器原始值
            "iq_raw": t.iq_raw,  # 电流/功率原始值
            "angle_raw_001deg": t.angle_raw_001deg,  # 多圈角度原始值
            "angle_deg": t.angle_deg,  # 多圈角度
            "invert_dir": t.invert_dir,  # 方向反转
            "low_mm": t.low_mm,  # 软限位下限
            "high_mm": t.high_mm,  # 软限位上限

            # 换算参数（UI 可视化核对）
            "pulley_ratio": t.pulley_ratio,  # 皮带轮传动比
            "screw_lead_mm": t.screw_lead_mm,  # 丝杆导程
            "motor_gear_ratio": t.motor_gear_ratio,  # 电机减速比
            "mm_per_rev_motor": t.mm_per_rev_motor,  # 电机每转线性位移
            "mm_per_deg": t.mm_per_deg,  # 每度线性位移
            "deg_per_mm": t.deg_per_mm,  # 每mm角度

            "mode": t.mode,  # 当前运动模式
            "busy": t.busy,  # 是否忙
            "last_error": t.last_error,  # 最近一次错误
            "estop": t.estop,

        }

    def is_busy(self) -> bool:  # 判断当前是否有运动任务在运行
        th = self._motion_thread  # 取当前运动线程对象
        return bool(th is not None and th.is_alive())  # 线程存在且存活则为忙

    # -------------------------
    # limits
    # -------------------------
    def set_soft_limits_mm(self, low_mm: float, high_mm: float):  # 设置软限位，参数为下限和上限
        try:  # 捕获异常，保证调用安全
            self.lift.set_soft_limits_mm(low_mm, high_mm)  # 调用升降轴方法设置限位
            self._last_error = ""  # 成功则清空错误
        except Exception as e:  # 捕获异常
            self._log_error("set_soft_limits_mm", e)  # 记录错误
            raise  # 继续抛出异常

    # -------------------------
    # STOP highest priority
    # -------------------------
    def stop(self):  # 停止所有运动任务，最高优先级
        self._cancel_event.set()  # 设置取消事件，通知所有任务终止
        try:  # 捕获异常，保证调用安全
            self._speed_cmd_mm_s = 0.0  # 清零速度命令
            self.lift.stop()  # 调用升降轴停止方法
            self._last_error = ""  # 成功则清空错误
        except Exception as e:  # 捕获异常
            self._log_error("stop", e)  # 记录错误
        self._mode = MODE_IDLE  # 设置模式为空闲

    # -------------------------
    # E-STOP (latching)
    # -------------------------
    def get_estop(self) -> bool:
        """查询急停是否锁存"""
        with self._estop_lock:
            return bool(self._estop_latched)

    def estop_engage(self, reason: str = "E-STOP") -> None:
        """
        触发急停（锁存）：立即停机 + 锁存状态保持
        - 任何运动指令都应在外层被拒绝
        """
        with self._estop_lock:
            self._estop_latched = True

        # 最高优先级：立即停止所有运动
        try:
            self.stop()
            self._speed_cmd_mm_s = 0.0
        except Exception:
            pass

        # 留一个可读的错误信息给 UI/SDK
        self._last_error = f"{reason}: latched, motion blocked until released."

    def estop_release(self) -> None:
        """
        解除急停：只解除锁存，不自动恢复运动
        """
        with self._estop_lock:
            self._estop_latched = False
        self._last_error = ""

    def estop_toggle(self) -> bool:
        """
        点按机制：按一下触发并锁存，再按一下解除
        返回：切换后的状态（True=急停中）
        """
        with self._estop_lock:
            new_state = not self._estop_latched
            self._estop_latched = new_state

        if new_state:
            self.estop_engage("E-STOP")
        else:
            self.estop_release()
        return new_state
    
    

    def _speed_guard_loop(self):
        """
        速度模式软限位守护线程：
        - 只在 MODE_SPEED 且 speed_cmd != 0 时工作
        - 实时读取位置，逼近/越界则 stop + 记录错误 + 锁死方向
        """
        guard_period = 0.05          # 监控周期，单位秒，20Hz监控，建议不低于10Hz
        slow_zone_mm = 15.0           # 慢速区间，距离限位30mm内算“逼近限位”，用于提前预警
        hard_margin_mm = -0.999999999      # 允许的越界容忍，建议为0或极小，超出即强制停

        last_warn = 0.0              # 上一次发出预警的时间戳，用于节流，避免日志刷屏

        while not self._speed_guard_stop.is_set():  # 主循环，未收到停止信号则持续运行
            try:
                # E-STOP 最高优先级：强制停机 + 清零输出
                if self.get_estop():
                    # 清零指令/输出，避免解除急停后突然冲一下
                    self._speed_cmd_mm_s = 0.0
                    # 如果你有 _speed_out_mm_s（带斜坡输出）就也清零，没有就跳过
                    if hasattr(self, "_speed_out_mm_s"):
                        self._speed_out_mm_s = 0.0
                    try:
                        self.lift.stop()
                        print("speed_guard: E-STOP engaged, STOP")
                    except Exception:
                        pass
                    self._mode = MODE_IDLE
                    time.sleep(self._speed_guard_period_s if hasattr(self, "_speed_guard_period_s") else 0.05)
                    continue

                if self._mode != MODE_SPEED:  # 仅在速度模式下工作，否则休眠后继续
                    time.sleep(guard_period)  # 休眠一个监控周期
                    continue  # 跳到下次循环

                v = float(self._speed_cmd_mm_s)  # 当前速度命令，单位mm/s
                if abs(v) < 1e-9:  # 速度极小视为静止，跳过本次循环
                    time.sleep(guard_period)  # 休眠
                    continue  # 跳到下次循环

                # 读取当前位置（直接读底层，避免 telemetry 延迟）
                pos = self.lift.get_position_mm()  # 当前升降轴位置，单位mm
                low = self.lift.limit_low_mm  # 软限位下限
                high = self.lift.limit_high_mm  # 软限位上限

                now = time.time()  # 当前时间戳

                # ---- 向上运动监控 ----
                if v > 0 and high is not None:  # 速度为正且有上限
                    dist = high - pos
                    if pos >= high - slow_zone_mm and pos < high and now - last_warn > 0.05:  # 进入慢速区且距离上次预警超过0.5秒
                        v = v * (dist / slow_zone_mm)+10  # 线性降速
                        self.lift.set_speed_mm_s(v)
                        print(v)
                        # 记录当前速度命令给 guard 使用
                        self._speed_cmd_mm_s = v
                        self._log_error("speed_guard", f"Approaching HIGH limit: pos={pos:.2f}mm, high={high:.2f}mm")  # 记录逼近上限预警
                        last_warn = now  # 更新时间戳
                        
                    if pos >= high + hard_margin_mm:  # 超过上限+容忍区，触发硬保护
                        self.lift.stop()  # 立即停止
                        print("speed_guard: hit HIGH limit, STOP")
                        self._speed_cmd_mm_s = 0.0  # 清零速度命令
                        self._speed_block_dir = +1  # 锁死上升方向
                        self._mode = MODE_IDLE  # 切换为空闲
                        self._log_error("speed_guard", f"Hit HIGH soft limit: pos={pos:.2f} >= {high:.2f} (STOP)")  # 记录越界错误
                        continue  # 跳到下次循环

                # ---- 向下运动监控 ----
                if v < 0 and low is not None:  # 速度为负且有下限
                    dist = pos - low
                    if pos <= low + slow_zone_mm and now - last_warn > 0.05:  # 进入慢速区且距离上次预警超过0.5秒
                        v = v * (dist / slow_zone_mm)-10  # 线性降速
                        self.lift.set_speed_mm_s(v)
                        # 记录当前速度命令给 guard 使用
                        self._speed_cmd_mm_s = v
                        self._log_error("speed_guard", f"Approaching LOW limit: pos={pos:.2f}mm, low={low:.2f}mm")  # 记录逼近下限预警
                        last_warn = now  # 更新时间戳

                    if pos <= low - hard_margin_mm:  # 超过下限-容忍区，触发硬保护
                        self.lift.stop()  # 立即停止
                        print("speed_guard: hit LOW limit, STOP")
                        self._speed_cmd_mm_s = 0.0  # 清零速度命令
                        self._speed_block_dir = -1  # 锁死下降方向
                        self._mode = MODE_IDLE  # 切换为空闲
                        self._log_error("speed_guard", f"Hit LOW soft limit: pos={pos:.2f} <= {low:.2f} (STOP)")  # 记录越界错误
                        continue  # 跳到下次循环

            except Exception as e:  # 捕获所有异常，保证守护线程健壮
                self._log_error("speed_guard_loop", e)  # 记录异常

            time.sleep(guard_period)  # 每轮循环都休眠一个监控周期

    # -------------------------
    # speed control (A2)
    # -------------------------
    def set_speed_mm_s(self, speed_mm_s: float) -> bool:
        # E-STOP：急停锁存时拒绝任何位置任务
        if self.get_estop():
            self._log_error("set_speed_mm_s", "E-STOP latched: speed command rejected")
            return False

        # position 任务运行中：只能 STOP 抢占
        if self.is_busy() and self._mode == MODE_POSITION:
            self._log_error("set_speed_mm_s", "BUSY: position task running")
            return False
                # E-STOP 最高优先级：急停锁存时拒绝任何速度命令


        with self._motion_lock:
            self._cancel_event.clear()
            try:
                v = float(speed_mm_s)
                if not (-250 <= v <= 250):
                    raise ValueError("speed_mm_s out of range")

                # 如果之前撞了限位，锁死方向：禁止继续往那个方向走
                if self._speed_block_dir == +1 and v > 0:
                    raise ValueError("Blocked: HIGH soft limit already hit (STOP first / move down)")
                if self._speed_block_dir == -1 and v < 0:
                    raise ValueError("Blocked: LOW soft limit already hit (STOP first / move up)")

                # 前置检查（仍然要有）
                pos = self.lift.get_position_mm()
                if self.lift.limit_high_mm is not None and v > 0 and pos >= self.lift.limit_high_mm:
                    self._speed_block_dir = +1
                    raise ValueError(f"SoftLimit HIGH hit: pos={pos:.2f} >= {self.lift.limit_high_mm}")
                if self.lift.limit_low_mm is not None and v < 0 and pos <= self.lift.limit_low_mm:
                    self._speed_block_dir = -1
                    raise ValueError(f"SoftLimit LOW hit: pos={pos:.2f} <= {self.lift.limit_low_mm}")

                # 下发速度
                self.lift.set_speed_mm_s(v)
                print(f"set_speed_mm_s: {v:.2f} mm/s")
                # 记录当前速度命令给 guard 使用
                self._speed_cmd_mm_s = v

                if self._speed_block_dir == +1 and v < 0:
                    self._speed_block_dir = 0
                if self._speed_block_dir == -1 and v > 0:
                    self._speed_block_dir = 0

                self._mode = MODE_SPEED if abs(v) > 1e-6 else MODE_IDLE
                self._last_error = ""
                return True

            except Exception as e:
                self._log_error("set_speed_mm_s", e)
                return False


    # -------------------------
    # position task (A4 async, cancelable)
    # -------------------------
    def move_to_mm_pos_async(
        self,
        target_mm: float,
        max_speed_dps: int = 120,
        tol_mm: float = 1.0,
        timeout_s: float = 180.0,
    ) -> bool:  # 异步位置运动任务，参数为目标位置、最大速度、容差、超时
                # E-STOP：急停锁存时拒绝任何位置任务
        if self.get_estop():
            self._log_error("move_to_mm_pos_async", "E-STOP latched: position command rejected")
            return False

        if self.is_busy():  # 如果已有任务在运行
            self._log_error("move_to_mm_pos_async", "BUSY: task already running")  # 记录错误
            return False  # 返回失败

        locked = self._motion_lock.acquire(blocking=False)  # 非阻塞获取锁
        if not locked:  # 获取失败说明有其它运动在运行
            self._log_error("move_to_mm_pos_async", "BUSY: motion lock locked")  # 记录错误
            return False  # 返回失败

        def _task():  # 任务线程体
            try:
                self._cancel_event.clear()  # 清除取消事件
                self._mode = MODE_POSITION  # 设置为位置模式
                self._last_error = ""  # 清空错误

                tgt = float(target_mm)  # 目标位置
                spd = int(max_speed_dps)  # 最大速度

                if not (0 < spd <= 168000):  # 检查速度范围
                    raise ValueError("max_speed_dps out of range")

                # ✅ 软限位前置拦截
                self.lift._check_target(tgt)  # 检查目标位置是否在限位内

                # 下发 A4（mm -> motor_deg）
                target_deg = self.lift._mm_to_deg(tgt)  # 位置转角度
                self.motor.set_pos_multi_a4(target_deg, spd)  # 下发运动指令

                # 等待到位/取消/超时（运行中兜底限位）
                t0 = time.time()  # 记录起始时间
                while True:  # 循环等待
                    if self.get_estop():
                        try:
                            self.lift.stop()
                            print("move_task: E-STOP engaged, STOP")
                        except Exception:
                            pass
                        self._mode = MODE_IDLE
                        return

                    if self._cancel_event.is_set():  # 检查是否被取消
                        try:
                            self.lift.stop()  # 停止运动
                            print("move_task: canceled, STOP")
                        except Exception:
                            pass  # 忽略异常
                        self._mode = MODE_IDLE  # 设置为空闲
                        return  # 退出线程

                    cur = self.lift.get_position_mm()  # 获取当前位置
                    if abs(tgt - cur) <= float(tol_mm):  # 到达目标
                        self._mode = MODE_IDLE  # 设置为空闲
                        return  # 退出

                    if self.lift.limit_low_mm is not None and cur < self.lift.limit_low_mm - 1e-6:  # 低限位
                        self.lift.stop()
                        print("move_task: hit LOW limit, STOP")
                        raise RuntimeError(f"Hit LOW soft limit: cur={cur:.2f} < {self.lift.limit_low_mm}")
                    if self.lift.limit_high_mm is not None and cur > self.lift.limit_high_mm + 1e-6:  # 高限位
                        self.lift.stop()
                        print("move_task: hit HIGH limit, STOP")
                        raise RuntimeError(f"Hit HIGH soft limit: cur={cur:.2f} > {self.lift.limit_high_mm}")

                    if time.time() - t0 > float(timeout_s):  # 超时
                        self.lift.stop()
                        print("move_task: timeout, STOP")
                        raise TimeoutError(f"move timeout: target={tgt}, cur={cur:.2f}")

                    time.sleep(0.05)  # 50ms轮询

            except Exception as e:  # 捕获所有异常
                self._log_error("move_task", e)  # 记录错误
                try:
                    self.lift.stop()  # 兜底停止
                    print("move_task: stop() called in exception handler")
                except Exception:
                    pass
                self._mode = MODE_IDLE  # 设置为空闲

            finally:
                try:
                    self._motion_lock.release()  # 释放锁
                except Exception:
                    pass

        self._motion_thread = threading.Thread(target=_task, daemon=True)  # 创建线程
        self._motion_thread.start()  # 启动线程
        return True  # 返回成功

    # -------------------------
    def set_zero_flash_here(self, confirm: str) -> bool:  # 设置当前位置为零点并写入闪存
        if self.is_busy():  # 如果有任务在运行
            self._log_error("set_zero_flash_here", "BUSY: task running")  # 记录错误
            return False  # 返回失败

        with self._motion_lock:  # 加锁，保证线程安全
            try:
                self.stop()  # 停止所有运动
                self.motor.set_current_as_zero_flash(confirm=confirm)  # 设置零点并写入闪存
                self._last_error = ""  # 清空错误
                return True  # 成功
            except Exception as e:  # 捕获异常
                self._log_error("set_zero_flash_here", e)  # 记录错误
                return False  # 失败

    def brake_release(self) -> bool:
        """手动松闸（后续给网页按钮/SDK用）"""
        with self._motion_lock:
            try:
                self.motor.brake_release()
                self._last_error = ""
                return True
            except Exception as e:
                self._log_error("brake_release", e)
                return False

    def brake_engage(self) -> bool:
        """手动抱闸（后续给网页按钮/SDK用）"""
        with self._motion_lock:
            try:
                self.motor.brake_engage()
                self._last_error = ""
                return True
            except Exception as e:
                self._log_error("brake_engage", e)
                return False

    def brake_read_state(self) -> Optional[bool]:
        """
        读取抱闸状态
        返回:
        True  = 已松闸（通电释放）
        False = 已抱闸（断电刹车）
        None  = 读取失败（会写入 error_log）
        """
        with self._motion_lock:
            try:
                st = self.motor.brake_read()  # ktech_lift.py: 返回 0x00/0x01
                self._last_error = ""
                return True if int(st) == 0x01 else False
            except Exception as e:
                self._log_error("brake_read_state", e)
                return None


    def close(self):  # 关闭服务，安全释放资源
        self.stop()  # 停止所有运动
        self._tele_stop.set()  # 通知采样线程退出
        try:
            if self._tele_thread.is_alive():  # 检查线程是否存活
                self._tele_thread.join(timeout=1.0)  # 等待线程退出，最多1秒
        except Exception:
            pass  # 忽略异常
        self.motor.close()  # 关闭底层电机对象
        self._speed_guard_stop.set()
        try:
            if self._speed_guard_thread.is_alive():
                self._speed_guard_thread.join(timeout=1.0)
        except Exception:
            pass

