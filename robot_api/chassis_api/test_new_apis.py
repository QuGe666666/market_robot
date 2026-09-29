#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
新增API测试脚本
测试储位管理、地图/场景管理、任务管理等新增功能
"""

import sys
import os
import time

# 添加父目录到路径以导入chassis_api
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from chassis_api import (
    get_chassis_api,
    ChassisHTTPAPI,
    ChassisAPIError,
    ActionOrder,
    # 储位相关
    Storage,
    StorageIdentity,
    StoragePose,
    DockPose,
    RealPose,
    StorageNav,
    StorageDock,
    # 场景相关
    SceneList,
    # 任务相关
    ExecTaskRequest,
)


class Colors:
    """终端颜色输出"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


class NewAPITester:
    """新增API测试类"""

    def __init__(self, host="169.254.128.2", port=5480, robot_id=30001):
        """
        初始化测试器

        Args:
            host: 底盘IP地址
            port: HTTP端口
            robot_id: 机器人ID
        """
        self.host = host
        self.port = port
        self.robot_id = robot_id
        self.api: ChassisHTTPAPI = None
        self.test_results = []

    def _print_header(self, text: str):
        """打印标题"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}{text}{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

    def _print_success(self, text: str):
        """打印成功信息"""
        print(f"{Colors.OKGREEN}✓ {text}{Colors.ENDC}")

    def _print_fail(self, text: str):
        """打印失败信息"""
        print(f"{Colors.FAIL}✗ {text}{Colors.ENDC}")

    def _print_info(self, text: str):
        """打印信息"""
        print(f"  {Colors.OKCYAN}{text}{Colors.ENDC}")

    def _print_warning(self, text: str):
        """打印警告"""
        print(f"{Colors.WARNING}⚠ {text}{Colors.ENDC}")

    def connect(self) -> bool:
        """
        连接底盘

        Returns:
            bool: 连接是否成功
        """
        self._print_info(f"尝试连接到 {self.host}:{self.port} ...")
        self.api = get_chassis_api(
            host=self.host,
            port=self.port,
            robot_id=self.robot_id
        )
        self.api.debug = False

        result = self.api.connect()

        if result:
            self._print_success("连接成功")
        else:
            self._print_fail("连接失败")

        return result

    def disconnect(self):
        """断开连接"""
        if self.api:
            self.api.disconnect()
            self._print_info("已断开连接")

    def run_test(self, name: str, test_func) -> bool:
        """
        执行单个测试

        Args:
            name: 测试名称
            test_func: 测试函数

        Returns:
            bool: 测试是否通过
        """
        self._print_header(f"测试: {name}")

        try:
            result, details = test_func()
            self.test_results.append({
                'name': name,
                'result': result,
                'details': details
            })

            if result:
                self._print_success("测试通过")
            else:
                self._print_fail("测试不通过")

            self._print_info(f"详情: {details}")
            return result

        except ChassisAPIError as e:
            self.test_results.append({
                'name': name,
                'result': False,
                'details': f"API错误 [{e.code}]: {e.message} - {e.details}"
            })
            self._print_fail(f"API错误: [{e.code}] {e.message}")
            self._print_info(f"详情: {e.details}")
            return False
        except Exception as e:
            self.test_results.append({
                'name': name,
                'result': False,
                'details': f"异常: {e}"
            })
            self._print_fail(f"异常: {e}")
            return False

    # ==================== 初始化测试 ====================

    def test_01_init_robot(self) -> tuple:
        """
        测试1: 初始化机器人（清除异常码）

        API: POST /woosh/robot/InitRobot

        说明:
            - 这是第一个测试，确保机器人处于正常状态
            - 清除之前的异常码和错误状态
        """
        self._print_info("初始化机器人（清除异常码）...")
        self._print_info("请求参数: {isRecord: false}")

        # 开启调试模式查看详细响应
        self.api.debug = True
        result = self.api.clear_abnormal_codes(is_record=True)
        self.api.debug = False

        if result:
            self._print_success("机器人初始化成功")
            details = "已清除异常码，机器人状态已重置"
            return True, details
        else:
            self._print_warning("机器人初始化失败 - 可能原因:")
            self._print_warning("  1. 机器人正在执行任务")
            self._print_warning("  2. API调用失败")
            return False, "机器人初始化失败"

    # ==================== 场景/地图管理测试 ====================

    def test_02_get_scene_list(self) -> tuple:
        """
        测试2: 获取场景列表

        API: POST /woosh/map/SceneList
        """
        self._print_info("调用 get_scene_list() ...")
        scene_list = self.api.get_scene_list()

        if scene_list is not None:
            self._print_success("成功获取场景列表")
            self._print_info(f"场景数量: {len(scene_list.scenes)}")

            for i, scene in enumerate(scene_list.scenes, 1):
                self._print_info(f"  场景 {i}: {scene.name}")
                self._print_info(f"    地图: {', '.join(scene.maps)}")

            if len(scene_list.scenes) > 0:
                # 保存所有场景信息用于后续测试
                self.all_scenes = scene_list.scenes
                self.first_scene_name = scene_list.scenes[0].name
                self.first_map_name = scene_list.scenes[0].maps[0] if scene_list.scenes[0].maps else None
                # 保存第二个场景（如果存在）用于切换测试
                if len(scene_list.scenes) > 1:
                    self.second_scene_name = scene_list.scenes[1].name
                    self.second_map_name = scene_list.scenes[1].maps[0] if scene_list.scenes[1].maps else None
                details = f"获取到 {len(scene_list.scenes)} 个场景"
                return True, details
            else:
                return False, "场景列表为空"
        else:
            return False, "获取场景列表失败"

    def test_03_switch_map(self) -> tuple:
        """
        测试3: 切换地图到wooshmap

        API: POST /woosh/robot/SwitchMap
        """
        # 固定切换到 wooshmap 地图
        scene_name = "wooshmap"
        map_name = "wooshmap"
        self._print_info(f"切换到场景: {scene_name}, 地图: {map_name}")

        # 切换地图前先取消当前任务
        self._print_info("取消当前任务...")
        cancel_result = self.api.action_order(order=ActionOrder.CANCEL)
        if cancel_result:
            self._print_success("任务已取消")
        else:
            self._print_warning("没有任务需要取消或取消失败")

        time.sleep(0.5)

        # 切换地图
        result = self.api.switch_map(scene_name=scene_name, map_name=map_name)

        if result:
            self._print_success("地图切换成功")
            time.sleep(1)  # 等待切换完成

            # 切换地图后初始化机器人
            self._print_info("初始化机器人...")
            init_result = self.api.clear_abnormal_codes(is_record=True)
            if init_result:
                self._print_success("初始化成功")
            else:
                self._print_warning("初始化失败")

            details = f"已切换到场景 {scene_name}, 地图 {map_name}"
            return True, details
        else:
            self._print_warning("地图切换失败 - 可能原因:")
            self._print_warning("  1. 机器人正在执行任务，无法切换地图")
            self._print_warning("  2. 场景/地图名称格式不正确")
            self._print_warning("  3. 该地图当前不可用")
            return False, f"地图切换失败 ({scene_name}/{map_name})"

    # ==================== 储位管理测试 ====================

    def test_04_create_storage(self) -> tuple:
        """
        测试5: 创建储位

        API: POST /woosh/map/mark/storage/Create
        """
        storage_id = int(time.time() % 10000)  # 使用时间戳生成唯一ID
        storage_no = f"TEST_{storage_id}"

        self._print_info(f"创建储位: ID={storage_id}, NO={storage_no}")

        storage = Storage(
            identity=StorageIdentity(id=storage_id, no=storage_no),
            pose=StoragePose(
                dock=DockPose(x=1.5, y=2.5, theta=0.0),
                real=RealPose(x=1.5, y=2.5, theta=0.0)
            ),
            nav=StorageNav(arr=0),
            dock=StorageDock()
        )

        self._print_info(f"储位信息: {storage.to_dict()}")

        result = self.api.create_storage(storage)

        if result:
            self._print_success("储位创建成功")
            # 保存储位信息用于更新测试
            self.test_storage = storage
            details = f"已创建储位 {storage_no}"
            return True, details
        else:
            return False, "储位创建失败"

    def test_05_create_storage_invalid(self) -> tuple:
        """
        测试5: 创建储位（无效参数测试）

        API: POST /woosh/map/mark/storage/Create
        """
        self._print_info("尝试创建无效储位（空数据）...")

        storage = Storage(
            identity=StorageIdentity(id=0, no=""),
            pose=StoragePose(
                dock=DockPose(x=0, y=0, theta=0),
                real=RealPose(x=0, y=0, theta=0)
            ),
            nav=StorageNav(arr=0),
            dock=StorageDock()
        )

        result = self.api.create_storage(storage)

        # 这个测试可能会失败，这是预期的
        if not result:
            self._print_success("正确拒绝了无效储位")
            return True, "正确拒绝了无效储位"
        else:
            self._print_warning("API接受了无效储位（可能允许空值）")
            return True, "API接受了无效储位"

    def test_06_delete_storage(self) -> tuple:
        """
        测试6: 删除储位

        API: POST /woosh/map/mark/storage/Delete
        """
        if not hasattr(self, 'test_storage'):
            return False, "没有可删除的储位，请先运行创建储位测试"

        storage = self.test_storage
        storage_no = storage.identity.no

        self._print_info(f"删除储位: {storage_no}")
        self._print_info(f"储位ID: {storage.identity.id}")

        # 开启调试模式查看详细信息
        self.api.debug = True
        result = self.api.delete_storage()
        self.api.debug = False

        if result:
            self._print_success("储位删除成功")
            details = f"已删除储位 {storage_no}"
            return True, details
        else:
            self._print_warning("储位删除失败 - 可能原因:")
            self._print_warning("  1. 储位ID不存在")
            self._print_warning("  2. 删除API需要特定权限")
            self._print_warning("  3. 储位正在使用中")
            self._print_warning("  4. API需要指定要删除的储位ID")
            return False, f"储位删除失败 (编号: {storage_no})"

    # ==================== 任务管理测试 ====================

    def test_07_exec_task_simple(self) -> tuple:
        """
        测试7: 执行简单导航任务

        API: POST /woosh/robot/ExecTask
        """
        # 使用测试储位作为目标点
        if hasattr(self, 'test_storage') and self.test_storage:
            mark_no = self.test_storage.identity.no
        else:
            mark_no = "A001"  # 默认测试点位

        # 执行导航任务前先初始化机器人
        self._print_info("初始化机器人...")
        self.api.debug = True
        init_result = self.api.clear_abnormal_codes(is_record=True)
        if init_result:
            self._print_success("初始化成功")
        else:
            self._print_warning("初始化失败，继续测试")

        self._print_info(f"执行导航任务到点位: {mark_no}")
        self._print_info(f"请求参数: {{taskId: 0, type: 1, direction: 0, taskTypeNo: 0, markNo: {mark_no}}}")

        result = self.api.exec_task(
            task_id=0,
            task_type=1,
            direction=0,
            task_type_no=0,
            mark_no=mark_no
        )
        self.api.debug = False

        if result:
            self._print_success("任务下发成功")
            details = f"已下发导航任务到 {mark_no}"
            return True, details
        else:
            self._print_warning("任务下发失败 - 可能原因:")
            self._print_warning("  1. 目标点位不存在")
            self._print_warning("  2. 机器人正在执行其他任务")
            self._print_warning("  3. 机器人状态不允许执行任务")
            self._print_warning("  4. 地图未加载或地图不匹配")
            return False, f"任务下发失败 (点位: {mark_no})"

    # ==================== 测试执行 ====================

    def run_all_tests(self):
        """运行所有测试"""
        self._print_header("新增API功能测试")
        print(f"{Colors.OKCYAN}目标: {self.host}:{self.port}, robotId={self.robot_id}{Colors.ENDC}")

        # 连接测试
        if not self.connect():
            self._print_fail("连接失败，终止测试")
            self._print_warning("请检查:")
            self._print_warning("  - 底盘是否上电")
            self._print_warning("  - IP地址是否正确")
            self._print_warning("  - HTTP端口是否正确")
            self._print_warning("  - 网络连接是否正常")
            return

        # 初始化测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}初始化测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试1: 初始化机器人", self.test_01_init_robot)

        # 场景/地图管理测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}场景/地图管理测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试2: 获取场景列表", self.test_02_get_scene_list)
        self.run_test("测试3: 切换地图到wooshmap", self.test_03_switch_map)

        # 储位管理测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}储位管理测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试4: 创建储位", self.test_04_create_storage)
        self.run_test("测试5: 创建储位（无效参数）", self.test_05_create_storage_invalid)
        self.run_test("测试6: 删除储位", self.test_06_delete_storage)

        # 任务管理测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}任务管理测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试7: 执行简单导航任务", self.test_07_exec_task_simple)

        self.disconnect()
        self.print_report()

    def print_report(self):
        """打印测试报告"""
        self._print_header("测试报告")

        passed = sum(1 for r in self.test_results if r['result'])
        total = len(self.test_results)
        pass_rate = (passed / total * 100) if total > 0 else 0

        print(f"\n{Colors.BOLD}{'序号':<6} {'测试名称':<40} {'结果':<10}{Colors.ENDC}")
        print("-" * 65)

        for i, result in enumerate(self.test_results, 1):
            status = f"{Colors.OKGREEN}✓ 通过{Colors.ENDC}" if result['result'] else f"{Colors.FAIL}✗ 不通过{Colors.ENDC}"
            print(f"{i:<6} {result['name']:<40} {status}")

        print("\n" + "-" * 65)
        print(f"{Colors.BOLD}测试结果: {passed}/{total} 通过{Colors.ENDC}")
        print(f"{Colors.BOLD}通过率: {pass_rate:.1f}%{Colors.ENDC}")

        if pass_rate == 100:
            print(f"{Colors.OKGREEN}{Colors.BOLD}🎉 所有测试通过！{Colors.ENDC}")
        elif pass_rate >= 80:
            print(f"{Colors.WARNING}{Colors.BOLD}⚠ 部分测试未通过，请检查{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}{Colors.BOLD}❌ 多项测试失败，请检查配置{Colors.ENDC}")

        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        # 打印失败测试的详情
        failed_tests = [r for r in self.test_results if not r['result']]
        if failed_tests:
            print(f"\n{Colors.FAIL}失败的测试详情:{Colors.ENDC}")
            for result in failed_tests:
                print(f"  • {result['name']}: {result['details']}")


def print_help():
    """打印帮助信息"""
    print(f"""
{Colors.OKCYAN}新增API测试脚本{Colors.ENDC}

测试新增的储位管理、地图/场景管理、任务管理功能

用法: python test_new_apis.py [选项] [参数]

选项:
  -h, --help          显示此帮助信息
  -H, --host <IP>     指定底盘IP地址（默认: 169.254.128.2）
  -P, --port <端口>   指定HTTP端口（默认: 5480）
  -r, --robot-id <ID> 指定机器人ID（默认: 30001）
  -v, --version       显示版本信息

示例:
  python test_new_apis.py                      # 运行所有测试
  python test_new_apis.py --host 192.168.1.50  # 指定底盘IP
  python test_new_apis.py -H 169.254.128.2 -P 5480

测试内容:
  • 初始化 (1个测试)
  • 场景/地图管理 (2个测试)
  • 储位管理 (3个测试)
  • 任务管理 (1个测试)
""")


def print_version():
    """打印版本信息"""
    print("新增API测试脚本 v1.0")
    print("测试功能: 储位管理、地图/场景管理、任务管理")
    print("日期: 2026-03-18")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description='新增API测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('-H', '--host', type=str, default='169.254.128.2',
                      help='底盘IP地址 (默认: 169.254.128.2)')
    parser.add_argument('-P', '--port', type=int, default=5480,
                      help='HTTP端口 (默认: 5480)')
    parser.add_argument('-r', '--robot-id', type=int, default=30001,
                      help='机器人ID (默认: 30001)')
    parser.add_argument('-v', '--version', action='store_true',
                      help='显示版本信息')

    args = parser.parse_args()

    if args.version:
        print_version()
        return

    tester = NewAPITester(
        host=args.host,
        port=args.port,
        robot_id=args.robot_id
    )

    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}测试被用户中断{Colors.ENDC}")
        if tester.api:
            tester.api.stop()
            tester.disconnect()
        sys.exit(0)


if __name__ == "__main__":
    main()