"""patrol_console 入口：ROS2 节点在后台线程运行，网页服务器在主线程运行。

用法（在 foxy-live 容器里，环境已 source /opt/ros/foxy）：
    python3 -m patrol_console.main --config /home/mu/foxy_ws/patrol_console/config/console.yaml
宿主机上一键启动见 patrol_console/start_console.sh。

然后在浏览器打开 http://<WS 的 IP>:8000（容器是 host 网络，端口直达）。
按 Ctrl+C 退出，退出前会停掉从网页启动的所有进程。

与上游 robot_console 的差异：去掉仿真场景 / 地图库 / 导航 / GLIM 相关的构造，
遥控的 busy 门改为"规划器正在发 /cmd_vel_intent 就拒绝遥控"（自主导航与遥控必须互斥，
两个发布者交替发速度会让狗来回抢指令）。
"""
import argparse
import glob
import os
import signal
import socket
import sys
import threading

from . import config


def _use_venv(venv):
    """把 Python 依赖目录加到 sys.path 最前面，不用激活虚拟环境也能 import fastapi。

    目录里只装 fastapi、uvicorn 和它们的依赖；numpy、rclpy 等仍然用系统的。
    两种布局都认：标准 venv（lib/python3*/site-packages）或容器里 pip --target 出来的平铺目录
    （foxy 容器没有 ensurepip，建不了真 venv，见 setup_deps.sh）。
    只影响本进程，从网页启动的子进程（point_lio_node 等）看不到这些包。
    """
    paths = glob.glob(os.path.join(venv, 'lib', 'python3*', 'site-packages'))
    if not paths and os.path.isdir(venv):
        paths = [venv]
    if not paths:
        sys.exit(f'Python deps dir {venv} not found (server.venv in console.yaml). '
                 'Install it with: bash patrol_console/setup_deps.sh')
    for p in paths:
        if p not in sys.path:
            sys.path.insert(1, p)


def _port_free(host, port):
    """端口没被占用（没有另一个控制台在运行）才返回 True。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
            return True
        except OSError:
            return False


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description='巡检平台网页控制台后端')
    parser.add_argument('--config', help='console.yaml 的路径', required=True)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    cfg = config.load(args.config)
    _use_venv(cfg['server']['venv'])
    host, port = cfg['server']['host'], int(cfg['server']['port'])
    # 先确认没有另一个控制台在用这个端口：否则下面清理"残留进程"时会把它正在用的进程停掉
    if not _port_free(host, port):
        sys.exit(f'端口 {port} 已被占用：可能已经有一个控制台在运行，直接打开 http://localhost:{port} 即可')

    import uvicorn
    from rclpy.executors import SingleThreadedExecutor
    from .mapping import MappingData
    from .process_manager import ProcessManager
    from .ros_node import ConsoleNode
    from .server import create_app
    from .teleop import Teleop

    # Ctrl+C 交给 uvicorn 处理（它会先关闭网页服务，再执行下面 finally 里的清理）。
    # rclpy 不装自己的 SIGINT 处理：Humble+ 用 SignalHandlerOptions.NO；
    # Foxy 没有这个选项（signals 模块只有 GuardCondition），直接 init——uvicorn 后装的
    # Python 级处理器会覆盖 rcl 的 C 级处理器，行为一致。
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:                     # Foxy
        rclpy.init(args=sys.argv)
    # 直接关掉终端窗口时进程收到的是 SIGHUP，默认会立刻退出、不做清理，
    # 从网页启动的进程（在单独的进程组里）就会留在后台。这里把它当成 Ctrl+C 处理
    signal.signal(signal.SIGHUP, lambda *_: os.kill(os.getpid(), signal.SIGINT))
    node = ConsoleNode(cfg)
    mapping = MappingData(node, cfg['mapping'])

    teleop_topic = cfg['teleop'].get('topic', '/cmd_vel')

    def planner_active():
        """自主导航时规划器在发 /cmd_vel_intent，遥控必须让位：两个发布者交替发速度，
        speed_adapter 收到的指令会来回跳，狗会抽搐。只数"别的节点"的发布者——
        控制台自己在这个话题上就有两个（遥控 + 急停零速），不能算进去。"""
        return any(p.node_name != node.get_name()
                   for p in node.get_publishers_info_by_topic(teleop_topic))

    teleop = Teleop(node, cfg['teleop'], busy=planner_active)
    # 单线程 executor：所有回调都很短，不需要多线程
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    procs = ProcessManager(cfg['stacks'], cfg['server']['log_dir'], node, f'.running_{port}.json')
    cleaned = procs.cleanup_orphans()
    if cleaned:
        node.get_logger().warn(f'已停止上次残留的进程：{", ".join(cleaned)}')

    def session_info():
        """实际执行的命令，写进运行记录（run.json）。"""
        info = {}
        for key in ('slam',):
            s = procs.get(mapping.stacks.get(key))
            if s is not None and s.state() in ('running', 'stopping'):
                info[f'{key}_cmd'] = ' '.join(s.run_cmd or s.cmd)
        return info
    mapping.run_info = session_info

    app = create_app(cfg, node, procs, mapping, teleop)
    node.get_logger().info(f'网页控制台：http://localhost:{port}（监听 {host}）')
    try:
        uvicorn.run(app, host=host, port=port, log_level='warning', timeout_graceful_shutdown=3)
    finally:
        # 清理期间再按 Ctrl+C 或关窗口都不打断，否则进程会留在后台
        # （Point-LIO 退出存地图可能要十几秒）
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        node.get_logger().info('正在停止从网页启动的进程（存地图可能要十几秒，请不要关闭窗口）……')
        procs.stop_all()
        mapping.close()               # 结束当前运行记录（写入结束时间和概况）
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
