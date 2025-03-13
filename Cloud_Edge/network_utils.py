import os
import subprocess
import signal
import platform

# 检查端口占用函数
def check_and_kill_port(port):
    system = platform.system().lower()

    if system == "windows":
        try:
            # 查找占用端口的进程ID
            cmd_find = f'netstat -ano | findstr :{port}'
            output = subprocess.check_output(cmd_find, shell=True).decode('gbk')
            if output:
                pid = output.strip().split()[-1]
                # 终止进程
                subprocess.run(f'taskkill /PID {pid} /F', shell=True, check=True)
                print(f"已强制终止占用{port}端口的进程(PID: {pid})")
        except subprocess.CalledProcessError:
            print(f"{port}端口未被占用")

    elif system in ["linux", "darwin"]:  # macOS属于darwin
        try:
            # 查找进程ID
            cmd_find = f'lsof -ti:{port}'
            pids = subprocess.check_output(cmd_find, shell=True).decode().split()
            # 终止所有相关进程
            for pid in pids:
                os.kill(int(pid), signal.SIGKILL)
                print(f"已强制终止占用{port}端口的进程(PID: {pid})")
        except:
            print(f"{port}端口未被占用")
    else:
        raise OSError("不支持的操作系统")