import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 导入标准消息类型 String
import bluetooth                  # 导入 PyBluez 库
import threading                  # 导入线程库
import select                     # 导入 select 库，用于非阻塞 I/O
import time                       # 导入时间库，用于延时
import socket                     # 导入 socket 库，用于 shutdown

class BluetoothNode(Node):
    def __init__(self):
        super().__init__('bluetooth_node') # 初始化 ROS 2 节点，节点名为 'bluetooth_node'
        self.get_logger().info('蓝牙节点已启动') # 记录日志信息

        # --- ROS 2 通信 ---
        # 创建一个发布者，发布从蓝牙接收到的数据到 'bluetooth_data_received' 话题
        self.bt_data_publisher = self.create_publisher(String, 'bluetooth_data_received', 10)
        # 示例订阅者 (如果需要通过 ROS 发送数据到蓝牙，取消注释并实现回调函数)
        # self.bt_send_subscriber = self.create_subscription(
        #     String, 'send_bluetooth_data', self.send_bt_data_callback, 10)

        # --- 蓝牙设置 (使用 pybluez RFCOMM) ---
        self.server_sock = None         # 服务器套接字
        self.client_sock = None         # 客户端套接字
        self.client_info = None         # 客户端信息 (地址, 端口)
        self.receive_thread = None      # 接收数据线程
        self.accept_thread = None       # 接受连接线程
        self.setup_thread = None        # 蓝牙设置线程
        self.is_running = True          # 节点运行状态标志

        # 标准 SPP (Serial Port Profile) UUID
        self.uuid = "00001101-0000-1000-8000-00805F9B34FB"
        self.service_name = "VacuumCleanerBT" # 蓝牙服务名称

        # 在单独的线程中启动蓝牙设置，避免阻塞 __init__
        self.setup_thread = threading.Thread(target=self.setup_bluetooth)
        self.setup_thread.daemon = True # 设置为守护线程，主线程退出时自动退出
        self.setup_thread.start()       # 启动线程

    def setup_bluetooth(self):
        """设置蓝牙服务器套接字并开始接受连接。"""
        while self.is_running and rclpy.ok(): # 添加重试循环，直到成功或节点关闭
            try:
                # 创建 RFCOMM 蓝牙套接字
                self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                # 绑定到本地任意可用端口
                self.server_sock.bind(("", bluetooth.PORT_ANY))
                # 开始监听传入连接，允许最多1个连接排队
                self.server_sock.listen(1)

                # 获取实际绑定的端口号
                port = self.server_sock.getsockname()[1]

                # 广播蓝牙服务
                bluetooth.advertise_service(self.server_sock, self.service_name,
                                          service_id=self.uuid, # 服务 UUID
                                          service_classes=[self.uuid, bluetooth.SERIAL_PORT_CLASS], # 服务类型
                                          profiles=[bluetooth.SERIAL_PORT_PROFILE]) # 服务协议

                self.get_logger().info(f"等待蓝牙连接在 RFCOMM 通道 {port} (UUID: {self.uuid})")

                # 启动一个线程来接受连接
                if self.accept_thread is None or not self.accept_thread.is_alive():
                    self.accept_thread = threading.Thread(target=self.accept_connection_thread)
                    self.accept_thread.daemon = True
                    self.accept_thread.start()
                break # 设置成功，退出重试循环

            except bluetooth.BluetoothError as e:
                # 处理蓝牙相关的错误
                self.get_logger().error(f"蓝牙设置失败: {e}。5秒后重试...")
                if self.server_sock:
                    try:
                        self.server_sock.close() # 关闭可能已创建的套接字
                    except Exception: pass
                    self.server_sock = None
                time.sleep(5) # 等待5秒再重试
            except Exception as e:
                 # 处理其他意外错误
                 self.get_logger().error(f"蓝牙设置期间发生意外错误: {e}")
                 self.is_running = False # 停止节点运行
                 break

        if not self.is_running:
             self.get_logger().error("蓝牙设置永久失败。")


    def accept_connection_thread(self):
        """接受传入的蓝牙连接。"""
        while self.is_running and rclpy.ok() and self.server_sock:
            try:
                # 使用 select 实现非阻塞 accept，以便可以检查 is_running 状态
                # 等待服务器套接字变为可读状态，超时时间为 1 秒
                readable, _, _ = select.select([self.server_sock], [], [], 1.0)
                if self.server_sock in readable:
                    # 如果已有客户端连接，先关闭旧连接
                    if self.client_sock:
                        self.get_logger().warn("已连接到一个客户端。先关闭现有连接。")
                        self.close_client_connection()
                        # 可选：如果需要，添加短暂延时
                        # time.sleep(0.1)

                    # 接受新的连接请求
                    self.client_sock, self.client_info = self.server_sock.accept()
                    self.get_logger().info(f"接受来自 {self.client_info} 的连接")

                    # 在单独的线程中开始接收数据
                    if self.receive_thread is None or not self.receive_thread.is_alive():
                        self.receive_thread = threading.Thread(target=self.receive_data_thread)
                        self.receive_thread.daemon = True
                        self.receive_thread.start()
                    else:
                         # 理论上，如果清理逻辑正确，不应发生此情况
                         self.get_logger().warn("接收线程逻辑错误：线程已存在。")


            except bluetooth.BluetoothError as e:
                 # 处理蓝牙接受/连接错误，例如连接被重置
                 self.get_logger().warn(f"蓝牙接受/连接错误: {e}")
                 self.close_client_connection() # 确保在错误时清理资源
                 # 继续尝试接受连接
            except OSError as e:
                 # 如果在 select/accept 期间套接字被关闭，可能发生此错误
                 if self.is_running:
                     self.get_logger().warn(f"接受期间发生 OS 错误 (可能是套接字已关闭): {e}")
                 break # 如果套接字被外部关闭，则退出线程
            except Exception as e:
                if self.is_running: # 避免在关闭期间记录错误日志
                    self.get_logger().error(f"接受连接时出错: {e}")
                self.close_client_connection()
                # 考虑在其他错误后是中断还是继续接受
                time.sleep(1) # 避免在持续错误时进入忙循环

        self.get_logger().info("接受连接线程已结束。")


    def receive_data_thread(self):
        """从连接的客户端接收数据。"""
        self.get_logger().info("接收数据线程已启动。")
        buffer = "" # 用于存储可能不完整的消息片段
        while self.is_running and self.client_sock and rclpy.ok():
            try:
                # 使用 select 实现非阻塞 recv
                readable, _, _ = select.select([self.client_sock], [], [], 0.5) # 0.5 秒超时
                if self.client_sock in readable:
                    # 接收数据，缓冲区大小为 1024 字节 (可根据需要调整)
                    data = self.client_sock.recv(1024)
                    if not data:
                        # 如果 recv 返回空数据，表示客户端已断开连接
                        self.get_logger().info("客户端断开连接 (recv 返回空)。")
                        self.close_client_connection()
                        break # 客户端断开，退出循环

                    # 处理接收到的数据 - 处理潜在的部分消息
                    try:
                        # 将接收到的字节解码为 UTF-8 字符串并追加到缓冲区
                        buffer += data.decode('utf-8')
                        # 处理完整的消息 (例如，以换行符分隔的消息)
                        while '\n' in buffer:
                            # 按换行符分割消息
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip() # 去除首尾空白字符
                            if line: # 避免发布空字符串
                                self.get_logger().info(f"通过蓝牙接收到: {line}")
                                # 将数据发布到 ROS 话题
                                msg = String()
                                msg.data = line
                                self.bt_data_publisher.publish(msg)
                    except UnicodeDecodeError:
                        # 如果数据不是有效的 UTF-8 编码
                        self.get_logger().warn(f"通过蓝牙接收到非 UTF-8 数据: {data}")
                        buffer = "" # 解码失败时清空缓冲区
                        # 如果需要，在此处处理二进制数据

            except bluetooth.BluetoothError as e:
                # 处理蓝牙接收错误
                self.get_logger().warn(f"蓝牙接收错误: {e}")
                self.close_client_connection()
                break # 出错时退出循环
            except OSError as e:
                 # 如果在 select/recv 期间套接字被关闭，可能发生此错误
                 if self.is_running:
                     self.get_logger().warn(f"接收期间发生 OS 错误 (可能是套接字已关闭): {e}")
                 break # 如果套接字被外部关闭，则退出线程
            except Exception as e:
                 if self.is_running:
                    self.get_logger().error(f"接收线程出错: {e}")
                 self.close_client_connection()
                 break

        self.get_logger().info("接收数据线程已结束。")


    # --- 用于发送数据的示例函数 (可能由 ROS 订阅者调用) ---
    def send_bt_data(self, data_str):
        """通过蓝牙发送字符串数据。"""
        if self.client_sock:
            try:
                # 如果客户端期望以换行符结束，确保数据包含换行符
                if not data_str.endswith('\n'):
                    data_str += '\n'
                # 使用 sendall 确保所有数据都被发送
                self.client_sock.sendall(data_str.encode('utf-8'))
                self.get_logger().info(f"通过蓝牙发送: {data_str.strip()}") # 记录日志时不包含换行符
            except bluetooth.BluetoothError as e:
                # 处理蓝牙发送错误
                self.get_logger().error(f"通过蓝牙发送数据失败: {e}")
                self.close_client_connection() # 发送错误时假定连接丢失
            except Exception as e:
                 # 处理其他发送错误
                 self.get_logger().error(f"发送数据时发生意外错误: {e}")
                 self.close_client_connection()
        else:
            # 如果没有客户端连接
            self.get_logger().warn("无法发送数据，无客户端连接。")

    # --- 示例 ROS 订阅者回调函数 ---
    # def send_bt_data_callback(self, msg):
    #     self.send_bt_data(msg.data)

    def close_client_connection(self):
        """关闭客户端套接字并等待接收线程结束。"""
        if self.client_sock:
            sock_copy = self.client_sock # 在置为 None 之前复制句柄
            self.client_sock = None # 防止在 receive_thread 中继续使用
            try:
                # 通知对方关闭连接 (读写)
                sock_copy.shutdown(socket.SHUT_RDWR)
            except Exception: pass # 忽略 shutdown 时的错误
            try:
                # 关闭套接字
                sock_copy.close()
            except Exception as e:
                self.get_logger().warn(f"关闭客户端套接字时出错: {e}")
            finally:
                self.client_info = None
                self.get_logger().info("客户端连接已关闭。")

        # 确保在套接字关闭后接收线程结束
        if self.receive_thread and self.receive_thread.is_alive():
             # 不要从接收线程内部 join 自身
             current_thread = threading.current_thread()
             if self.receive_thread != current_thread:
                 self.receive_thread.join(timeout=1.0) # 等待最多 1 秒
                 if self.receive_thread.is_alive():
                     # 如果线程仍然存活，记录警告
                     self.get_logger().warn("接收线程未能干净地退出。")
             self.receive_thread = None


    def shutdown(self):
        """清理资源。"""
        if not self.is_running:
            return # 避免重复关闭
        self.get_logger().info("正在关闭蓝牙节点...")
        self.is_running = False # 设置运行标志为 False，通知其他线程停止

        # 先关闭客户端连接
        self.close_client_connection()

        # 关闭服务器套接字
        if self.server_sock:
            server_sock_copy = self.server_sock
            self.server_sock = None # 防止在 accept_thread 中继续使用
            try:
                # 停止广播服务 - PyBluez 可能没有显式的停止函数，关闭套接字通常可以处理
                server_sock_copy.close()
                self.get_logger().info("服务器套接字已关闭。")
            except Exception as e:
                self.get_logger().error(f"关闭服务器套接字时出错: {e}")

        # 等待所有线程结束
        if self.accept_thread and self.accept_thread.is_alive():
             self.accept_thread.join(timeout=1.0)
             if self.accept_thread.is_alive():
                 self.get_logger().warn("接受连接线程未能干净地退出。")
        if self.setup_thread and self.setup_thread.is_alive():
             # is_running 为 False 后，设置线程应该很快退出
             self.setup_thread.join(timeout=1.0)

        self.get_logger().info("蓝牙节点关闭完成。")


def main(args=None):
    rclpy.init(args=args) # 初始化 ROS 2 Python 客户端库
    bluetooth_node = None # 初始化节点变量
    try:
        bluetooth_node = BluetoothNode() # 创建蓝牙节点实例
        rclpy.spin(bluetooth_node) # 运行节点，处理回调函数等，直到节点关闭
    except KeyboardInterrupt:
        # 处理 Ctrl+C 中断
        print("收到 KeyboardInterrupt，正在关闭...")
    except Exception as e:
        # 处理主循环中的其他异常
        if bluetooth_node:
            bluetooth_node.get_logger().fatal(f"主循环中未处理的异常: {e}", exc_info=True)
        else:
            print(f"节点初始化前未处理的异常: {e}")
    finally:
        # 确保资源被释放
        if bluetooth_node:
            # 在销毁节点前显式调用 shutdown 方法
            bluetooth_node.shutdown()
            if rclpy.ok(): # 检查 rclpy 是否仍在运行
                bluetooth_node.destroy_node() # 销毁节点
        if rclpy.ok():
            rclpy.shutdown() # 关闭 ROS 2 Python 客户端库
        print("ROS 关闭完成。")


if __name__ == '__main__':
    main() # 如果脚本作为主程序运行，则调用 main 函数
