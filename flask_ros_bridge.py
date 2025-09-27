# flask_ros_bridge.py
import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading

from xline_path_planner.srv import PlanPath

class FlaskRosBridge(Node):
    def __init__(self):
        super().__init__('flask_ros_bridge')
        
        # 创建ROS2 Service客户端
        self.plan_path_client = self.create_client(PlanPath, 'plan_path')
        
        # 等待服务可用
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_path service...')
        
        # 创建Flask应用
        self.app = Flask(__name__)
        CORS(self.app)
        
        # 设置路由
        @self.app.route('/plan_path', methods=['POST'])
        def plan_path_endpoint():
            try:
                # 解析请求数据
                data = request.get_json()
                if not data:
                    return jsonify({
                        'success': False,
                        'error': 'No JSON data provided',
                        'message': '请求数据为空'
                    }), 400
                
                file_name = data.get('file_name')
                
                if not file_name:
                    return jsonify({
                        'success': False,
                        'error': 'file_name is required',
                        'message': '文件名称不能为空'
                    }), 400
                
                # 调用ROS2服务
                result = self.call_plan_path_service(file_name)
                return jsonify(result)
                
            except Exception as e:
                self.get_logger().error(f'Error in plan_path endpoint: {e}')
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'message': '服务器内部错误'
                }), 500
        
        # 启动Flask服务器
        self.flask_thread = threading.Thread(target=self.run_flask_server)
        self.flask_thread.daemon = True
        self.flask_thread.start()
        
        self.get_logger().info('Flask ROS Bridge started on http://0.0.0.0:5000')
    
    def call_plan_path_service(self, file_name):
        """调用ROS2路径规划服务"""
        try:
            # 创建服务请求
            request_msg = PlanPath.Request()
            request_msg.file_name = file_name
            
            # 调用服务
            self.get_logger().info(f'Planning path for file: {file_name}')
            future = self.plan_path_client.call_async(request_msg)
            
            # 等待响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                response = future.result()
                return {
                    'success': response.success,
                    'error': response.error,
                    'message': response.message,
                    'file_name': file_name
                }
            else:
                return {
                    'success': False,
                    'error': 'Service call timeout',
                    'message': f'文件 "{file_name}" 规划超时',
                    'file_name': file_name
                }
                
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')
            return {
                'success': False,
                'error': str(e),
                'message': f'文件 "{file_name}" 规划过程中出现异常',
                'file_name': file_name
            }
    
    def run_flask_server(self):
        self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    bridge = FlaskRosBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()