from robot_mechanics.robot_controller import RobotController
from robot_mechanics.status import Status
from globalconfig import GlobalConfig
from backend_logging import setup_logging, get_logger
import logging
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import time

FIELD_ROBOT = "ROBOT"
FIELD_IP = "ip"

class Handler():
    """
    This is the main class responsible for connecting to all necessary resources
    and distributing messages correctly.
    """
    def __init__(self):
        self.robot_controller = None
        self.server = None
        self.socketio = None
        self.status = Status.Disconnected
        self.last_log_position = 0

    def start(self, config:GlobalConfig):
        setup_logging(config)
        
        robot_ip = config[FIELD_ROBOT,FIELD_IP]
        self.robot_controller = RobotController(robot_ip, self)
        
        get_logger(__name__).log(100,
                            f"Robot Handler starting...")
        try:
            self.start_server()
        except Exception as e:
            get_logger(__name__).log(logging.ERROR, f"Error: {e}")
            raise
        finally:
            get_logger(__name__).log(
                100,
                f"Flask server shutting down"
            )
    
    def start_server(self):
        self.server = Flask(__name__)
        self.socketio = SocketIO(self.server)

        @self.server.route('/')
        def index():
            return render_template('index.html')
        
        @self.socketio.on('connect')
        def handle_connect():
            emit('initial_logs', {'data': self.get_new_log_content().strip()})
            get_logger(__name__).log(
                logging.INFO,
                f"Client connected, starting log emission"
            )
            self.handle_command("connect_to_robot")
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            get_logger(__name__).log(logging.INFO,
                                     f"Client disconnected")

        @self.socketio.on('start_destack')
        def button_start_destack():
            self.handle_command("start_destack")

        @self.socketio.on('destack_done')
        def button_destack_done():
            self.handle_command("destack_done")    

        @self.socketio.on('emergency_stop')
        def button_emergency_stop():
            self.handle_command("emergency_stop")

        self.socketio.start_background_task(self.send_new_logs)
        self.socketio.run(self.server, debug=False)

    def get_new_log_content(self):
        with open('logs/backend.log', 'r') as log_file:
            log_file.seek(self.last_log_position)
            new_content = log_file.read()
            self.last_log_position = log_file.tell()
        return new_content

    def send_new_logs(self):
        while True:
            new_content = self.get_new_log_content()
            if new_content:
                self.socketio.emit('new_log', {'data': new_content.strip()})
            time.sleep(1)
    
    def change_status(self,status):
        self.status = status
        self.socketio.emit("update_status",self.status)

    def handle_command(self, command):
        get_logger(__name__).log(logging.INFO,
                                 f"Executing command '{command}'")
        if command == "connect_to_robot":
            self.robot_controller.connect()
        elif command == "destack_done":
            self.robot_controller.destack_done()
        elif command == "emergency_stop":
            self.robot_controller.stop()
        elif command == "start_destack":
            self.robot_controller.start_destack()