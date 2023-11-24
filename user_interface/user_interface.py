from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
from backend_logging import get_logger
import logging
import time
import os

class UI():
    """
    This class is responsible for creating the user_interface
    and receiving and sending messages to/from it
    """
    def __init__(self):
        self.server = None
        self.socketio = None
        self.last_position = 0
    
    def start_ui(self):
        self.server = Flask(__name__)
        self.socketio = SocketIO(self.server)

        @self.server.route('/')
        def index():
            return render_template('index.html')

        @self.socketio.on('start_destack')
        def button_start_destack():
            get_logger(__name__).log(logging.INFO,
                                     f"Start destacking button clicked")
            return

        @self.socketio.on('emergency_stop')
        def button_emergency_stop():
            get_logger(__name__).log(logging.INFO,
                                     f"Emergency stop button clicked")
            return
        
        @self.socketio.on('destack_done')
        def button_destack_done():
            get_logger(__name__).log(logging.INFO,
                                     f"Destacking done button clicked")
            return

        @self.socketio.on('connect')
        def handle_connect():
            emit('initial_logs', {'data': self.get_new_log_content().strip()})
            get_logger(__name__).log(
                logging.INFO,
                f"Starting log emission"
            )
    
        try:
            self.socketio.start_background_task(self.send_new_logs)
            self.socketio.run(self.server, debug=True)
        except Exception as e:
            get_logger(__name__).log(logging.ERROR,
                                     f"Error: {e}")
            raise
        finally:
            get_logger(__name__).log(
                100,
                f"Flask server shutting down"
            )
    def get_new_log_content(self):
        with open('logs/backend.log', 'r') as log_file:
            log_file.seek(self.last_position)
            new_content = log_file.read()
            self.last_position = log_file.tell()
        return new_content

    def send_new_logs(self):
        while True:
            new_content = self.get_new_log_content()
            if new_content:
                self.socketio.emit('new_log', {'data': new_content.strip()})
            time.sleep(1)  # Adjust sleep time as needed