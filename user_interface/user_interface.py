from flask import Flask, render_template
from backend_logging import get_logger
import logging

class UI():
    """
    This class is responsible for creating the user_interface
    and receiving and sending messages to/from it
    """
    def __init__(self):
        self.app = None
    
    def start_ui(self):
        self.app = Flask(__name__)

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/start_destack')
        def button_start_destack():
            # Code to execute when button 1 is clicked
            # Replace this with your own command execution logic
            get_logger(__name__).log(logging.INFO,
                                     f"Start destacking button clicked")
            return

        @self.app.route('/emergency_stop')
        def button_emergency_stop():
            # Code to execute when button 2 is clicked
            # Replace this with your own command execution logic
            get_logger(__name__).log(logging.INFO,
                                     f"Emergency stop button clicked")
            return
        
        @self.app.route('/destack_done')
        def button_destack_done():
            # Code to execute when button 2 is clicked
            # Replace this with your own command execution logic
            get_logger(__name__).log(logging.INFO,
                                     f"Destacking done button clicked")
            return
        try:
            self.app.run(debug=True)
        except Exception as e:
            get_logger(__name__).log(logging.ERROR,
                                     f"Error: {e}")