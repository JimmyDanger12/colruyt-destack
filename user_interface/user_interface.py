from flask import Flask, render_template

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

        @self.app.route('/button1')
        def button1():
            # Code to execute when button 1 is clicked
            # Replace this with your own command execution logic
            print("Button 1 clicked!")
            return 'Button 1 Clicked!'

        @self.app.route('/button2')
        def button2():
            # Code to execute when button 2 is clicked
            # Replace this with your own command execution logic
            print("Button 2 clicked!")
            return 'Button 2 Clicked!'

        self.app.run(debug=True)