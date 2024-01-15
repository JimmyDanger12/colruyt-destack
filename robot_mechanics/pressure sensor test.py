import pyfirmata2 as pyfirmata
import time

board = pyfirmata.Arduino('COM3')
it = pyfirmata.util.Iterator(board)
it.start()

analog_input1 = board.get_pin('a:0:i')
analog_input2 = board.get_pin('a:1:i')

while True:
    analog_value1 = analog_input1.read()
    analog_value2 = analog_input2.read()
    print('sensor 1:' + str(analog_value1))
    print('sensor 2:' + str(analog_value2))
    print(' ')
    time.sleep(0.5)