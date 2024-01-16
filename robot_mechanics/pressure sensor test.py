import pyfirmata2 as pyfirmata
import time


board = pyfirmata.Arduino('COM3')
it = pyfirmata.util.Iterator(board)
it.start()

analog_input1 = board.get_pin('a:0:i')
analog_input2 = board.get_pin('a:1:i')
DIG_OUT_ACT_SLI_EXT = board.digital[7]
DIG_OUT_ACT_SLI_RETR = board.digital[8]
while True:
    board.digital[6].write(1)
    analog_value1 = analog_input1.read()
    analog_value2 = analog_input2.read()
    print('sensor 1:' + str(analog_value1))
    print('sensor 2:' + str(analog_value2))
    print(' ')
    time.sleep(0.5)