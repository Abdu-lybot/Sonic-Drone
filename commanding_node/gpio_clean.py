import RPi.GPIO as GPIO

class Rpi_gpio_comm_off:

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(7, GPIO.OUT)


    def start(self):
        #GPIO.cleanup()
        GPIO.output(12, GPIO.LOW)
        GPIO.output(7, GPIO.LOW)

if __name__ == "__main__":

    com = Rpi_gpio_comm_off()
    com.start()
