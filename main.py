# Uses code from https://bitbucket.org/MattHawkinsUK/ for the crowpi functions
import RPi.GPIO as GPIO
import smbus
import time
import Adafruit_CharLCD as LCD
import sys
import Adafruit_DHT


bus = smbus.SMBus(1)


class TempSensor():
    
    def __init__(self):
        sensor = 11
        pin = 4
        
    def get_temp(self):
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
        return [temperature, humidity]
    
    
class LightSensor():

    def __init__(self):

        # Define some constants from the datasheet

        self.DEVICE = 0x5c # Default device I2C address

        self.POWER_DOWN = 0x00 # No active state
        self.POWER_ON = 0x01 # Power on
        self.RESET = 0x07 # Reset data register value

        # Start measurement at 4lx resolution. Time typically 16ms.
        self.CONTINUOUS_LOW_RES_MODE = 0x13
        # Start measurement at 1lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_1 = 0x10
        # Start measurement at 0.5lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_2 = 0x11
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_1 = 0x20
        # Start measurement at 0.5lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_2 = 0x21
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_LOW_RES_MODE = 0x23

    def convertToNumber(self, data):
        return ((data[1] + (256 * data[0])) / 1.2)

    def readLight(self):
        data = bus.read_i2c_block_data(self.DEVICE,self.ONE_TIME_HIGH_RES_MODE_1)
        return self.convertToNumber(data)

def main():

    light_sensor = LightSensor()
    temperature = TempSensor()

    print("Light Level : " + str(light_sensor.readLight()) + " lx")
    time.sleep(0.5)
    print("Temperature is " + str(get_temp()[0]))
    print("Humidity is " + str(get_temp()[1]))

if __name__ == "__main__":
    main()


