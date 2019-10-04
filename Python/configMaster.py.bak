import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
# bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
# number = bus.read_byte_data(address, 1)
    return number

while True:
    var = input("Enter 1 – 9: ")
    if var:
        break

writeNumber(int(var, 10))
print("RPI: Hi Arduino, I sent you ", var)
# sleep one second
time.sleep(1)

number = readNumber()
print("Arduino: Hey RPI, I received a digit ", number)
time.sleep(2)
lcd.clear()
print("Arduino: Hey RPI, I received ", number)
lcd.clear()
lcd.color = [100, 0, 0]
time.sleep(1)
lcd.message = "Sent: " + var
lcd.message = "\nGot: %d" %(number) 
time.sleep(2)
