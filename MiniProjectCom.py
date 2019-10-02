import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
#Function to configure the comunication between the arduino and the Raspberry Pi
def configure_communication():
    lcd_columns = 16
    lcd_rows = 2
    
    # Initialise I2C bus.
    bus = smbus.SMBus(1)
    #Set SDA and SCL to the connection on the I2c
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    #Sets background colour of the LCD
    lcd.color = [0, 100, 0]
    time.sleep(1)
    return bus, i2c, lcd
    #Function to configure the messages writted to the LCD
def write_messages(lcd, message1="", message2=""):
    if message1 is not None:
        message1 = str(message1)
    
    if message2 is not None:
        message2 = str(message2)
    #Concatinqtes the message
    message = message1 + "\n" + message2

    lcd.message = message
    #Function to turn the LCD off
def turn_lcd_off(lcd):
    lcd.clear()
    lcd.color = [0,0,0]
    time.sleep(1)
