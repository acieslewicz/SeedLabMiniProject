import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def configure_communication():
    lcd_columns = 16
    lcd_rows = 2
    
    # Initialise I2C bus.
    bus = smbus.SMBus(1)
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [0, 120, 180]
    time.sleep(1)
    return bus, i2c, lcd
    
def write_messages(lcd, message1="", message2=""):
    if message1 is not None:
        message1 = str(message1)
    
    if message2 is not None:
        message2 = str(message2)
    
    message = message1 + "\n" + message2

    lcd.message = message
    
def turn_lcd_off(lcd):
    lcd.clear()
    lcd.color = [0,0,0]
    time.sleep(1)