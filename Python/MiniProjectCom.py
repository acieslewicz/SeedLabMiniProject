"""
Mini Project OpenCV and communication integration
========

Handles configuring and controlling LCD in addition to configuring i2c bus 

Look how easy it is to use:

    Import as a module in order to access the functions inside
	
Credits
--------
Alexander Cieslewicz

Features
--------

- Configures i2c bus and 2x16 LCD display
- Writes 2 line messages to lcd screen
- Turns off lcd

Dependencies
------------

- smbus
- busio
- board
- time
- adafruit_character_lcd.character_lcd_rgb_i2c

Support
-------
If there are any issues email us at: acieslewicz@mymail.mines.edu
"""

import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def configure_communication():
    """Configures i2c devices and bus for communication with LCD and Arduino"""
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

def write_messages(lcd, message1="", message2=""):
    """Prints 2 line messages on a 2x16 adafruit lcd
    
    Keyword arguments:
    lcd -- lcd object to display on
    message1 -- desired message for top line of lcd
    message2 -- desired message for bottom line of lcd
    """
    #Format the two messages so that they are displayed as desired on the LCD
    if message1 is not None:
        message1 = str(message1)
    
    if message2 is not None:
        message2 = str(message2)
    message = message1 + "\n" + message2

    lcd.message = message

def turn_lcd_off(lcd):
    """Clears and turns off the LCD
    
    Keyword arguments:
    lcd -- lcd object to turn off
    """
    lcd.clear()
    lcd.color = [0,0,0]
    time.sleep(1)
