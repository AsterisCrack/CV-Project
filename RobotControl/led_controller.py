import time
import board
import neopixel
import threading

global brightness
brightness = 20
global current_color
current_color = '#FFFFFF'
global current_effect
current_effect = "static_color"