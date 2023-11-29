#File to control the led strip

import time
import board
import neopixel
import threading
import photoresistor_controller

#Start important variables for configuration
global on
on = False
global auto_brigtness
auto_brigtness = False
global brightness
brightness = 20
global current_color
current_color = '#FFFFFF'
global current_effect
current_effect = "static_color"

#Start global threads
global brightness_thread
brightness_thread = None
global effect_thread
effect_thread = None

#Function to send this configs to the other scripts
def receive_config_values():
    global on
    global auto_brigtness
    global brightness
    global current_color
    global current_effect

    return on, auto_brigtness, brightness, current_color, current_effect

#Function that other scripts may use to update configs
def update_config_values(on_received=None, auto_brigtness_received=None, brightness_received=None, current_color_received=None):
    global on
    global auto_brigtness
    global brightness
    global current_color

    if on_received != None:
        on = on_received
    if auto_brigtness_received != None:
        auto_brigtness = auto_brigtness_received
    if brightness_received != None:
        brightness = brightness_received
    if current_color_received != None:
        current_color = current_color_received
    
    return None

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D21

# The number of NeoPixels
num_pixels = 100

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=brightness/100, auto_write=False, pixel_order=ORDER
)


def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)

def static_color(color):
    if type(color)==str:
        color = color[1::]
        color = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
    pixels.fill(color)
    pixels.show()

#There are several ways that on and off can be activated.
#To avoid turning on when already on, or turning off when already off, we use a global variable that needs to be changed before calling the function
#That way, the toggling ways of on off can just call toggle_on_off() and avoid handling this
#For methods that say on or off directly (like the web page) we have to first check if the state now is the same as the one we want to set

def receive_on_off():
    global on
    return on

def toggle_on_off():
    global on
    if on:
        turn_off()
    else:
        turn_on()

def turn_off():
    global on
    if not on:
        return
    start_effect("static_color", '#000000', True)
    on = False

def turn_on():
    global on
    if on:
        return
    on = True
    global current_effect
    start_effect(current_effect)

class OnOffSensorThread(threading.Thread):
    def __init__(self, sensor_function, args=None, sleep_time=0.1):
        super().__init__()
        self.stop_event = threading.Event()
        self.sensor_function = sensor_function
        self.args = args
        self.sleep_time = sleep_time

    def run(self):
        while not self.stop_event.is_set():
            if self.call_sensor():
                toggle_on_off()
            time.sleep(self.sleep_time)

    def call_sensor(self):
        if self.args:
            return self.sensor_function(*self.args)
        else:
            return self.sensor_function()

    def stop(self):
        self.stop_event.set()

def change_color(color_received):
    global current_effect
    global current_color
    global effect_thread
    if str(effect_thread) == "color_chase":
        current_color = color_received
    else:
        current_effect = "static_color"
        current_color = color_received
        start_effect(current_effect)

def change_effect(effect):
    global current_effect
    if current_effect == effect:
        return
    start_effect(effect)

def change_brightness(brightness_received):
    try:
        if type(brightness_received) != int:
            brightness_received = int(brightness_received)
            if 0 > brightness_received or 100 < brightness_received:
                return
    except:
        pass
    global brightness
    brightness = brightness_received
    pixels.brightness = brightness/100
    pixels.show()

class BrightnessThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            brightness = photoresistor_controller.get_brightness(0, 1000, 10, 100)
            change_brightness(brightness)
            time.sleep(0.1)

    def stop(self):
        self.stop_event.set()

def toggle_auto_brightness():
    global auto_brigtness
    global brightness_thread
    auto_brigtness = not auto_brigtness
    if auto_brigtness:
        #Start auto brightness thread
        brightness_thread = BrightnessThread()
        brightness_thread.start()
    else:
        #Stop auto brightness thread
        brightness_thread.stop()
        brightness_thread.join()
        brightness_thread = None


class EffectThread(threading.Thread):
    def __init__(self, effect, color=None):
        super().__init__()
        self.stop_event = threading.Event()
        self.effect = effect
        if type(color)==str:
            color = color[1::]
            color = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
        self.color = color

    def run(self):
        global current_effect
        while not self.stop_event.is_set():
            if self.effect == "rainbow" and current_effect == self.effect:
                self.rainbow_cycle(0.001)
            elif self.effect == "color_chase" and current_effect == self.effect:
                self.color_chase_cycle(0.07)
    
    def rainbow_cycle(self, wait):
        for j in range(255):
            if self.stop_event.is_set():
                break
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + j
                pixels[i] = wheel(pixel_index & 255)
            pixels.show()
            time.sleep(wait)
    
    def color_chase_cycle(self, wait):
        global current_color
        n = 7 #Number of leds in the trail
        trains = 7 #Number of trains
        separation = num_pixels//trains #Number of leds between each train
        positions = [i*separation for i in range(trains)]
        #A whole cycle of the effect
        for i in range(num_pixels):
            color = current_color
            if type(color)==str:
                color = color[1::]
                color = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
                
            if self.stop_event.is_set():
                break
            #A single train
            for j in range(num_pixels):
                pixels[j] = (0,0,0)
            for j in range(trains):
                #The leds in the train
                brightness=1
                for k in range(n):
                    #The position of the led in the train
                    pos = positions[j]+i-k
                    #If the position is negative, add the number of leds to it
                    if pos < 0:
                        pos += num_pixels
                    #If the position is greater than the number of leds, subtract the number of leds from it
                    elif pos >= num_pixels:
                        pos -= num_pixels
                    #Calculate the brightness of the led
                    brightness = brightness*(2/3)
                    #Set the color with changed brightness
                    color_to_set = []
                    for c in color:
                        color_to_set.append(int(c*brightness))
                    pixels[pos] = (color_to_set)
            #Update the positions of the trains
            for j in range(trains):
                positions[j] += 1
                if positions[j] >= num_pixels:
                    positions[j] -= num_pixels
            #Show the leds
            pixels.show()
            time.sleep(wait)

    def stop(self):
        self.stop_event.set()

    def __str__(self) -> str:
        return self.effect

def start_effect(effect, color=None, do_not_change_effect=False):
    global on
    if not on:
        return
    global effect_thread
    if effect_thread != None:
        stop_effect()

    global current_effect
    global current_color
    if not do_not_change_effect:
        current_effect = effect
    #If the effect is static just call it
    if effect == "static_color":
        if color!=None:
            static_color(color)
        else:
            static_color(current_color)
    #If not, Start a stoppable thread to run the effect
    elif effect == "rainbow":
        effect_thread = EffectThread(effect)
        effect_thread.start()
    
    elif effect == "color_chase":
        if color!=None:
            effect_thread = EffectThread(effect, color)
        else:
            effect_thread = EffectThread(effect, current_color)
        effect_thread.start()

def stop_effect():
    global effect_thread
    if str(effect_thread) != "rainbow" or str(effect_thread) == "color_chase":
        effect_thread.stop()
        effect_thread.join()
        effect_thread = None




