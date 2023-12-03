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

global effect_thread
effect_thread = None

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D21

# The number of NeoPixels
num_pixels = 6

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=brightness/100, auto_write=False, pixel_order=ORDER
)

#Lighting effects
def static_color(color):
    if type(color)==str:
        color = color[1::]
        color = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
    pixels.fill(color)
    pixels.show()


def change_color(color_received):
    global current_effect
    global current_color
    global effect_thread
    if type(color_received)==str:
        if color_received == "red":
            color_received = "#FF0000"
        elif color_received == "green":
            color_received = "#00FF00"
        elif color_received == "blue":
            color_received = "#0000FF"
        elif color_received == "yellow":
            color_received = "#FFFF00"
        elif color_received == "white":
            color_received = "#FFFFFF"
            
    if str(effect_thread) == "breathe":
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
            if self.effect == "breathe" and current_effect == self.effect:
                self.breathe_cycle(0.03)
    
    def breathe_cycle(self, wait):
        global brightness
        b = brightness
        #Reduce brightness gradually
        for j in range(100, 1, -1):
            if self.stop_event.is_set():
                break
           #Reduce brightness gradually
            b = int(b - b/j)
            pixels.brightness = b/100
            pixels.show()
            time.sleep(wait)
        #Wait a bit
        time.sleep(wait*30) 

        #Increase brightness gradually
        for j in range(1, 100, 1):
            if self.stop_event.is_set():
                break
            b = int(b + b/j)
            pixels.brightness = b/100
            pixels.show()
            time.sleep(wait)
        #Wait a bit
        time.sleep(wait*10)


    def stop(self):
        self.stop_event.set()

    def __str__(self) -> str:
        return self.effect

def start_effect(effect, color=None, do_not_change_effect=False):
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
    elif effect == "breathe":
        effect_thread = EffectThread(effect)
        effect_thread.start()

def stop_effect():
    global effect_thread
    if str(effect_thread) == "breathe":
        print("Stopping effect")
        effect_thread.stop()
        print("Waiting for thread to stop")
        effect_thread.join()
        print("Thread stopped")
        effect_thread = None

def stop_all():
    global effect_thread
    stop_effect()
    pixels.fill((0,0,0))
    pixels.show()
   #Stop neopixel
    pixels.deinit() 
    print("Neopixel stopped")
