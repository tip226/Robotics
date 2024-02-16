import sensor, image, time
from pyb import Pin

# Initialize pins for LED control
pin_red = Pin("P2", Pin.OUT_PP, Pin.PULL_NONE)
pin_blue = Pin("P3", Pin.OUT_PP, Pin.PULL_NONE)

# Color detection thresholds
red_threshold = (16, 100, 9, 58, -33, 47)
blue_threshold = (7, 74, -20, 59, -59, -22)

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

while(True):
    img = sensor.snapshot()
    red_blobs = img.find_blobs([red_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    blue_blobs = img.find_blobs([blue_threshold], pixels_threshold=100, area_threshold=100, merge=True)

    # Control LEDs based on detected colors
    if red_blobs and not blue_blobs:
        # Detected red only
        pin_red.high()
        pin_blue.low()
    elif blue_blobs and not red_blobs:
        # Detected blue only
        pin_red.low()
        pin_blue.high()
    elif red_blobs and blue_blobs:
        # Detected both red and blue
        pin_red.high()
        pin_blue.high()
    else:
        # No color detected
        pin_red.low()
        pin_blue.low()
