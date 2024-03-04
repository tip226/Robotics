import sensor, image, time
from pyb import Pin
from pyb import UART

# UART setup
uart = UART(1, 19200, timeout_char=200)

# Color detection thresholds
red_threshold = (0, 46, 60, -1, -32, 29)
blue_threshold = (0, 61, 9, -58, -77, 15)

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

    # Send detected colors to ESP32S3
    if red_blobs and not blue_blobs:
        uart.write('R') # Detected red only
    elif blue_blobs and not red_blobs:
        uart.write('B') # Detected blue only
    elif red_blobs and blue_blobs:
        uart.write('RB') # Detected both red and blue
    else:
        uart.write('N') # No color detected
