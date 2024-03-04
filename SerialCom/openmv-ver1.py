import sensor, image, time
from pyb import Pin
from pyb import UART

# UART setup
uart = UART(1, 19200, timeout_char=200)

# Color detection thresholds
red_threshold = (25, 51, 31, 58, -6, 24)
blue_threshold = (16, 30, -16, -1, -17, -3)

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

    while(True):
        img = sensor.snapshot()
        # Combined detection of red and blue blobs
        blobs = img.find_blobs([red_threshold, blue_threshold], pixels_threshold=100, area_threshold=100, merge=True)

        detected_colors = ''
        for blob in blobs:
            if blob.code() & 1:  # Red blob detected
                img.draw_rectangle(blob.rect(), (255, 0, 0))
                detected_colors += 'R'
            if blob.code() & 2:  # Blue blob detected
                img.draw_rectangle(blob.rect(), (0, 0, 255))
                detected_colors += 'B'

        # Send detected colors to ESP32S3
        if detected_colors:
            uart.write(detected_colors)  # Send detected colors
            print(detected_colors)
        else:
            uart.write('N')  # No color detected
            print("Nothing")
