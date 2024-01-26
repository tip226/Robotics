# Red detection with LED - By: Tina Pham - Fri Jan 26 2024

import sensor, image, time, math, pyb

# Import the pyb module for LED control
red_led = pyb.LED(1)

red_threshold = (30, 100, 15, 127, 15, 127)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    red_detected = False

    for blob in img.find_blobs([red_threshold], pixels_threshold=100, area_threshold=100):
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            red_detected = True

    if red_detected:
        red_led.on()
    else:
        red_led.off()

    print(clock.fps())
