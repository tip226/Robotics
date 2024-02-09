# Red detection with LED - By: Tina Pham - Fri Jan 26 2024

import sensor, image, time, math, pyb

# Import the pyb module for LED control
red_led = pyb.LED(1)
blue_led = pyb.LED(3)

red_threshold = (34, 63, 52, 77, 5, 43)
blue_threshold = (7, 74, -20, 59, -59, -22)
#face_threshold = (35, 45, 20, 40, 10, 40)
#red_threshold = ((17, 35, 20, 20, -26, 10), (45, 63, 40, 77, 40, 67))

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_contrast(-3)
sensor.set_brightness(0)
sensor.set_saturation(3)
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([red_threshold, blue_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    red_detected = False
    blue_detected = False

    for blob in blobs:
        if blob.code() == 1:
            img.draw_rectangle(blob.rect(), color=(255, 0, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))
            red_detected = True
        elif blob.code() == 2:
            img.draw_rectangle(blob.rect(), color=(0, 0, 255))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
            blue_detected = True

    # Control LEDs based on detected colors
    if red_detected and not blue_detected:
        red_led.on()
        blue_led.off()
    elif blue_detected and not red_detected:
        blue_led.on()
        red_led.off()
    elif red_detected and blue_detected:
        # Turn on both LEDs to represent "pink"
        red_led.on()
        blue_led.on()
    else:
        red_led.off()
        blue_led.off()

    print(clock.fps())
