"""
Author       : Hanqing Qi & Karen Li & Jiawei Xu
Date         : 2023-10-20 17:16:42
LastEditors  : Jiawei Xu
LastEditTime : 2023-10-27 0:38:54
FilePath     :
Description  : Send the blob detection data (cx, cy, w, h) to the esp32
"""

import sensor, image, time
import omv
from pyb import LED
from pyb import UART
from machine import I2C
from machine import Pin
from vl53l1x import VL53L1X
import mjpeg, pyb
import random
import math


class Tracking_ROI:
    """ class Tracking_ROI:
        A square tracking window class that takes in new detection
        bounding box and adjust ROI for next detection
    """
    def __init__(self, x0=0, y0=0,
                 max_w=240, max_h=160,
                 min_windowsize=20,
                 forgetting_factor=0.5):
        self.roi = [x0, y0, max_w, max_h]
        self.x0 = x0
        self.y0 = y0
        self.max_w = max_w
        self.max_h = max_h
        self.min_windowsize = min_windowsize
        self.ff = forgetting_factor


    def update(self, detection=False, x=None, y=None, w=None, h=None):
        if detection == False:
            # failed detection result in maximum tracking box
            self.roi[0] = (1 - self.ff)*self.roi[0] + self.ff*self.x0
            self.roi[1] = (1 - self.ff)*self.roi[1] + self.ff*self.y0
            self.roi[2] = (1 - self.ff)*self.roi[2] + self.ff*self.max_w
            self.roi[3] = (1 - self.ff)*self.roi[3] + self.ff*self.max_h
        else:
            if x == None:
                return
            else:
                xx = x - 0.15*w
                yy = y - 0.15*h
                ww = 1.3*w
                hh = 1.3*h
                self.roi[0] = (1 - self.ff)*self.roi[0] + self.ff*xx
                self.roi[1] = (1 - self.ff)*self.roi[1] + self.ff*yy
                self.roi[2] = (1 - self.ff)*self.roi[2] + self.ff*ww
                self.roi[3] = (1 - self.ff)*self.roi[3] + self.ff*hh
        # corner case
        if self.roi[0] < self.x0:
            self.roi[0] = self.x0
        if self.roi[1] < self.y0:
            self.roi[1] = self.y0
        if self.roi[0] + self.roi[2] > self.max_w:
            self.roi[2] = self.max_w - self.roi[0]
        if self.roi[1] + self.roi[3] > self.max_h:
            self.roi[3] = self.max_h - self.roi[1]


    def reset(self):
        self.roi = [self.x0, self.y0, self.max_w, self.max_h]

    def get_roi(self):
        return [int(self.roi[i]) for i in range(4)]


class TrackedBlob:
    """ TrackedBlob class:
        An advanced class that tracks a colored blob based on a feature vector of 5 values:
            center x, center y, bbox width, bbox height, rotation angle

        It has a window to compute the feature distance for enhanced smoothness
    """
    def __init__(self, init_blob, norm_level: int,
                       feature_dist_threshold=100,
                       window_size = 3, blob_id=0):
        self.blob_history = [init_blob]
        self.feature_vector = [init_blob.x(),
                               init_blob.y(),
                               init_blob.w(),
                               init_blob.h(),
                               init_blob.rotation_deg()]

        self.norm_level = norm_level
        self.untracked_frames = 0
        self.feature_dist_threshold = feature_dist_threshold
        self.window_size = window_size
        self.id = blob_id

    def reset(self):
        """ Reset the tracker by empty the blob_history and feature vector while
            keeping the parameters
        """
        self.blob_history = None
        self.feature_vector = None

    def reinit(self, blob):
        """ reinitialize a reset blob by populate its history list and
            feature vector with a new blob
        """
        self.blob_history = [blob]
        self.feature_vector = [blob.x(),
                               blob.y(),
                               blob.w(),
                               blob.h(),
                               blob.rotation_deg()]
        self.untracked_frames = 0

    def compare(self, new_blob):
        """ Compare a new blob with a tracked blob in terms of
            their feature vector distance
        """
        feature = (new_blob.x(),
                   new_blob.y(),
                   new_blob.w(),
                   new_blob.h(),
                   new_blob.rotation_deg())
        my_feature = self.feature_vector
        if not new_blob.code() == self.blob_history[-1].code():
            # Different colors automatically grant a maimum distance
            return 32767
        elif self.norm_level == 1:
            return (math.fabs(feature[0]-my_feature[0]) +
                    math.fabs(feature[1]-my_feature[1]) +
                    math.fabs(feature[2]-my_feature[2]) +
                    math.fabs(feature[3]-my_feature[3]) +
                    math.fabs(feature[4]-my_feature[4]))
        else:
            return math.sqrt((feature[0]-my_feature[0])**2 +
                             (feature[1]-my_feature[1])**2 +
                             (feature[2]-my_feature[2])**2 +
                             (feature[3]-my_feature[3])**2 +
                             (feature[4]-my_feature[4])**2)


    def update(self, blobs):
        """ Update a tracked blob with a list of new blobs in terms of their feature distance.
            Upon a new candidate blob, we update the tracking history based on whether the
            histroy list is already filled or not
        """
        if blobs is None:
            # auto fail if None is fed
            self.untracked_frames += 1
            return None

        min_dist = 32767
        candidate_blob = None
        for b in blobs:
            # find the blob with minimum feature distance
            dist = self.compare(b)
            if dist < min_dist:
                min_dist = dist
                candidate_blob = b

        if min_dist < self.feature_dist_threshold:
            # update the feature history if the feature distance is below the threshold
            self.untracked_frames = 0
            # print("Successful Update! Distance: {}".format(min_dist))
            history_size = len(self.blob_history)
            self.blob_history.append(candidate_blob)
            feature = (candidate_blob.x(),
                       candidate_blob.y(),
                       candidate_blob.w(),
                       candidate_blob.h(),
                       candidate_blob.rotation_deg())

            if history_size <  self.window_size:
                # populate the history list if the number of history blobs is below the
                # window size
                for i in range(5):
                    # calculate the moving average
                    self.feature_vector[i] = (self.feature_vector[i]*history_size +
                        feature[i])/(history_size + 1)
            else:
                # O.W. pop the oldest and push a new one
                oldest_blob = self.blob_history[0]
                oldest_feature = (oldest_blob.x(),
                                  oldest_blob.y(),
                                  oldest_blob.w(),
                                  oldest_blob.h(),
                                  oldest_blob.rotation_deg())
                for i in range(5):
                    self.feature_vector[i] = (self.feature_vector[i]*self.window_size +
                        feature[i] - oldest_feature[i])/self.window_size
                self.blob_history.pop(0)
            return candidate_blob.rect()
        else:
            self.untracked_frames += 1
            return None


class Tracker:
    """ Base class for blob and goal tracker
    """
    def __init__(self, tracked_blob: TrackedBlob, thresholds, clock, show=True):
        self.tracked_blob = tracked_blob
        self.original_thresholds = [threshold for threshold in thresholds]
        self.current_thresholds = [threshold for threshold in thresholds]
        self.clock = clock
        self.show = show
        self.roi = Tracking_ROI(forgetting_factor=0.1)

    def track(self):
        pass


class BlobTracker(Tracker):
    """ BlobTracker class that initializes with a single TrackedBlob
        and tracks it with dynamic threshold
        TODO: track multiple blobs
    """
    def track(self):
        """ Detect blobs with tracking capabilities
            :input: tracked_blob: a TrackedBlob class object
                    thresholds: the list of color thresholds we want to track
                    show: True if we want to visualize the tracked blobs
                    clock: clock
        """
        # initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = find_reference(self.clock,
                                                        self.original_thresholds,
                                                        time_show_us=0)
            blue_led.on()
            self.tracked_blob.reinit(reference_blob)
            # update the adaptive threshold
            new_threshold = comp_new_threshold(statistics, 2.0)
            for i in range(len(self.current_thresholds)):
                self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                THRESHOLD_UPDATE_RATE)

            # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
            self.roi.update(True, self.tracked_blob.feature_vector[0],
                            self.tracked_blob.feature_vector[1],
                            self.tracked_blob.feature_vector[2],
                            self.tracked_blob.feature_vector[3])
            return self.tracked_blob.feature_vector, True
        else:
            # O.W. update the blob
            img = sensor.snapshot()
            self.clock.tick()
            blobs = img.find_blobs(self.current_thresholds, merge=True,
                                   pixels_threshold=75,
                                   area_threshold=100,
                                   margin=20,
                                   roi=self.roi.get_roi(),
                                   x_stride=1,
                                   y_stride=1)
            blue_led.on()
            roi = self.tracked_blob.update(blobs)

            if self.tracked_blob.untracked_frames >= 15:
                # if the blob fails to track for 15 frames, reset the tracking
                red_led.off()
                green_led.on()
                self.tracked_blob.reset()
                # self.roi.reset()
                blue_led.off()
                print("boom!")
                self.current_thresholds = [threshold for threshold in self.original_thresholds]
                return None, False
            else:
                if roi:
                    green_led.off()
                    red_led.off()
                    self.roi.update(True, roi[0], roi[1], roi[2], roi[3])
                    statistics = img.get_statistics(roi=roi)
                    new_threshold = comp_new_threshold(statistics, 3.0)
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                        new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                        THRESHOLD_UPDATE_RATE)
                else:
                    green_led.off()
                    red_led.on()
                    self.roi.update()
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.original_thresholds[i],
                                                                       self.current_thresholds[i])
                # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
                if self.show:
                    x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
                    img.draw_rectangle(x0, y0, w, h)
                    img.draw_rectangle(self.roi.get_roi(), color=(255, 255, 0))
                    st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
                    img.draw_string(0, 0, st, color = (255,0,0))
                return self.tracked_blob.feature_vector, True


class GoalTracker(Tracker):
    """ GoalTracker class that initializes with a single TrackedBlob and tracks it
        with dynamic threshold and ROI, the track function is specific for targets
        which involve turning LEDs on and off
        TODO: track multiple blobs
    """
    def track(self, edge_removal=True):
        """ Detect blobs with tracking capabilities
            :input: tracked_blob: a TrackedBlob class object
                    thresholds: the list of color thresholds we want to track
                    show: True if we want to visualize the tracked blobs
                    clock: clock
        """
        # initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = find_reference(self.clock,
                                                        self.original_thresholds,
                                                        time_show_us=0,
                                                        blink=True)
            blue_led.on()
            self.tracked_blob.reinit(reference_blob)
            # update the adaptive threshold
            new_threshold = comp_new_threshold(statistics, 2.0)
            for i in range(len(self.current_thresholds)):
                self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                THRESHOLD_UPDATE_RATE)

            # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
            self.roi.update(True, self.tracked_blob.feature_vector[0],
                            self.tracked_blob.feature_vector[1],
                            self.tracked_blob.feature_vector[2],
                            self.tracked_blob.feature_vector[3])
            return self.tracked_blob.feature_vector, True
        else:
            # O.W. update the blob
            blue_led.on()
            img, blobs = goal_blob_detection(self.current_thresholds, edge_removal=edge_removal)
            roi = self.tracked_blob.update(blobs)

            if self.tracked_blob.untracked_frames >= 15:
                # if the blob fails to track for 15 frames, reset the tracking
                red_led.off()
                green_led.on()
                self.tracked_blob.reset()
                # self.roi.reset()
                blue_led.off()
                print("boom!")
                self.current_thresholds = [threshold for threshold in self.original_thresholds]
                return None, False
            else:
                if roi:
                    green_led.off()
                    red_led.off()
                    self.roi.update(True, roi[0], roi[1], roi[2], roi[3])
                    statistics = img.get_statistics(roi=roi)
                    new_threshold = comp_new_threshold(statistics, 3.0)
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                        new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                        THRESHOLD_UPDATE_RATE)
                else:
                    green_led.off()
                    red_led.on()
                    self.roi.update()
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.original_thresholds[i],
                                                                       self.current_thresholds[i])
                # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
                if self.show:
                    x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
                    img.draw_rectangle(x0, y0, w, h, color=(255, 0, 0))
                    img.draw_rectangle(self.roi.get_roi(), color=(128, 128, 0))
                    st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
                    img.draw_string(0, 0, st, color = (0,0,0))
                    img.flush()
                return self.tracked_blob.feature_vector, True


def hold_up_for_sensor_refresh(last_time_stamp, wait_time) -> None:
    """
    description: wait for the sensor for some time from the
                 last snapshot to avoid a partial new image
    return  {*}: None
    """
    elapsed = wait_time - (int((time.time_ns() - last_time_stamp)/1000))
    if elapsed > 0:
        time.sleep_us(elapsed)

    return None


def goal_blob_detection(goal_thresholds, isColored=False, edge_removal=True):
    """ Detecting retroreflective goals with a blinking IR LED
    """
    omv.disable_fb(True) # no show on screen

    # get an extra frame buffer and take a snapshot
    if isColored:
        extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    else:
        extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
    extra_fb.replace(sensor.snapshot())

    # turn on the LED
    led_pin.value(1)
    time_last_snapshot = time.time_ns() # wait for the sensor to capture a new image
    # time block 1:
    # Do something other than wait, preferrably detection filtering and tracking
    hold_up_for_sensor_refresh(time_last_snapshot, WAIT_TIME_US)
    img = sensor.snapshot()

    # turn off the LED
    led_pin.value(0)
    time_last_snapshot = time.time_ns()

    # time block 2:
    # Do something other than wait, preferrably raw detection
    img.sub(extra_fb, reverse = False)

    # remove the edge noises
    edge_mask = None
    if edge_removal:
        if isColored:
            extra_fb.to_grayscale().find_edges(image.EDGE_SIMPLE)
        else:
            extra_fb.find_edges(image.EDGE_SIMPLE)
        edge_mask = extra_fb.dilate(3, 3).negate()

    img.negate()
    blobs = blobs = img.find_blobs(goal_thresholds,
                                   area_threshold=40,
                                   pixels_threshold=20,
                                   margin=10,
                                   merge=True,
                                   mask=edge_mask)
    sensor.dealloc_extra_fb()
    omv.disable_fb(False)
    img.flush()
    hold_up_for_sensor_refresh(time_last_snapshot, WAIT_TIME_US)
    return img, blobs


def blob_tracking(reference_blob,
                  thresholds,
                  clock,
                  blob_type=1,
                  norm_level=1,
                  feature_dist_threshold=200):
    """ The blob tracker initialization for balloons
        blob_type=1 for balloons
        blob_type=2 for goals
    """
    tracked_blob = TrackedBlob(reference_blob,
                               norm_level=norm_level,
                               feature_dist_threshold=feature_dist_threshold)
    if blob_type == 1:
        blob_tracker = BlobTracker(tracked_blob, thresholds, clock)
    elif blob_type == 2:
        blob_tracker = GoalTracker(tracked_blob, thresholds, clock)
    else:
        exit(1)
    return blob_tracker


def init_sensor_target(isColored=True, framesize=sensor.HQVGA, windowsize=None) -> None:
    sensor.reset()                        # Initialize the camera sensor.
    if isColored:
        sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
    else:
        sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(framesize)
    if windowsize is not None:            # Set windowing to reduce the resolution of the image
        sensor.set_windowing(windowsize)
    # sensor.skip_frames(time=1000)         # Let new settings take affect.
    sensor.set_auto_whitebal(False) # keep the color the same regardless of position, fixed color throughout, can change threshold without worrying about light env
    sensor.set_auto_exposure(False) # time each pixel is exposed to the light, how much energy. proportional to the brightness
    sensor.set_auto_gain(False) # multiply by a number to get x times brighter, higher gain will introduce noise, doesn't get brighter or darker when setting to F
#    sensor.__write_reg(0xad, 0b01001100) # R ratio
#    sensor.__write_reg(0xae, 0b01010100) # G ratio
#    sensor.__write_reg(0xaf, 0b01101000) # B ratio
    # RGB gains
#    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
#    sensor.__write_reg(0x80, 0b10111100) # enable gamma, CC, edge enhancer, interpolation, de-noise
#    sensor.__write_reg(0x81, 0b01101100) # enable BLK dither mode, low light Y stretch, autogray enable
#    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
#    sensor.__write_reg(0x03, 0b00000010) # high bits of exposure control
#    sensor.__write_reg(0x04, 0b11110000) # low bits of exposure control
#    sensor.__write_reg(0xb0, 0b11100000) # global gain

#    # RGB gains
#    sensor.__write_reg(0xa3, 0b01110000) # G gain odd
#    sensor.__write_reg(0xa4, 0b01110000) # G gain even
#    sensor.__write_reg(0xa5, 0b10000000) # R gain odd
#    sensor.__write_reg(0xa6, 0b10000000) # R gain even
#    sensor.__write_reg(0xa7, 0b10000000) # B gain odd
#    sensor.__write_reg(0xa8, 0b10000000) # B gain even
#    sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
#    sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
#    sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
#    # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
#                                           # strangely constrained by auto saturation
#    sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
#    sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
#    sensor.__write_reg(0xd3, 0b01001000) # luma contrast
    # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    # sensor.skip_frames(time=2000) # Let the camera adjust.


def draw_initial_blob(img, blob, sleep_us=500000) -> None:
    """ Draw initial blob and pause for sleep_us for visualization
    """
    if not blob or sleep_us < 41000:
        # No need to show anything if we do not want to show
        # it beyond human's 24fps classy eyes' capability
        return None
    else:
        img.draw_edges(blob.min_corners(), color=(255,0,0))
        img.draw_line(blob.major_axis_line(), color=(0,255,0))
        img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
        # sleep for 500ms for initial blob debut
        time.sleep_us(sleep_us)


def find_max(blobs):
    """ Find maximum blob in a list of blobs
        :input: a list of blobs
        :return: Blob with the maximum area,
                 None if an empty list is passed
    """
    max_blob = None
    max_area = 0
    for blob in blobs:
        if blob.area() > max_area:
            max_blob = blob
            max_area = blob.pixels()
    return max_blob


def comp_new_threshold(statistics, mul_stdev=2):
    """ Generating new thresholds based on detection statistics
        l_low = l_mean - mul_stdev*l_stdev
        l_high = l_mean + mul_stdev*l_stdev
        a_low = a_mean - mul_stdev*a_stdev
        a_high = a_mean - mul_stdev*a_stdev
        b_low = b_mean - mul_stdev*b_stdev
        b_high = b_mean - mul_stdev*b_stdev
    """
    l_mean = statistics.l_mean()
    l_stdev = statistics.l_stdev()
    a_mean = statistics.a_mean()
    a_stdev = statistics.a_stdev()
    b_mean = statistics.b_mean()
    b_stdev = statistics.b_stdev()
    new_threshold = (l_mean - mul_stdev*l_stdev, l_mean + mul_stdev*l_stdev,
                     a_mean - mul_stdev*a_stdev, a_mean - mul_stdev*a_stdev,
                     b_mean - mul_stdev*b_stdev, b_mean - mul_stdev*b_stdev)
    return new_threshold


def comp_weighted_avg(vec1, vec2, w1=0.5, w2=0.5):
    """ Weighted average, by default just normal average
    """
    avg = [int(w1*vec1[i] + w2*vec2[i]) for i in range(len(vec1))]
    return tuple(avg)


def one_norm_dist(v1, v2):
    # 1-norm distance between two vectors
    return sum([abs(v1[i] - v2[i]) for i in range(len(v1))])


def two_norm_dist(v1, v2):
    # 2-norm distance between two vectors
    return math.sqrt(sum([(v1[i] - v2[i])**2 for i in range(len(v1))]))


def find_reference(clock, thresholds,
                   density_threshold=0.25,
                   roundness_threshold=0.35,
                   time_show_us=50000,
                   blink=False):
    """ Find a reference blob that is dense and round,
        also return the color statistics in the bounding box
    """
    biggest_blob = None
    while not biggest_blob:
        blob_list = []
        clock.tick()
        if blink:
            img, blob_list = goal_blob_detection(thresholds)
        else:
            img = sensor.snapshot()
            b_blobs = img.find_blobs(thresholds, merge=True,
                                     pixels_threshold=30,
                                     area_threshold=50,
                                     margin=20,
                                     x_stride=1,
                                     y_stride=1)
            for blob in b_blobs:
                # find a good initial blob by filtering out the not-so-dense and not-so-round blobs
                if (blob.density() > density_threshold and
                    blob.roundness() > roundness_threshold):
                    blob_list.append(blob)
        biggest_blob = find_max(blob_list)

    draw_initial_blob(img, biggest_blob, time_show_us)
    statistics = img.get_statistics(roi=biggest_blob.rect())
    return biggest_blob, statistics


def checksum(arr, initial= 0):
    """ The last pair of byte is the checksum on iBus
    """
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB


def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]

    # Perform the checksume
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB
    return msg


def mode_initialization(input_mode, mode):
    """ Switching between blinking goal tracker and balloon tracker
    """
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(isColored=True)
            thresholds = GREEN
            reference_blob, statistics = find_reference(clock, thresholds, blink=False)
            tracker = blob_tracking(reference_blob, thresholds, clock,
                                    blob_type=1, feature_dist_threshold=300)
        elif input_mode == 1:
            init_sensor_target(isColored=False)
            # Find reference
            thresholds = GRAY
            reference_blob, statistics = find_reference(clock, thresholds,
                                                        roundness_threshold=0.55,
                                                        blink=True)
            tracker = blob_tracking(reference_blob, thresholds, clock,
                                    blob_type=2, feature_dist_threshold=200)

        return input_mode, tracker



if __name__ == "__main__":
    ### Macros
    GREEN = [(27, 94, -47, -12, 6, 19)]
    PURPLE = [(35, 52, -8, 10, -33, -3)]
    GRAY = [(0, 20)]
    THRESHOLD_UPDATE_RATE = 0.0
    WAIT_TIME_US = 50000
    ### End Macros

    led_pin = Pin("P2", Pin.OUT)
    led_pin.value(0)
    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)
    clock = time.clock()

    mode = 0

    # Initialize UART
    # uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)
    uart = UART(1, 19200, timeout_char=200)
    # Sensor initialization

    mode, tracker = mode_initialization(mode, -1)

    while True:
        print(mode)
        tracker.track()
        if tracker.tracked_blob.feature_vector:
            roi = tracker.roi.get_roi()
            feature_vec = tracker.tracked_blob.feature_vector
            x_value = roi[0] + roi[2]//2
            y_value = roi[1] + roi[3]//2
            w_value = int(feature_vec[2])
            h_value = int(feature_vec[3])
            msg = IBus_message([x_value, y_value, w_value, h_value])
        else:
            msg = IBus_message([0, 0, 0, 0])

        # send 32 byte message
        uart.write(msg)
        # receive 32 byte message
        # if random.random() > 0.9:
        #     res = mode_initialization(0, mode)
        #     if res:
        #         mode, tracker = res
        # elif random.random() < 0.1:
        #     res = mode_initialization(1, mode)
        #     if res:
        #         mode, tracker = res

        if uart.any():
            uart_input = uart.read()
            if uart_input == 0x80:
                res = mode_initialization(0, mode)
                if res:
                    mode, tracker = res
            elif uart_input == 0x81:
                res = mode_initialization(1, mode)
                if res:
                    mode, tracker = res

