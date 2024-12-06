import sensor, image, time
from pid import PID
from machine import UART
import ustruct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
clock = time.clock()

uart = UART(3, 115200)

red_threshold = (40, 15, 21, 127, -128, 127)
yellow_threshold = (0, 100, -45, 10, -25, 19)
blue_threshold = (12, 68, -26, -2, -36, -7)
size_threshold = 1000
x_pid = PID(p=3, i=0, imax=80, d=0)
h_pid = PID(p=0.05, i=0, imax=50, d=0.00)


def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob


def send_data(v, w):
    global uart
    data = ustruct.pack("<bbffb", 0x2C, 0x12, v, w, 0x5B)

    try:
        uart.write(data)
    except BaseException:
        print("bad package")


while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([red_threshold])
    if blobs:
        max_blob = find_max(blobs)
        x_error = max_blob[5] - img.width() / 2
        #        print(max_blob[2]*max_blob[3])
        h_error = max_blob[2] * max_blob[3] - size_threshold

        img.draw_rectangle(max_blob[0:4])
        img.draw_cross(max_blob[5], max_blob[6])

        x_output = x_pid.get_pid(x_error, 1)
        h_output = h_pid.get_pid(h_error, 0.05, 0.8)
        print("x_output:", x_output, "x_e", x_error)
        print("h_output:", h_output)
        send_data(-h_output, x_output)
    else:
        print("no target")
