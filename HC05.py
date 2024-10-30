from pyfirmata import Arduino
import time

board = Arduino('COM5')

ena_l = board.get_pin('d:3:p')
in1_l = board.get_pin('d:4:o')
in2_l = board.get_pin('d:5:o')

ena_r = board.get_pin('d:6:p')
in1_r = board.get_pin('d:7:o')
in2_r = board.get_pin('d:8:o')

l_speeds = []
r_speeds = []

SPEED_FACTOR = 1.5
TIME_FACTOR = 0.7

delta = None

# def f(x):
#     if(x <= 0.2):
#         y = 0
#     elif(0.2 < x <= 0.3):
#         y = 66.1 * (x - 0.3) + 6.61
#     elif(0.3 < x <= 0.4):
#         y = 531.9 * (x - 0.4) + 59.8
#     elif(0.4 < x <= 0.5):
#         y = 1502.15667 * (x - 0.5) + 210.016
#     elif(0.5 < x <= 0.6):
#         y = 1097.491667 * (x - 0.6) + 319.765
#     elif(0.6 < x <= 0.7):
#         y = 908.261667 * (x - 0.7) + 410.591
#     elif(0.7 < x <= 0.8):
#         y = 866.14 * (x - 0.8) + 497.205
#     elif(0.8 < x <= 0.9):
#         y = 403.225 * (x - 0.9) + 537.528
#     elif(0.9 < x <= 1):
#         y = 1224.491667 * (x - 1) + 659.977
#     else: 
#         y = 1

#     return y

def inverse_f(y):
    if(y == 0):
        x = 0
    elif(0 < y <= 6.61):
        x = 10/661 * (y - 6.61) + 0.3
    elif(6.61 < y <= 59.8):
        x = 10/5319 * (y - 59.8) + 0.4
    elif(59.8 < y <= 210.016):
        x = 300/450647 * (y - 210.016) + 0.5
    elif(210.016 < y <= 319.765):
        x = 120/131699 * (y - 319.765) + 0.6
    elif(319.765 < y <= 410.591):
        x = 600/544957 * (y - 410.591) + 0.7
    elif(410.591 < y <= 497.205):
        x = 0.00115454776364 * (y - 497.205) + 0.8
    elif(497.205 < y <= 537.528):
        x = 0.00248000496001 * (y - 537.528) + 0.9
    elif(537.528 < y <= 659.977):
        x = 0.000816665418983 * (y - 659.977) + 1
    else:
        x = 1

    return x

def speed_curve(speed):
    if speed >= 0:
        negative_factor = 1
    else:
        negative_factor = -1

    curved_speed = inverse_f(abs(speed))
    
    return curved_speed * negative_factor

def save_movements(left_speed, right_speed, delta_time):
    global delta, l_speeds, r_speeds

    speed_l = speed_curve(left_speed*SPEED_FACTOR)
    speed_r = speed_curve(right_speed*SPEED_FACTOR)
    
    l_speeds.append(speed_l)
    r_speeds.append(speed_r)
    
    delta = delta_time

def start_movements():
    global l_speeds, r_speeds
    for i in range(len(l_speeds)):
        move_motors(l_speeds[i], r_speeds[i])
        print(f"L: {l_speeds[i]} R: {r_speeds[i]}")
        time.sleep(delta*TIME_FACTOR)

    l_speeds = []
    r_speeds = []

    move_motors(0,0)

def move_motors(speed_l, speed_r):

    ena_r.write(abs(speed_r))
    if(speed_r == 0):
        in1_r.write(0)
        in2_r.write(0)
    elif(speed_r < 0):
        in1_r.write(1)
        in2_r.write(0)
    else:
        in1_r.write(0)
        in2_r.write(1)

    ena_l.write(abs(speed_l))
    if(speed_l == 0):
        in1_l.write(0)
        in2_l.write(0)
    elif(speed_l > 0):
        in1_l.write(0)
        in2_l.write(1)
    else:
        in1_l.write(1)
        in2_l.write(0)

move_motors(-0.4,-0.4)

time.sleep(1)

move_motors(0,0)