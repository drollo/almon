import serial
from w1thermsensor import W1ThermSensor
import RPi.GPIO as GPIO
import time

PWR_MON_PIN = 17
PWR_ON = 0
PWR_OFF = 1

TEMP_NORMAL = 0
TEMP_LOW = 1
TEMP_LIMIT = 5

MSG_COOLDOWN_TIME = 3600
POLL_TIME = 5
STATE_CHANGE_THRESHOLD = 5

PWR_DOWN_MSG = "Alpon virta poikki!"
PWR_UP_MSG = "Alpon virta palasi"

contact_nbr = "0443055904"

sensor = W1ThermSensor()
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWR_MON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwr_state = PWR_ON
pwr_state_change_cnt = 0
pwr_up_msg_sent = True
pwr_down_msg_sent = False
pwr_down_msg_cooldown = MSG_COOLDOWN_TIME / POLL_TIME

temp_state = TEMP_NORMAL
statestate_change_cnt = 0
temp_low_msg_sent = False
temp_low_msg_cooldown = MSG_COOLDOWN_TIME / POLL_TIME

def almo_debug(string):
    print string

def sms_receive_all(device='/dev/ttyUSB0'):
    OK_cnt = 0
    lines = []

    ser = serial.Serial(device, timeout=5)

    ser.write('AT+CMGF=1\r')
    ser.write('AT+CMGL="ALL"\r')

    while OK_cnt < 2:
        line = ser.readline()
        if line == 'OK\r\n':
            OK_cnt += 1
        elif OK_cnt == 1 and line != '\r\n':
            lines.append(line.replace('\r\n',''))

    ser.close()

    return lines

def sms_send(number, message, device='/dev/ttyUSB0'):
    ser = serial.Serial(device, timeout=5)

    ser.write('AT+CMGF=1\r')
    ser.write('AT+CMGS="%s"\r' % number)
    ser.write('%s\x1a' % message)

    ser.close()

def pwr_check():
    global pwr_state
    global pwr_up_msg_sent
    global pwr_down_msg_sent
    global pwr_down_msg_cooldown
    global pwr_state_change_cnt

    if GPIO.input(PWR_MON_PIN) == PWR_OFF:
        almo_debug("Power is down")

        if pwr_state == PWR_ON:
            pwr_state = PWR_OFF
            pwr_state_change_cnt = 0

        if pwr_state_change_cnt < STATE_CHANGE_THRESHOLD:
            pwr_state_change_cnt += 1
        else:
            if pwr_down_msg_sent == False:
                sms_send(contact_nbr, PWR_DOWN_MSG)
                pwr_down_msg_sent = True
                pwr_up_msg_sent = False
                pwr_down_msg_cooldown = MSG_COOLDOWN_TIME / POLL_TIME
            else:
                if pwr_down_msg_cooldown == 0:
                    pwr_down_msg_sent = False
                else:
                    pwr_down_msg_cooldown -= 1
    else:
        almo_debug("Power is up")

        if pwr_state == PWR_OFF:
            pwr_state = PWR_ON
            pwr_state_change_cnt = 0

        if pwr_state_change_cnt < 5:
            pwr_state_change_cnt += 1
        else:
            if pwr_up_msg_sent == False:
                sms_send(contact_nbr, PWR_UP_MSG)
                pwr_up_msg_sent = True


def temp_check():
    global temp_state
    global temp_low_msg_sent
    global temp_low_msg_cooldown

    temp = sensor.get_temperature()

    almo_debug("Current temp: %d" % temp)

    if temp < TEMP_LIMIT:
        if temp_state == TEMP_NORMAL:
            temp_state = TEMP_LOW
            temp_state_change_cnt = 0

        if temp_state_chagne_cnt < STATE_CHANGE_THRESHOLD:
            temp_state_change_cnt += 1
        else:
            if temp_low_msg_sent == False:
                sms_send(contact_nbr, TEMP_LOW_MSG)
                temp_low_msg_sent = True
                temp_low_msg_cooldown = MSG_COOLDOWN_TIME / POLL_TIME
            else:
                if temp_low_msg_cooldown == 0:
                    temp_low_msg_sent = False
                else:
                    temp_low_msg_cooldown -= 1
    else:
        temp_state = TEMP_NORMAL

while 1:
    pwr_check()
    temp_check()
    time.sleep(POLL_TIME)
