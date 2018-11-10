# coding=utf8
import serial
from w1thermsensor import W1ThermSensor
import RPi.GPIO as GPIO
import time
import os

class Sms:
    nbr = ""
    msg = ""
    read = False

PWR_MON_PIN = 17
PWR_ON = 0
PWR_OFF = 1

TEMP_NORMAL = 0
TEMP_LOW = 1
TEMP_LIMIT = 5

MSG_COOLDOWN = 1000
POLL_TIME = 5
SER_TIMEOUT = 5
STATE_CHANGE_THRESHOLD = 5

PWR_DOWN_MSG = "Alpon virta poikki!"
PWR_UP_MSG = "Alpon virta palasi"

contact_nbr = "0443055904"
sensor_names = ["ykkössensori"]

sensor_tbl = {}
sensor_idx = 0
for sensor in W1ThermSensor.get_available_sensors():
    if sensor_idx < len(sensor_names):
        sensor_name = sensor_names[sensor_idx]
    else:
        sensor_name = "sensor %s" % sensor.id
    sensor_tbl[sensor.id] = sensor_name
    sensor_idx += 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWR_MON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwr_state = PWR_ON
pwr_state_change_cnt = 0
pwr_up_msg_sent = True
pwr_down_msg_sent = False
pwr_down_msg_cooldown = MSG_COOLDOWN

temp_state = TEMP_NORMAL
statestate_change_cnt = 0
temp_low_msg_sent = False
temp_low_msg_cooldown = MSG_COOLDOWN

def almon_debug(string):
    print string

def ip_local_get():
    wlan0_found = False
    output = os.popen('ifconfig -a').read().split('\n')
    for line in output:
        if 'wlan0:' in line:
            wlan0_found = True
        elif wlan0_found == True and "inet" in line:
            return line.split(' ')[9]

def ip_public_get():
    parse_ip = False
    output = os.popen('host myip.opendns.com resolver1.opendns.com').read().split(' ')
    for line in output:
        if 'address' in line:
            parse_ip = True
        elif parse_ip == True:
            return line.split('\n')[0]

def sms_read_data(device='/dev/ttyUSB0'):
    ser = serial.Serial(device, timeout=SER_TIMEOUT)
    CMGL_found = False
    lines = []
    msg_data = []

    ser.write('AT+CMGF=1\r')
    ser.write('AT+CMGL="ALL"\r')

    lines = ser.readlines()
    for line in lines:
        if '+CMGL:' in line:
            msg_data.append(line.replace('\r\n', ''))
            CMGL_found = True
        elif CMGL_found == True and line != '\n\r':
            msg_data.append(line.replace('\r\n', ''))
            CMGL_found = False

    ser.close()

    return msg_data

def sms_receive_all():
    lines = sms_read_data()
    sms_list = []
    idx = 0
    for line in lines:
        if idx % 2 == 0:
            sms = Sms()
            sms.nbr = line.split('\"')[3]
            if 'REC READ' in line.split('"'):
                sms.read = True
            sms_list.append(sms)
        else:
            sms_list[-1].msg = line.replace('\"','')
        idx += 1

    return sms_list

def sms_delete_all_read(device='/dev/ttyUSB0'):
    ser = serial.Serial(device, timeout=SER_TIMEOUT)
    lines = sms_read_data()
    idx_list = []

    for line in lines:
        if '+CMGL:' in line and 'REC READ' in line.split('"'):
            almon_debug("delete message %s" % (line))
            sms_idx = ''.join(line.split(' ')[1]).split(',')[0]
            ser.write('AT+CMGD=%s\r' % (sms_idx))
            time.sleep(1)

    ser.close()

def sms_send(number, message, device='/dev/ttyUSB0'):
    ser = serial.Serial(device, timeout=SER_TIMEOUT)

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
        almon_debug("Power is down")

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
                pwr_down_msg_cooldown = MSG_COOLDOWN
            else:
                if pwr_down_msg_cooldown == 0:
                    pwr_down_msg_sent = False
                else:
                    pwr_down_msg_cooldown -= 1
    else:
        almon_debug("Power is up")

        if pwr_state == PWR_OFF:
            pwr_state = PWR_ON
            pwr_state_change_cnt = 0

        if pwr_state_change_cnt < 5:
            pwr_state_change_cnt += 1
        else:
            if pwr_up_msg_sent == False:
                sms_send(contact_nbr, PWR_UP_MSG)
                pwr_up_msg_sent = True

def temp_get_min():
    temp_min = 1000000
    for sensor in W1ThermSensor.get_available_sensors():
        temp = sensor.get_temperature()
        almon_debug("Current temp, %s: %d" % (sensor_tbl[sensor.id], temp))
        if temp < temp_min:
            temp_min = temp
            lowest_sensor = sensor_tbl[sensor.id]
    return temp_min, lowest_sensor

def temp_check():
    global temp_state
    global temp_low_msg_sent
    global temp_low_msg_cooldown

    temp, lowest_sensor = temp_get_min()
    almon_debug("Current lowest temp: %d, sensor: %s" % (temp, lowest_sensor))

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
                temp_low_msg_cooldown = MSG_COOLDOWN
            else:
                if temp_low_msg_cooldown == 0:
                    temp_low_msg_sent = False
                else:
                    temp_low_msg_cooldown -= 1
    else:
        temp_state = TEMP_NORMAL

while 1:
    sms_list = sms_receive_all()
    for sms in sms_list:
        almon_debug('received sms: %s, %s' % (sms.nbr, sms.msg))
        if sms.msg.lower() == "ping":
            ip_local = ip_local_get()
            ip_public = ip_public_get()
            almon_debug("send sms: %s pong %s %s" % (sms.nbr, ip_local, ip_public))
            sms_send(sms.nbr, "pong %s %s" % (ip_local, ip_public))
            time.sleep(1)

    sms_delete_all_read()
    pwr_check()
    temp_check()
    time.sleep(POLL_TIME)
