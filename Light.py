import lirc
import sys
import RPi.GPIO as GPIO
import time
import serial
import random
import threading
import smbus as smbus
import math
from rpi_ws281x import Color, PixelStrip, ws

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.cleanup()
# sockid = lirc.init("project.py")

# Variables
LED_COUNT = 60         # Number of LED pixels.
LED_PIN = 10           # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ = 800000   # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10           # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255   # Set to 0 for darkest and 255 for brightest
# True to invert the signal (when using NPN transistor level shift)
LED_INVERT = False
LED_CHANNEL = 0
LED_STRIP = ws.SK6812_STRIP_RGBW
HEIGHT = 80
currentMode = 0
In_Pin1 = 19
In_Pin2 = 20
LightSenser = 12
Lazer = 16
remote = 22
ser = serial.Serial("/dev/ttyAMA0", 9600)
a = '55 5A 02 C3 C1'
currentVerticalAngle = 90
currentHorizontalAngle = 90
recordAngle = []
recordDist = []
remoteControl = False
lightState = False
goBackTime = 0
originTime = 0
angle1=45
angle2=90
LED_num = 0
step1=4
step2=4
target1 = [90, 90]
target2 = [90, 90]
target3 = [90, 90]
direct = "RESTART"
address = 0x48
discomode = 0
A0 = 0x40
I2C = smbus.SMBus(1)
bus = smbus.SMBus(1)
bus.write_byte(address, A0)
Mode1 = False
Mode2 = False
Mode3 = False
Mode4 = False
Mode5 = False


# setGPIOMode
GPIO.setup(19, GPIO.OUT)
p1 = GPIO.PWM(In_Pin1, 50)
GPIO.setup(20, GPIO.OUT)
p2 = GPIO.PWM(In_Pin2, 50)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(22, GPIO.IN)
GPIO.setup(12, GPIO.IN)

# OpenPWM
p1.start(0)
p2.start(0)


def thread2():
    while True:
        ModeChange()


def pasreset(data):  # Parse keys, only a few major modes require multithreading
    global currentMode, currentHorizontalAngle, currentVerticalAngle, remoteControl, direct, Mode1, Mode2, Mode3, Mode4, Mode5, LED_num, discomode
    if data == 'echo "KEY_1"':
        if Mode1 == False:
            print("static OPEN")
            Mode1 = True
            p1.ChangeDutyCycle(2.5+90/360*20)
            p2.ChangeDutyCycle(2.5+90/360*20)
            currentMode = 1
            time.sleep(0.2)
        else:
            Mode1 = False
            LightsOff()
            print("static CLOSE")
            # colorWipe(strip, Color(0, 0, 0), 0)
            currentMode = 0
            time.sleep(0.2)
    elif data == 'echo "KEY_2"':
        if Mode2 == False:
            print("remoteControl OPEN")
            LightsOn()
            Mode2 = True
            currentMode = 2
        else:
            LightsOff()
            Mode2 = False
            # colorWipe(strip, Color(0, 0, 0), 0)
            print("remoteControl CLOSE")
            currentMode = 0
    elif data == 'echo "KEY_3"':

        if Mode3 == False:
            print("disco1 OPEN")
            LightsOn()
            Mode3 = True
            discomode = 2
            currentMode = 3
        elif Mode3 == True and discomode == 2:
            print("disco2 OPEN")
            LightsOn()
            Mode3 = True
            discomode = 1
            currentMode = 3
        else:
            print("disco CLOSE")
            LightsOff()
            Mode3 = False
            # colorWipe(strip, Color(0, 0, 0), 0)
            currentMode = 0
    elif data == 'echo "KEY_4"':
        if Mode4 == False:
            print("goBack OPEN")
            LightsOn()
            Mode4 = True
            currentMode = 4
        else:
            print("goBack CLOSE")
            LightsOff()
            Mode4 = False
            # colorWipe(strip, Color(0, 0, 0), 0)
            currentMode = 0
    elif data == 'echo "KEY_5"':
        if Mode5 == False:
            print("McRae OPEN")
            LightsOff()
            Mode5 = True
            currentMode = 5
        else:
            print("McRae CLOSE")
            LightsOff()
            Mode5 = False
            LED_num = 0
            # colorWipe(strip, Color(0, 0, 0), 0)
            time.sleep(0.1)
            currentMode = 0
    elif data == 'echo "KEY_UP"' and currentMode == 2:
        if currentHorizontalAngle >= 0 and currentHorizontalAngle <= 180:
            currentHorizontalAngle -= 10
            p2.ChangeDutyCycle(2.5+(currentHorizontalAngle)/360*20)
        direct = "UP"
        print("UP")
    elif data == 'echo "KEY_DOWN"' and currentMode == 2:
        if currentHorizontalAngle >= 0 and currentHorizontalAngle <= 180:
            currentHorizontalAngle += 10
            p2.ChangeDutyCycle(2.5+(currentHorizontalAngle)/360*20)
        direct = "DOWN"
        print("DOWN")
    elif data == 'echo "KEY_LEFT"' and currentMode == 2:
        if currentVerticalAngle >= 0 and currentVerticalAngle <= 180:
            currentVerticalAngle -= 10
            p1.ChangeDutyCycle(2.5+(currentVerticalAngle)/360*20)
        print("LEFT")
    elif data == 'echo "KEY_RIGHT"' and currentMode == 2:
        if currentVerticalAngle >= 0 and currentVerticalAngle <= 180:
            currentVerticalAngle += 10
            p1.ChangeDutyCycle(2.5+(currentVerticalAngle)/360*20)
        print("RIGHT")
    elif data == 'echo "KEY_RESTART"' and currentMode == 2:
        p1.ChangeDutyCycle(2.5+(90)/360*20)
        p2.ChangeDutyCycle(2.5+(90)/360*20)
        print("RESTART")


def staticMode():
    global a, currentHorizontalAngle, currentMode, originTime
    d = bytes.fromhex(a)
    if not ser.isOpen():
        print("open failed")

    try:
        ser.write(d)
        time.sleep(0.1)
        count = ser.inWaiting()
        if count > 0:
            recv = (ser.read(count)).hex()
            num = recv[9:10]
            if num != '0':
                originTime = 0
                GPIO.output(Lazer, GPIO.HIGH)
                targetAngle = '0x'+recv[20:22]

                velocity2 = '0x'+recv[16:20]
                targetDist1 = '0x'+recv[12:14]+'00'
                targetDist2 = '0x'+recv[14:16]
                a1 = int(targetAngle, 16)
                d2 = int(targetDist2, 16)
                d1 = int(targetDist1, 16)
                v2 = int(velocity2, 16)
                d = d1+d2
                if v2 > 32768:
                    v2 = 65536-v2
                print(v2)
                stepChase(strip, Color(23, 13+d, 0), d)
                if len(recordDist) < 200:
                    recordDist.append(d)
                else:
                    recordDist.clear()
                # transmit d to the LED
                if a1 > 90:
                    a1 = -(256-a1)
                targetAngle = 90+a1
                # targetAngle=150
                if v2 > 0 and v2 < 20:
                    velFactor = 3
                elif v2 >= 20 and v2 < 80:
                    velFactor = 5
                elif v2 > 80:
                    velFactor = 7
                else:
                    velFactor = 0
                if targetAngle > currentHorizontalAngle:
                    step = 1*velFactor
                elif targetAngle < currentHorizontalAngle:
                    step = -1*velFactor
                else:
                    step = 0
                currentHorizontalAngle = currentHorizontalAngle+step
                # print(recv)
                # print(currentHorizontalAngle)
                if len(recordAngle) < 200:
                    recordAngle.append(currentHorizontalAngle)
                else:
                    recordAngle.clear()
                p1.ChangeDutyCycle(2.5+currentHorizontalAngle/360*20)
            else:
                # gradually become default(incomplete)
                p1.ChangeDutyCycle(2.5+90/360*20)
                print("origine")
                originTime += 1
                if originTime == 30:
                    GPIO.output(Lazer, GPIO.LOW)
                    currentHorizontalAngle = 90
                    colorWipeSide2Mid(strip, Color(0, 0, 0))
                    originTime = 0
            time.sleep(0.1)

    except KeyboardInterrupt:
        if ser != None:
            ser.close()


def LightsOn():
    global lightState
    GPIO.output(Lazer, GPIO.HIGH)
    lightState = True


def LightsOff():
    global lightState
    GPIO.output(Lazer, GPIO.LOW)
    lightState = False


def discoMode1():
    global discomode,angle1,angle2,step1,step2
    
    if discomode == 1:
        LightsOn()
        num = bus.read_byte(0x48)
        lightvoice(strip, num)
        p2.ChangeDutyCycle(2.5+angle1/360*20)
        p1.ChangeDutyCycle(2.5+angle2/360*20)
        if angle1>135:
            step1=-4
        if angle1<45:
            step1=4
        if angle2>135:
            step2=-4
        if angle2<45:
            step2=4
        angle1=angle1+step1
        angle2=angle2+step2
        time.sleep(0.1)
    if discomode == 2:
        LightsOn()
        global currentHorizontalAngle, currentVerticalAngle
        currentHorizontalAngle = random.randint(45, 135)
        currentVerticalAngle = random.randint(45, 135)
        p2.ChangeDutyCycle(2.5+(currentHorizontalAngle)/360*20)
        p1.ChangeDutyCycle(2.5+(currentVerticalAngle)/360*20)
        rainbow(strip)
        time.sleep(0.05)


def goBackMode():
    global recordAngle, recordDist, goBackTime
    p2.ChangeDutyCycle(2.5+90/360*20)
    p1.ChangeDutyCycle(2.5+90/360*20)
    if goBackTime == 0:
        if len(recordAngle) != 0:
            for i in range(len(recordAngle)):
                p1.ChangeDutyCycle(2.5+recordAngle[i]/360*20)
                stepChase(strip, Color(20, 187, 0), recordDist[i])
                time.sleep(0.2)
            recordAngle.clear()
            recordDist.clear()
            print("Finish")
            goBackTime == 1


def McRaeMode():
    global LED_num, target1, target2, target3

    p1.ChangeDutyCycle(2.5+90/360*20)
    p2.ChangeDutyCycle(2.5+90/360*20)

    d = bytes.fromhex(a)
    if not ser.isOpen():
        print("It's High noon")
    try:
        ser.write(d)
        time.sleep(0.1)
        count = ser.inWaiting()
        t1 = time.time()
        if count > 0:
            recv = (ser.read(count)).hex()
            num = recv[9:10]
            if num == "0":
                computeRecv(recv)
                print("no target")
            elif num == "1":
                computeRecv(recv, target1)
            elif num == "2":
                computeRecv(recv, target1, target2)
            elif num == "3":
                computeRecv(recv, target1, target2, target3)
        if LED_num < 60:
            # One LED light at a time
            strip.setPixelColor(LED_num, Color(245-LED_num*4, 0+LED_num*4, 0))
            strip.show()
            time.sleep(0.05)
            LED_num += 1
        else:
            p1.ChangeDutyCycle(2.5+target1[0]/360*20)
            p2.ChangeDutyCycle(2.5+target1[1]/360*20)
            time.sleep(0.5)
            p1.ChangeDutyCycle(2.5+target2[0]/360*20)
            p2.ChangeDutyCycle(2.5+target2[1]/360*20)
            time.sleep(0.5)
            p1.ChangeDutyCycle(2.5+target3[0]/360*20)
            p2.ChangeDutyCycle(2.5+target3[1]/360*20)
            time.sleep(3)
            # default
            p1.ChangeDutyCycle(2.5+90/360*20)
            p2.ChangeDutyCycle(2.5+90/360*20)
            LED_num = 0
            target1 = [90, 90]
            target2 = [90, 90]
            target3 = [90, 90]
            print("Finish")
            LightsOff()
            Mode5 = False
            currentMode = 0

    except KeyboardInterrupt:
        if ser != None:
            ser.close()


def AutoOpen():
    global lightState
    colorWipe(strip, Color(0, 0, 0))
    if GPIO.input(12):
        # print("on")
        if not lightState:
            LightsOn()
    else:
        # print("off")
        if lightState:
            LightsOff()


def ModeChange():
    # global strip
    if currentMode == 0:
        # colorWipe(strip,Color(0,0,0))
        time.sleep(0.2)
        AutoOpen()
    elif currentMode == 1:
        staticMode()
    elif currentMode == 2:  # 只能做一个灯角度的遥控，是否有点单调，是否需要加其他功能
        LightsOn()
    elif currentMode == 3:  # 射灯的变化频率固定，才灯的变化或许也可以更丰富
        discoMode1()
    elif currentMode == 4:
        goBackMode()
    elif currentMode == 5:  # 可能角度需要再修正一下
        LightsOff()
        McRaeMode()


def lightvoice(strip, voice):
    num = voice-130
    if num<0:
        num=3
    if num > 180:
        num = 60
    # print(num)
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))
    for j in range(num):
        strip.setPixelColor(j, Color(0, 0+j*4, 245-j*4))
    strip.show()


def stepChase(strip, color, dist, wait_ms=1000):
    id = math.ceil(dist/(100/60))-20
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))
    for j in range(id, id+15):
        strip.setPixelColor(j, color)
    strip.show()


def colorWipe(strip, color, wait_ms=1):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()


def colorWipeSide2Mid(strip, color, wait_ms=10):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()-29):
        strip.setPixelColor(i, color)
        strip.setPixelColor(strip.numPixels()-i, color)
        strip.show()


def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)


def rainbow(strip, wait_ms=0.1, iterations=1):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256 * iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((i + j) & 255))
        strip.show()
        time.sleep(wait_ms / 1000.0)


def slowup(strip):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(245-i*4, 0+i*4, 0))
        strip.show()
        time.sleep(0.2)


def computeRecv(recv, target1=None, target2=None, target3=None):
    if target1:
        Angle1 = '0x'+recv[20:22]
        Dist1 = '0x'+recv[12:14]+'00'
        Dist2 = '0x'+recv[14:16]
        d2 = int(Dist2, 16)
        d1 = int(Dist1, 16)
        d = d1+d2
        print(d)
        # compute the pitch angle
        pitAngle = -math.atan(HEIGHT/d)*180+180
        print(pitAngle)
        a1 = int(Angle1, 16)
        if a1 > 90:
            a1 = -(256-a1)
        horzAngle = 90+a1
        target1[0] = horzAngle
        target1[1] = pitAngle
    else:
        target1 = [90, 90]
    if target2:
        Angle1 = '0x'+recv[36:38]
        Dist1 = '0x'+recv[28:30]+'00'
        Dist2 = '0x'+recv[30:32]
        d2 = int(Dist2, 16)
        d1 = int(Dist1, 16)
        d = d1+d2
        # compute the pitch angle
        pitAngle = -math.atan(HEIGHT/d)*180+180
        a1 = int(Angle1, 16)
        if a1 > 90:
            a1 = -(256-a1)
        horzAngle = 90+a1
        target2[0] = horzAngle
        target2[1] = pitAngle
    else:
        target2 = [90, 90]
    if target3:
        Angle1 = '0x'+recv[52:54]
        Dist1 = '0x'+recv[44:46]+'00'
        Dist2 = '0x'+recv[46:48]
        d2 = int(Dist2, 16)
        d1 = int(Dist1, 16)
        d = d1+d2
        # compute the pitch angle
        pitAngle = -math.atan(HEIGHT/d)*180+180
        a1 = int(Angle1, 16)
        if a1 > 90:
            a1 = -(256-a1)
        horzAngle = 90+a1
        target3[0] = horzAngle
        target3[1] = pitAngle
    else:
        target3 = [90, 90]


if __name__ == "__main__":
    t2 = threading.Thread(target=thread2)  # create thread2
    # t1 = threading.Thread(target=thread1) # create thread1
    t2.setDaemon(False)
    t2.start()
    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
                       LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    colorWipe(strip, Color(0, 0, 0), 0)

    with lirc.LircdConnection("project.py",) as conn:
        while True:
            string = conn.readline()
            pasreset(string)
