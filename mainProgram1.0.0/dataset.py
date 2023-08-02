from typing import Any
from machine import SPI, I2C, Pin, PWM
from bmp180 import BMP180
from BMX055 import AE_BMX055
from echo import ECHOdev
from GPSmaster import GPSdev
from IM import IMdev
from motorDriver import MD2
from sdcard import SDCard
from SDDriver import SDDriver
from servo import Servo
from micropython import const
import math
import time


class params():
    def __init__(self):
        self.maxDuty             = const(65535)
        self.controllBase        = const(750) # simple and analog straight base
        self.altiThreshouldRange = const(10)
        self.altiThreshouldMin   = const(5)
        self.targetLat           = const(38.2611543)
        self.targetLon           = const(140.8546001)
        self.svRepeat            = const(3)


class pinAsign():
    def __init__(self):
        self.SCL     = Pin(3)
        self.SDA     = Pin(2)
        self.led     = Pin(25, Pin.OUT)
        self.i2c     = I2C(1, scl=self.SCL, sda=self.SDA, freq=100000)# on pyboard
        self.trigger = Pin(27, Pin.OUT)
        self.echo    = Pin(26, Pin.IN)
        self.sv0     = Pin(22)
        self.sv1     = Pin(5)
        self.led.value(0)


class modules():
    def __init__(self):
        self.myBMP  = BMP180(pinAsign.i2c, pinAsign.SCL, pinAsign.SDA)
        self.myBMX  = AE_BMX055(pinAsign.i2c)
        self.myEcho = ECHOdev(pinAsign.trigger, pinAsign.echo)
        self.myGPS  = GPSdev(0, 9600)
        self.myIM   = IMdev()
        self.myMD   = MD2()                      #min:0 max:1000
        self.stab = Servo(pinAsign.sv0, 180, 140)
        self.para = Servo(pinAsign.sv1, 0, 90)
        self.mySD   = SDDriver(numLen=3, editDir="/sd", name="/dataLogMain", ext=".csv")
        self.mySD.mkFile()
        self.mySD.wTFile("sensing rate, ax, ay, az, gx, gy, gz, mx, my, mz, temp, press, alti, Lat, Lon, echo distance, Phase, azims, targetAngle, OpR, OpL, altiMax, altiMin, middleMagx, middleMagy, rangeMagx, rangeMagy, offsetAzims, distance to E")


class valiables:
    def __init__(self):

        # time keepers
        self.pt = time.ticks_ms()         # sensing rate of IMU (previous time)
        self.nt = time.ticks_ms()         # sensing rate of IMU (now time)
        self.dt = self.nt-self.pt                   # sensing rate[ms]

        self.LEDtimeN  = time.ticks_ms()  # time keeper (refreshed time, for LED)
        self.LEDtimeR1 = time.ticks_ms()  # time keeper (refetence time1, for LED)
        self.LEDtimeR2 = time.ticks_ms()  # time keeper (refetence time2, for LED)
        self.LEDct     = 0                # LED brink count
        self.LEDflag   = False            # LED brink flag (True: on, false: off)

        self.IMtimeN   = time.ticks_ms()  # time keeper (refreshed time, for IM)
        self.IMtimeP   = time.ticks_ms()  # time keeper (previous time, for IM)

        self.phaseChangedTime = time.ticks_ms()

        # the most important
        self.phase = -1

        # valiables
        self.acc        = [0, 0, 0]
        self.gyro       = [0, 0, 0]
        self.mag        = [0, 0, 0]
        self.temp, self.press, self.alti = 0, 0, 0
        self.Lat               = 0.0
        self.Lon               = 0.0
        self.echoDist          = 200
        self.azims             = 0
        self.distance          = 100
        self.targetAngle       = 0
        self.OpR, self.OpL     = 0, 0

        # valiables for landing recognition
        self.altiMax = -100
        self.altiMin = 100

        # valiables for geomagnetic caliblation
        self.magMax = [-10000, -10000]
        self.magMin = [1000, 1000]
        self.middleMag = [0, 0]
        self.rangeMag  = [1, 1]

        # valiable for azims caliblation
        self.offsetAzims = 0

        # datalist for sending and loging
        self.dataList = [self.dt, self.acc[0], self.acc[1], self.acc[2], self.gyro[0], self.gyro[1], self.gyro[2], self.mag[0], self.mag[1], self.mag[2], self.temp, self.press, self.alti, self.Lat, self.Lon, self.echoDist, self.phase, self.azims, self.targetAngle, self.OpR, self.OpL, self.altiMax, self.altiMin, self.middleMag[0], self.middleMag[1], self.rangeMag[0], self.rangeMagy[1], self.offsetAzims, self.distance]

        # valiables for motor drive
        self.motorMode = 0
        self.motorVal  = 0
        # about motor variables
        # motorMode: select motor mode
        #     ➡ -1: rocking (turning all pin on)
        #         0: free (turning all pin off)
        #         1: straight (turning both F pin on samely)
        #         2: analog straight (turning both F pin on and controll the value)
        #         3: turn (turning F pin or B pin on and controll the value)
        #
        # motorval : controll motor speed
        #     ➡ -1, 0: doesn't have the meanig
        #            1: only pluss value (only +)
        #         2, 3: all value (-:go left, +:go right)

        # valiable of servo flag
        self.svFlag = False