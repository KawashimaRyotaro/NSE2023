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
import _thread
import math
import time

maxDuty             = const(65535)
controllBase        = const(750) # straight base
altiThreshouldRange = const(10)
altiThreshouldMin   = const(5)
targetLat           = const(38.2611543)
targetLon           = const(140.8546001)

SCL     = Pin(3)
SDA     = Pin(2)
led     = Pin(25, Pin.OUT)
i2c     = I2C(1, scl=SCL, sda=SDA, freq=100000)# on pyboard
trigger = Pin(27, Pin.OUT)
echo    = Pin(26, Pin.IN)

myBMP  = BMP180(i2c, SCL, SDA)
myBMX  = AE_BMX055(i2c)
myEcho = ECHOdev(trigger, echo)
myGPS  = GPSdev(0, 9600)
myIM   = IMdev()
myMD   = MD2()                      #min:0 max:1000
mySD   = SDDriver(numLen=3, editDir="/sd", name="/dataLogMain", ext=".csv")
mySD.mkFile()
mySD.wTFile("sensing rate, ax, ay, az, gx, gy, gz, mx, my, mz, temp, press, alti, Lat, Lon, echo distance, Phase, azims, targetAngle, OpR, OpL, altiMax, altiMin, middleMagx, middleMagy, rangeMagx, rangeMagy, offsetAzims")

pt = time.ticks_ms()         # sensing rate of IMU (previous time)
nt = time.ticks_ms()         # sensing rate of IMU (now time)
dt = nt-pt                   # sensing rate[ms]

phaseChangedTime = time.ticks_ms()

phase = -1

acc        = [0, 0, 0]
gyro       = [0, 0, 0]
mag        = [0, 0, 0]
temp, press, alti = 0, 0, 0
Lat               = 0.0
Lon               = 0.0
echoDist          = 200
azims             = 0
targetAngle       = 0
OpR, OpL          = 0, 0

altiMax = -100
altiMin = 100

# valiable for geomagnetic caliblation
magMax = [-10000, -10000]
magMin = [1000, 1000]
middleMag = [0, 0]
rangeMag  = [1, 1]

offsetAzims = 0

dataList = [dt, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], temp, press, alti, Lat, Lon, echoDist, Phase, azims, targetAngle, OpR, OpL, altiMax, altiMin, middleMag[0], middleMag[1], rangeMag[0], rangeMagy[1], offsetAzims]

LEDtimeN  = time.ticks_ms()  # time keeper (refreshed time, for LED)
LEDtimeR1 = time.ticks_ms()  # time keeper (refetence time1, for LED)
LEDtimeR2 = time.ticks_ms()  # time keeper (refetence time2, for LED)
LEDct     = 0                # LED brink count
LEDflag   = False            # LED brink flag (True: on, false: off)
led.value(0)

IMtimeN   = time.ticks_ms()  # time keeper (refreshed time, for IM)
IMtimeP   = time.ticks_ms()  # time keeper (previous time, for IM)

motorMode = 0
motorVal  = 0
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


def main0():
    while True:
        getSensor()
        motorRefresh()
        logingData()
        refreshValue()
        
        match phase:
            # Macine initializing
            case -1:
                init()
            
            # Frying phase
            case 0:
                phase0()
            
            # Parashute separation and Stabilizer launching
            case 1:
                phase1()
            
            # Geomagnetic caliblation
            case 2:
                phase2()
            
            # GPS approaching
            case 3:
                phase3()
            
            # Image approaching
            case 4:
                phase4()
            
            # Goal
            case 5:
                phase5()
                break

def main1():
    while True:
        LEDtimeN = time.ticks_ms()  # refresh time keeper
        IMtimeN  = time.ticks_ms()  # refresh time keeper
        
        myECHO.readDistance()
        echoDist = myECHO.distance
        
        if myGPS.readable():
            myGPS.readGPS()
            Lat = myGPS.Lati
            Lon = myGPS.Long
        
        if IMtimeN-IMtimeP >= 1000:
            myIM.send(dataList)
            IMtimeP = IMtimeN
        
        LEDmanager(phase)
    

def init():
    if Lat == 0.0 or Lon == 0.0:
        pass
    else:
        phase++
        phaseChangedTime = time.ticks_ms()


def phase0():
    if alti > altiMax:
        altiMax = alti
    if alti < altiMin:
        altiMin = alti
    if altiMax-altiMin >= altiThreshouldRange and abs(alti-altiMin) <= altiThreshouldMin:
        phase++
        phaseChangedTime = time.ticks_ms()
        
    
def phase1():
    now = time.ticks_ms()
    
    # go straight in order to leave from parashute
    if now-phaseChangedTime < 5:
        motorMode = 1
        motorVal  = 900
        
    # turn during geomagnetic caliblation
    elif 5 <= now-phaseChangedTime <15:
        motorMode = 3
        motorVal  = 500
        geomagneticCalib()
    
    
    
def phase2():


def phase3():


def phase4():


def phase5():


def getSensor(debug = False):
    
    acc[0],  acc[1],  acc[2] = bmx055.accel
    gyro[0], gyro[1], gyro[2] = bmx055.gyro
    mag[0],  mag[1],  mag[2]  = bmx055.mag
      
    temp  = bmp180.temperature
    press = bmp180.pressure
    alti  = bmp180.altitude
      
    nt = time.ticks_us()
    dt = nt - pt
    pt = nt
    
    if phase >= 3:
        xMag = (xMag-middleMagx)/rangeMagx
        yMag = (yMag-middleMagy)/rangeMagy
        zMag = (zMag-middleMagz)/rangeMagz
        
    if debug:
        print('Accl= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xAccl, yAccl, zAccl))
        print('Gyro= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xGyro, yGyro, zGyro))
        print('Mag=  ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xMag, yMag, zMag))
        print('Accl= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xAccl, yAccl, zAccl))
        print("T=", temp, ", p=", p, ", h=", altitude)
        print("echo distance=", myECHO.distance)
        print("dt=", dt*10**-3, "s")


def motorRefresh():
    match motorMode:
        case -1:
            myMD.qSt()
            OpR, OpL = 2000, 2000
        case 0:
            myMD.St()
            OpR, OpL = 0, 0
        case 1:
            myMD.straight(motorVal)
            OpR, OpL = motorVal, motorVal
        case 2:
            myMD.analogStraight(motorVal, controllBase)
            OpR, OpL = controllBase-motorVal, controllBase+motorVal
        case 3:
            myMD.turn(motorVal)
            OpR, OpL = -motorVal, motorVal
        case _:
            pass
                 
            
def LEDmanager(limCt):
    if limCt<0:
        led.value(1)
        LEDtimeR1 = LEDtimeN  # refresh time keeper
        LEDtimeR2 = LEDtimeN  # refresh time keeper
        LEDct     = 0
        LEDflag   = True
    else:
        if LEDflag:
            if LEDtimeN-LEDtimeR2 >= 200:
                led.value(0)
        else:
            if LEDtimeN-LEDtimeR1 >= 2000:
                led.value(1)
                LEDtimeR1 = LEDtimeN  # refresh time keeper
                LEDtimeR2 = LEDtimeN  # refresh time keeper
                LEDct     = 0
                LEDflag   = True
            elif LEDtimeN-LEDtimeR2 >= 400 and LEDct < limCt:
                led.value(1)
                LEDtimeR2 = LEDtimeN
                LEDct++
                LEDflag   = True
     
     
def logingData():
    logData = ""
    for i in dataList:
        logData += f"{i}"
        if i == dataList[-1]:
            pass
        else:
            logData += ", "
    mySD.wTFile(logData)


def refreshValue():
    azims = math.atan2(mag[1], mag[0])
    targetAngle = math.atan2(targetLat-Lat, targetLon-Lon)
    dataList = [dt, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], temp, press, alti, Lat, Lon, echoDist, phase, azims, targetAngle, OpR, OpL, altiMax, altiMin, middleMag[0], middleMag[1], rangeMag[0], rangeMagy[1], offsetAzims]


def geomagneticCalib():
    for i in range(0, 2):
        if mag[i] > magMax[i]:
            magMax[i] = mag[i]


if __name__ == "__main__":
    _thread.start_new_thread(main0,())
    _thread.start_new_thread(main1,())