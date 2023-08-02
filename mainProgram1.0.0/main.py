from dataset import params, pinAsign, modules, valiables
import _thread
import math
import time

param  = params()
pin    = pinAsign()
module = modules()
val    = valiables()

def main0():
    while True:
        getSensor()
        motorRefresh()
        logingData()
        refreshValue()
        
        match val.phase:
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
    

def init():
    module.stab.OFF()
    module.para.OFF()
    if val.Lat == 0.0 or val.Lon == 0.0:
        pass
    else:
        val.phase = 0
        val.phaseChangedTime = time.ticks_ms()


def phase0():
    if val.alti > val.altiMax:
        val.altiMax = val.alti
    if val.alti < val.altiMin:
        val.altiMin = val.alti
    if val.altiMax-val.altiMin >= param.altiThreshouldRange and abs(val.alti-val.altiMin) <= param.altiThreshouldMin:
        val.phase = 1
        val.phaseChangedTime = time.ticks_ms()
        
    
def phase1():
    now = time.ticks_ms()
    for i in range(0,param.svRepeat):
        if (1000+2000*i) < (now-val.phaseChangedTime) < (2000*(i+1)) and not val.svFlag:
            module.stab.ON()
            module.para.ON()
        elif (2000*i) < (now-val.phaseChangedTime) < (1000+2000*i)  and val.svFlag:
            module.stab.OFF()
            module.para.OFF()
    if (now-val.phaseChangedTime) > 2000*(param.svRepeat+1):
        val.phase = 2
    
    
def phase2():
    now = time.ticks_ms()
    
    # go straight in order to leave from parashute
    if now-val.phaseChangedTime < 5:
        val.motorMode = 1
        val.motorVal  = 900
        
    # turn during geomagnetic caliblation
    elif 5 <= (now-val.phaseChangedTime) <15:
        val.motorMode = 3
        val.motorVal  = 500
        geomagneticCalib()
    
    elif now-val.phaseChangedTime >= 15:
        val.phase = 3


def phase3():
    val.motorMode = 2
    diffAngle  = val.targetAngle - val.azims
    if diffAngle > 180:
        diffAngle = diffAngle - 360
    elif diffAngle <-180:
        diffAngle = diffAngle + 360
    val.motorVal = diffAngle*200/180


def phase4():


def phase5():


def getSensor(debug = False):
    
    val.acc[0],  val.acc[1],  val.acc[2]  = module.bmx055.accel
    val.gyro[0], val.gyro[1], val.gyro[2] = module.bmx055.gyro
    val.mag[0],  val.mag[1],  val.mag[2]  = module.bmx055.mag
      
    val.temp  = module.bmp180.temperature
    val.press = module.bmp180.pressure
    val.alti  = module.bmp180.altitude
      
    val.nt = time.ticks_us()
    val.dt = val.nt - val.pt
    val.pt = val.nt
        
    if debug:
        print('Accl= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xAccl, yAccl, zAccl))
        print('Gyro= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xGyro, yGyro, zGyro))
        print('Mag=  ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xMag, yMag, zMag))
        print('Accl= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xAccl, yAccl, zAccl))
        print("T=", temp, ", p=", p, ", h=", altitude)
        print("echo distance=", myECHO.distance)
        print("dt=", dt*10**-3, "s")


def motorRefresh():
    match val.motorMode:
        case -1:
            module.myMD.qSt()
            val.OpR, val.OpL = 2000, 2000
        case 0:
            module.myMD.St()
            val.OpR, val.OpL = 0, 0
        case 1:
            module.myMD.straight(val.motorVal)
            val.OpR, val.OpL = val.motorVal, val.motorVal
        case 2:
            module.myMD.analogStraight(val.motorVal, param.controllBase)
            val.OpR, val.OpL = param.controllBase-val.motorVal, param.controllBase+val.motorVal
        case 3:
            module.myMD.turn(val.motorVal)
            val.OpR, val.OpL = -val.motorVal, val.motorVal
        case _:
            pass
     
     
def logingData():
    logData = ""
    for i in val.dataList:
        logData += f"{i}"
        if i == val.dataList[-1]:
            pass
        else:
            logData += ", "
    module.mySD.wTFile(logData)


def refreshValue():
    if val.phase >= 2:
        for i in range(0, 2):
            val.mag[i] = (val.mag[0] - val.middleMag[i]) / val.rangeMag[i]
        val.azims       = math.atan2(val.mag[1], val.mag[0])
        val.targetAngle = math.atan2(param.targetLat-val.Lat, param.targetLon-val.Lon)
        val.distance    = math.sqrt(sum([i**2 for i in [param.targetLat-val.Lat, param.targetLon-val.Lon]]))
    val.dataList = [val.dt, val.acc[0], val.acc[1], val.acc[2], val.gyro[0], val.gyro[1], val.gyro[2], val.mag[0], val.mag[1], val.mag[2], val.temp, val.press, val.alti, val.Lat, val.Lon, val.echoDist, val.phase, val.azims, val.targetAngle, val.OpR, val.OpL, val.altiMax, val.altiMin, val.middleMag[0], val.middleMag[1], val.rangeMag[0], val.rangeMag[1], val.offsetAzims]


def geomagneticCalib():
    for i in range(0, 2):
        if val.mag[i] > val.magMax[i]:
            val.magMax[i] = val.mag[i]
        if val.mag[i] > val.magMin[i]:
            val.magMin[i] = val.mag[i]
        val.rangeMag  = (val.magMax - val.magMin)/2
        val.middleMag = (val.magMax + val.middleMag)/2


def main1():
    while True:
        val.LEDtimeN = time.ticks_ms()  # refresh time keeper
        val.IMtimeN  = time.ticks_ms()  # refresh time keeper
        
        module.myECHO.readDistance()
        val.echoDist = module.myECHO.distance
        
        if module.myGPS.readable():
            module.myGPS.readGPS()
            val.Lat = module.myGPS.Lati
            val.Lon = module.myGPS.Long
        
        if val.IMtimeN-val.IMtimeP >= 1000:
            module.myIM.send(val.dataList)
            val.IMtimeP = val.IMtimeN
        
        LEDmanager(val.phase)
                 
            
def LEDmanager(limCt):
    if limCt<0:
        module.led.value(1)
        val.LEDtimeR1 = val.LEDtimeN  # refresh time keeper
        val.LEDtimeR2 = val.LEDtimeN  # refresh time keeper
        val.LEDct     = 0
        val.LEDflag   = True
    else:
        if LEDflag:
            if val.LEDtimeN-val.LEDtimeR2 >= 200:
                module.led.value(0)
        else:
            if val.LEDtimeN-val.LEDtimeR1 >= 2000:
                module.led.value(1)
                val.LEDtimeR1 = val.LEDtimeN  # refresh time keeper
                val.LEDtimeR2 = val.LEDtimeN  # refresh time keeper
                val.LEDct     = 0
                val.LEDflag   = True
            elif val.LEDtimeN-val.LEDtimeR2 >= 400 and val.LEDct < limCt:
                module.led.value(1)
                val.LEDtimeR2 = val.LEDtimeN
                val.LEDct    +=1
                LEDflag       = True


if __name__ == "__main__":
    _thread.start_new_thread(main0,())
    _thread.start_new_thread(main1,())