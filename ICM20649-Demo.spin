{
    --------------------------------------------
    Filename: ICM20649-Demo.spin
    Author: Jesse Burt
    Description: Demo of the ICM20649 driver
    Copyright (c) 2020
    Started Aug 28, 2020
    Updated Sep 3, 2020
    See end of file for terms of use.
    --------------------------------------------
}
CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

' -- User-modifiable constants
    LED         = cfg#LED1
    SER_RX      = 31
    SER_TX      = 30
    SER_BAUD    = 115_200

    SCL_PIN     = 16
    SDA_PIN     = 17
    I2C_HZ      = 400_000
' --

    DAT_X_COL   = 20
    DAT_Y_COL   = DAT_X_COL + 15
    DAT_Z_COL   = DAT_Y_COL + 15

OBJ

    cfg     : "core.con.boardcfg.flip"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    io      : "io"
    int     : "string.integer"
    imu     : "sensor.imu.6dof.icm20649.i2c"

PUB Main{} | dispmode

    setup{}

    imu.accellowpassfilter(50)                              ' 6, 12, 24, 50, 111, 246, 473
    imu.acceldatarate(20)                                   ' 1..1121
    imu.accelscale(4)                                       ' 4, 8, 16, 30 (g's)
    imu.accelaxisenabled(%111)                              ' 0: disable, %001..%111: enable (all axes)

    imu.gyrolowpassfilter(51)                               ' 6, 12, 24, 51, 120, 152, 197, 361, 12106 (disable LPF)
    imu.gyrodatarate(20)                                    ' 1..1100
    imu.gyroscale(500)                                      ' 500, 1000, 2000, 4000
    imu.gyroaxisenabled(%111)                               ' 0: disable, %001..%111: enable (all axes)
    imu.gyrobias(0, 0, 0, 1)                                ' x, y, z: -32768..32767, rw = 1 (write)

    ser.hidecursor{}
    dispmode := 0
    displaysettings{}

    repeat
        case ser.rxcheck{}
            "q", "Q":                                       ' Quit the demo
                ser.position(0, 17)
                ser.str(string("Halting"))
                imu.stop{}
                time.msleep(5)
                quit
            "c", "C":                                       ' Perform calibration
                calibrate{}
                displaysettings{}
            "r", "R":                                       ' Change display mode: raw/calculated
                ser.position(0, 14)
                repeat 2
                    ser.clearline{}
                    ser.newline{}
                dispmode ^= 1
        case dispmode
            0:
                ser.position(0, 14)
                accelraw{}
                gyroraw{}
            1:
                ser.position(0, 14)
                accelcalc{}
                gyrocalc{}

    ser.showcursor{}
    flashled(LED, 100)

PUB AccelCalc{} | ax, ay, az

    repeat until imu.acceldataready{}
    imu.accelg (@ax, @ay, @az)
    ser.str(string("Accel micro-g: "))
    ser.position(DAT_X_COL, 14)
    decimal(ax, 1_000_000)
    ser.position(DAT_Y_COL, 14)
    decimal(ay, 1_000_000)
    ser.position(DAT_Z_COL, 14)
    decimal(az, 1_000_000)
    ser.clearline{}
    ser.newline{}

PUB AccelRaw{} | ax, ay, az

    repeat until imu.acceldataready{}
    imu.acceldata (@ax, @ay, @az)
    ser.str(string("Accel raw: "))
    ser.position(DAT_X_COL, 14)
    ser.str (int.decpadded (ax, 7))
    ser.position(DAT_Y_COL, 14)
    ser.str (int.decpadded (ay, 7))
    ser.position(DAT_Z_COL, 14)
    ser.str (int.decpadded (az, 7))
    ser.clearline{}
    ser.newline{}

PUB GyroCalc{} | gx, gy, gz

'    repeat until imu.gyrodataready{}
    imu.gyrodps (@gx, @gy, @gz)
    ser.str(string("Gyro micro DPS:  "))
    ser.position(DAT_X_COL, 15)
    decimal(gx, 1_000_000)
    ser.position(DAT_Y_COL, 15)
    decimal(gy, 1_000_000)
    ser.position(DAT_Z_COL, 15)
    decimal(gz, 1_000_000)
    ser.clearline{}
    ser.newline{}

PUB GyroRaw{} | gx, gy, gz

'    repeat until imu.gyrodataready{}
    imu.gyrodata (@gx, @gy, @gz)
    ser.str(string("Gyro raw:  "))
    ser.position(DAT_X_COL, 15)
    ser.str (int.decpadded (gx, 7))
    ser.position(DAT_Y_COL, 15)
    ser.str (int.decpadded (gy, 7))
    ser.position(DAT_Z_COL, 15)
    ser.str (int.decpadded (gz, 7))
    ser.clearline{}
    ser.newline{}

PUB Calibrate{}

    ser.position (0, 21)
    ser.str(string("Calibrating..."))
    imu.calibrateaccel{}
    imu.calibrategyro{}
    ser.position (0, 21)
    ser.str(string("              "))

PUB DisplaySettings{} | axo, ayo, azo, gxo, gyo, gzo

    ser.position(0, 3)
    ser.str(string("AccelScale: "))
    ser.dec(imu.accelscale(-2))
    ser.newline{}

    ser.str(string("AccelLowPassFilter: "))
    ser.dec(imu.accellowpassfilter(-2))
    ser.newline{}

    ser.str(string("AccelDataRate: "))
    ser.dec(imu.acceldatarate(-2))
    ser.newline{}

    ser.str(string("GyroScale: "))
    ser.dec(imu.gyroscale(-2))
    ser.newline{}

    ser.str(string("GyroLowPassFilter: "))
    ser.dec(imu.gyrolowpassfilter(-2))
    ser.newline{}

    ser.str(string("GyroDataRate: "))
    ser.dec(imu.gyrodatarate(-2))
    ser.newline{}

    imu.gyrobias(@gxo, @gyo, @gzo, 0)
    ser.str(string("GyroBias: "))
    ser.dec(gxo)
    ser.str(string("(x), "))
    ser.dec(gyo)
    ser.str(string("(y), "))
    ser.dec(gzo)
    ser.str(string("(z)"))
    ser.newline{}

    imu.accelbias(@axo, @ayo, @azo, 0)
    ser.str(string("AccelBias: "))
    ser.dec(axo)
    ser.str(string("(x), "))
    ser.dec(ayo)
    ser.str(string("(y), "))
    ser.dec(azo)
    ser.str(string("(z)"))
    ser.newline{}

PRI Decimal(scaled, divisor) | whole[4], part[4], places, tmp
' Display a fixed-point scaled up number in decimal-dot notation - scale it back down by divisor
'   e.g., Decimal (314159, 100000) would display 3.14159 on the termainl
'   scaled: Fixed-point scaled up number
'   divisor: Divide scaled-up number by this amount
    whole := scaled / divisor
    tmp := divisor
    places := 0

    repeat
        tmp /= 10
        places++
    until tmp == 1
    part := int.deczeroed(||(scaled // divisor), places)

    ser.dec (whole)
    ser.char (".")
    ser.str (part)

PUB Setup{}

    repeat until ser.StartRXTX (SER_RX, SER_TX, 0, SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.str(string("Serial terminal started", ser#CR, ser#LF))
    if imu.startx(SCL_PIN, SDA_PIN, I2C_HZ)
        imu.defaults{}
        imu.presetimuactive{}
        ser.str(string("ICM20649 driver started (I2C)"))
    else
        ser.str(string("ICM20649 driver failed to start - halting"))
        imu.stop{}
        time.msleep(5)
        flashled(LED, 500)


#include "lib.utility.spin"

DAT
{
    --------------------------------------------------------------------------------------------------------
    TERMS OF USE: MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
    associated documentation files (the "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
    LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    --------------------------------------------------------------------------------------------------------
}
