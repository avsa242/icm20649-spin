{
    --------------------------------------------
    Filename: ICM20649-Demo.spin
    Author:
    Description:
    Copyright (c) 2020
    Started Aug 28, 2020
    Updated Aug 28, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

' -- User-modifiable constants
    SER_RX      = 31
    SER_TX      = 30
    SER_BAUD    = 115_200

    I2C_SCL     = 16
    I2C_SDA     = 17
    I2C_HZ      = 400_000

' --

OBJ

    cfg     : "core.con.boardcfg.flip"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    imu     : "sensor.imu.6dof.icm20649.i2c"

PUB Main{}

    setup{}

    repeat{}

PUB Setup{}

    repeat until ser.startrxtx(SER_RX, SER_TX, 0, SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.str(string("Serial terminal started", ser#CR, ser#LF))
    if imu.startx(I2C_SCL, I2C_SDA, I2C_HZ)
        ser.str(string("ICM20649 driver started", ser#CR, ser#LF))
    else
        ser.str(string("ICM20649 driver failed to start - halting", ser#CR, ser#LF))
        imu.stop{}
        time.msleep(500)
        ser.stop{}

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
