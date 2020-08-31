{
    --------------------------------------------
    Filename: sensor.imu.6dof.icm20649.i2c.spin
    Author: Jesse Burt
    Description: Driver for the TDK/Invensense ICM20649 6DoF IMU
    Copyright (c) 2020
    Started Aug 28, 2020
    Updated Aug 29, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR                = core#SLAVE_ADDR
    SLAVE_RD                = core#SLAVE_ADDR|1

    DEF_SCL                 = 28
    DEF_SDA                 = 29
    DEF_HZ                  = 100_000
    I2C_MAX_FREQ            = core#I2C_MAX_FREQ

' Indicate to user apps how many Degrees of Freedom each sub-sensor has
'   (also imply whether or not it has a particular sensor)
    ACCEL_DOF               = 3
    GYRO_DOF                = 3
    MAG_DOF                 = 0
    BARO_DOF                = 0
    DOF                     = ACCEL_DOF + GYRO_DOF + MAG_DOF + BARO_DOF

' Bias adjustment (AccelBias(), GyroBias()) read or write
    R                       = 0
    W                       = 1

' Axis-specific constants
    X_AXIS                  = 0
    Y_AXIS                  = 1
    Z_AXIS                  = 2
    ALL_AXIS                = 3

' Temperature scale constants
    CELSIUS                 = 0
    FAHRENHEIT              = 1
    KELVIN                  = 2

' FIFO modes
    BYPASS                  = 0
    FIFO                    = 1
    STREAM                  = 2

VAR

    word    _abiasraw[3], _gbiasraw[3]
    word    _ares, _gres
OBJ

    i2c : "com.i2c"                                     ' PASM I2C Driver
    core: "core.con.icm20649.spin"                      ' Low-level constants
    time: "time"                                        ' Basic timing functions

PUB Null{}
' This is not a top-level object

PUB Start{}: okay
' Start with "standard" Propeller I2C pins and 100kHz
    return startx(DEF_SCL, DEF_SDA, DEF_HZ)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ): okay
' Start with custom I2C I/O pin settings and bus speed
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.setupx (SCL_PIN, SDA_PIN, I2C_HZ)
                time.usleep(core#TPOR)                  ' Device startup delay
                if i2c.present(SLAVE_WR)                ' Response from device?
                    if deviceid{} == core#DEVID_RESP    ' Verify device ID
                        reset{}                         ' Restore default settings
                        return

    return FALSE                                        ' One of the above failed

PUB Stop{}
' As applicable:
'   power down the device
'   stop the low-level interface engine (e.g., I2C, SPI, UART, etc)
    i2c.terminate{}

PUB Defaults{}
' Set factory defaults
    reset{}

PUB PresetIMUActive 'XXX tentatively named
' Preset settings:
'   Accel active
'   Gyro active
'   Temp sensor active
'   Auto clock source
'   ODR: xxx Hz

PUB AccelAxisEnabled(xyz_mask): curr_mask
' Enable data output for accelerometer (all axes)
'   Valid values: %000 (disable) or %001..%111 (enable), for all axes
'   Any other value polls the chip and returns the current setting
'   NOTE: All axes are affected. The xyz_mask parameter is used for
'       compatibility with other IMU drivers.
    curr_mask := $00
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case xyz_mask
        %000:
            xyz_mask := %111 << core#DISABLE_ACCEL      ' Chip logic is inverse
        %001..%111:                                     ' If any bit is set,
            xyz_mask := %000                            '   enable the accel
        other:
            curr_mask >>= core#DISABLE_ACCEL
            curr_mask &= core#DISABLE_ACCEL_BITS
            return (curr_mask ^ %111)

PUB AccelBias(ptr_x, ptr_y, ptr_z, rw) | tmp[3], tc_bit[3]
' Read or write/manually set accelerometer calibration offset values
'   Valid values:
'       When rw == W (1, write)
'           ptr_x, ptr_y, ptr_z: -16384..16383
'       When rw == R (0, read)
'           ptr_x, ptr_y, ptr_z:
'               Pointers to variables to hold current settings for respective axes
'   NOTE: The ICM20649 accelerometer is pre-programmed with offsets, which may or may not be adequate for your application
    readreg(core#XA_OFFS_H, 2, @tmp[X_AXIS])            ' Discrete reads because the three axes
    readreg(core#YA_OFFS_H, 2, @tmp[Y_AXIS])            '   aren't contiguous register pairs
    readreg(core#ZA_OFFS_H, 2, @tmp[Z_AXIS])

    case rw
         W:
            tc_bit[X_AXIS] := tmp[X_AXIS] & 1           ' LSB of each axis' data is a temperature compensation flag
            tc_bit[Y_AXIS] := tmp[Y_AXIS] & 1
            tc_bit[Z_AXIS] := tmp[Z_AXIS] & 1

            ptr_x := (ptr_x & $FFFE) | tc_bit[X_AXIS]
            ptr_y := (ptr_y & $FFFE) | tc_bit[Y_AXIS]
            ptr_z := (ptr_z & $FFFE) | tc_bit[Z_AXIS]

            writereg(core#XA_OFFS_H, 2, @ptr_x)
            writereg(core#YA_OFFS_H, 2, @ptr_y)
            writereg(core#ZA_OFFS_H, 2, @ptr_z)
            return
        R:
            long[ptr_x] := ~~tmp[X_AXIS]
            long[ptr_y] := ~~tmp[Y_AXIS]
            long[ptr_z] := ~~tmp[Z_AXIS]
        other:
            return

PUB AccelClearInt{} | tmp
' Clears out any interrupts set up on the Accelerometer
'   and resets all Accelerometer interrupt registers to their default values.
    tmp := $00

PUB AccelData(ptr_x, ptr_y, ptr_z) | tmp[2]
' Reads the Accelerometer output registers
    readreg(core#ACCEL_XOUT_H, 6, @tmp)
    long[ptr_x] := ~~tmp.word[2]
    long[ptr_y] := ~~tmp.word[1]
    long[ptr_z] := ~~tmp.word[0]

PUB AccelDataOverrun{}: flag
' Indicates previously acquired data has been overwritten
    flag := $00

PUB AccelDataRate(Hz): curr_hz
' Set accelerometer output data rate, in Hz
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_hz := $00

PUB AccelDataReady{}: flag
' Accelerometer sensor new data available
'   Returns TRUE or FALSE
    flag := $00

PUB AccelG(ax, ay, az) | tmpx, tmpy, tmpz
' Reads the Accelerometer output registers and scales the outputs to micro-g's (1_000_000 = 1.000000 g = 9.8 m/s/s)
    acceldata(@tmpx, @tmpy, @tmpz)
    long[ax] := tmpx * _ares
    long[ay] := tmpy * _ares
    long[az] := tmpz * _ares

PUB AccelInt{}: flag
' Flag indicating accelerometer interrupt asserted
'   Returns TRUE if interrupt asserted, FALSE if not
    flag := $00

PUB AccelScale(g): curr_scale
' Sets the full-scale range of the Accelerometer, in g's
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_scale := $00

PUB CalibrateAccel{} | tmpx, tmpy, tmpz, tmpbiasraw[3], axis, samples
' Calibrate the accelerometer
'   NOTE: The accelerometer must be oriented with the package top facing up for this method to be successful
    tmpx := tmpy := tmpz := axis := samples := 0
    longfill(@tmpbiasraw, 0, 3)
    accelbias(0, 0, 0, W)

    accelscale(0)       ' Set according to datasheet/AN recommendations
    acceldatarate(0)

    fifoenabled(TRUE)   ' Use the FIFO, if it exists
    fifomode(FIFO)
    fifothreshold (0)  ' Set according to datasheet/AN recommendations
    samples := fifothreshold(-2)
    repeat until fifofull{}

    repeat samples
' Read the accel data stored in the FIFO
        acceldata(@tmpx, @tmpy, @tmpz)
        tmpbiasraw[X_AXIS] += tmpx
        tmpbiasraw[Y_AXIS] += tmpy
        tmpbiasraw[Z_AXIS] += tmpz - (1_000_000 / _ares) ' Assumes sensor facing up!

    accelbias(tmpbiasraw[X_AXIS]/samples, tmpbiasraw[Y_AXIS]/samples, tmpbiasraw[Z_AXIS]/samples, W)

    fifoenabled(FALSE)
    fifomode(BYPASS)

PUB CalibrateGyro{} | tmpx, tmpy, tmpz, tmpbiasraw[3], axis, samples
' Calibrate the gyroscope
    tmpx := tmpy := tmpz := axis := samples := 0
    longfill(@tmpbiasraw, 0, 3)
    gyrobias(0, 0, 0, W)

    gyroscale(0)       ' Set according to datasheet/AN recommendations
    gyrodatarate(0)

    fifoenabled(TRUE)   ' Use the FIFO, if it exists
    fifomode(FIFO)
    fifothreshold (0)  ' Set according to datasheet/AN recommendations
    samples := fifothreshold(-2)
    repeat until fifofull{}

    repeat samples
' Read the accel data stored in the FIFO
        gyrodata(@tmpx, @tmpy, @tmpz)
        tmpbiasraw[X_AXIS] += tmpx
        tmpbiasraw[Y_AXIS] += tmpy
        tmpbiasraw[Z_AXIS] += tmpz

    gyrobias(tmpbiasraw[X_AXIS]/samples, tmpbiasraw[Y_AXIS]/samples, tmpbiasraw[Z_AXIS]/samples, W)

    fifoenabled(FALSE)
    fifomode(BYPASS)

PUB DeviceID{}: id
' Read device identification
    readreg(core#WHO_AM_I, 1, @id)

PUB FIFOEnabled(enabled): curr_setting
' Enable FIFO memory
'   Valid values: FALSE (0), TRUE(1 or -1)
'   Any other value polls the chip and returns the current setting
    curr_setting := $00

PUB FIFOFull: flag
' FIFO Threshold status
'   Returns: FALSE (0): lower than threshold level, TRUE(-1): at or higher than threshold level
    flag := $00

PUB FIFOMode(mode): curr_mode
' Set FIFO behavior
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_mode := $00

PUB FIFOThreshold(level): curr_lvl
' Set FIFO threshold level
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_lvl := $00

PUB FIFOUnreadSamples: nr_samples
' Number of unread samples stored in FIFO
'   Returns:
    nr_samples := $00

PUB GyroAxisEnabled(xyz_mask): curr_mask
' Enable data output for Gyroscope - per axis
'   Valid values: FALSE (0) or TRUE (1 or -1), for each axis
'   Any other value polls the chip and returns the current setting
    curr_mask := $00
    case xyz_mask
        %000..%111:
        OTHER:
            return

PUB GyroBias(gxbias, gybias, gzbias, rw)
' Read or write/manually set Gyroscope calibration offset values
'   Valid values:
'       rw:
'           R (0), W (1)
'       gxbias, gybias, gzbias:
'           -32768..32767
'   NOTE: When rw is set to READ, gxbias, gybias and gzbias must be addresses of respective variables to hold the returned calibration offset values.
    case rw
        R:
            long[gxbias] := _gbiasraw[X_AXIS]
            long[gybias] := _gbiasraw[Y_AXIS]
            long[gzbias] := _gbiasraw[Z_AXIS]

        W:
            case gxbias
                -32768..32767:
                    _gbiasraw[X_AXIS] := gxbias
                OTHER:

            case gybias
                -32768..32767:
                    _gbiasraw[Y_AXIS] := gybias
                OTHER:

            case gzbias
                -32768..32767:
                    _gbiasraw[Z_AXIS] := gzbias
                OTHER:
        OTHER:
            return

PUB GyroClearInt{} | tmp
' Clears out any interrupts set up on the Gyroscope and resets all Gyroscope interrupt registers to their default values.
    tmp := $00

PUB GyroData(gx, gy, gz) | tmp[2]
' Reads the Gyroscope output registers
    tmp := $00
    long[gx] := ~~tmp.word[X_AXIS] - _gbiasraw[X_AXIS]
    long[gy] := ~~tmp.word[Y_AXIS] - _gbiasraw[Y_AXIS]
    long[gz] := ~~tmp.word[Z_AXIS] - _gbiasraw[Z_AXIS]

PUB GyroDataRate(Hz): curr_rate
' Set Gyroscope Output Data Rate, in Hz
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_rate := $00

PUB GyroDataReady{}: flag
' Gyroscope sensor new data available
'   Returns TRUE or FALSE
    flag := $00

PUB GyroDPS(gx, gy, gz) | tmp[3]
' Read the Gyroscope output registers and scale the outputs to micro-degrees of rotation per second (1_000_000 = 1.000000 deg/sec)
    tmp := $00
    gyrodata(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])
    long[gx] := tmp[X_AXIS] * _gres
    long[gy] := tmp[Y_AXIS] * _gres
    long[gz] := tmp[Z_AXIS] * _gres

PUB GyroInt{}: flag
' Flag indicating gyroscope interrupt asserted
'   Returns TRUE if interrupt asserted, FALSE if not
    flag := $00

PUB GyroIntSelect(mode): curr_mode
' Set gyroscope interrupt generator selection
'   Valid values:
'
'   Any other value polls the chip and returns the current setting
    curr_mode := $00

PUB GyroScale(scale): curr_scale
' Set full scale of gyroscope output, in degrees per second (dps)
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_scale := $00

PUB Interrupt{}: flag
' Flag indicating one or more interrupts asserted
'   Returns TRUE if one or more interrupts asserted, FALSE if not
    flag := $00

PUB Powered(state): curr_state
' Enable device power
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(core#PWR_MGMT_1, 1, @curr_state)
    case ||(state)
        0, 1:
            state := (||(state) ^ 1) << core#SLEEP
        other:
            return ((curr_state >> core#SLEEP) & 1) == 0

    state := (curr_state & core#SLEEP_MASK) | state
    writereg(core#PWR_MGMT_1, 1, @state)

PUB Reset{} | tmp
' Reset the device
    tmp := 1 << core#DEVICE_RESET
    writereg(core#PWR_MGMT_1, 1, @tmp)

PUB Temperature{}: temp
' Get temperature from chip
'   Returns: Temperature in hundredths of a degree Celsius (1000 = 10.00 deg C)
    temp := $00

PUB TempDataReady{}: flag
' Flag indicating new temperature sensor data available
'   Returns TRUE or FALSE
    flag := $00

PUB XLGDataRate(Hz): curr_rate
' Set output data rate, in Hz, of accelerometer and gyroscope
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_rate := $00

PRI readReg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt, tmp
' Read nr_bytes from the slave device
    case reg_nr                                         ' Basic register validation
        core#ACCEL_XOUT_H..core#TEMP_OUT_H:             ' Prioritize output data regs
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr
            i2c.start{}
            i2c.wr_block(@cmd_pkt, 2)
            i2c.start{}
            i2c.write(SLAVE_RD)
            repeat tmp from nr_bytes-1 to 0
                byte[ptr_buff][tmp] := i2c.read(tmp == 0)
            i2c.stop{}
        $000, $003, $005, $006, $007, $00f..$013, $017, $019..$01c, $028,{
        } $029, $03b..$052, $066..$069, $070..$072, $074, $076, $102, $103,{
        } $104, $10e..$110, $114, $115, $117, $118, $11a, $11b, $128,{
        } $200..$209, $210..$215, $252..$254, $300..$317:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := core#REG_BANK_SEL        ' Set register bank (0..3):
            cmd_pkt.byte[2] := reg_nr.byte[1] << core#USER_BANK '   use the bank # encoded in nibble 2
            i2c.start{}
            i2c.wr_block(@cmd_pkt, 3)

            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr.byte[0]            ' Actual reg # is just the lower 8 bits
            i2c.start{}
            i2c.wr_block(@cmd_pkt, 2)

            i2c.start{}
            i2c.write(SLAVE_RD)
            i2c.rd_block(ptr_buff, nr_bytes, TRUE)

            i2c.stop{}
        other:
            return

    if lookdown(reg_nr.byte[1]: 1..3)                   ' If applicable,
        cmd_pkt.byte[0] := SLAVE_WR                     ' Return to bank 0
        cmd_pkt.byte[1] := core#REG_BANK_SEL            ' when done (most used
        cmd_pkt.byte[2] := 0                            ' regs are in bank 0)

        i2c.start{}
        i2c.wr_block(@cmd_pkt, 3)
        i2c.stop{}

PRI writeReg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt, tmp
' Write nr_bytes to the slave device
    case reg_nr                                         ' Basic reg. validation
        $003, $005, $006, $007, $00f..$013, $066..$069, $072, $076, $102,{
        } $103, $104, $10e..$110, $114, $115, $117, $118, $11a, $11b, $128,{
        } $200..$209, $210..$215, $252..$254, $300..$316:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := core#REG_BANK_SEL
            cmd_pkt.byte[2] := reg_nr.byte[1] << core#USER_BANK ' Use bank # encoded in nibble 2
            i2c.start{}
            i2c.wr_block(@cmd_pkt, 3)

            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr.byte[0]           ' Actual reg # is just the lower 8 bits
            i2c.start{}
            i2c.wr_block(@cmd_pkt, 2)

            repeat tmp from nr_bytes-1 to 0
                i2c.write(byte[ptr_buff][tmp])
            i2c.stop{}
        other:
            return

    if lookdown(reg_nr.byte[1]: 1..3)
        cmd_pkt.byte[0] := SLAVE_WR
        cmd_pkt.byte[1] := core#REG_BANK_SEL
        cmd_pkt.byte[2] := 0

        i2c.start{}
        i2c.wr_block(@cmd_pkt, 3)
        i2c.stop{}

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
