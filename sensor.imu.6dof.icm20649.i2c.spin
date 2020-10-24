{
    --------------------------------------------
    Filename: sensor.imu.6dof.icm20649.i2c.spin
    Author: Jesse Burt
    Description: Driver for the TDK/Invensense ICM20649 6DoF IMU
    Copyright (c) 2020
    Started Aug 28, 2020
    Updated Oct 24, 2020
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
    BYPASS                  = 2
    FIFO                    = 0
    STREAM                  = 15

' Accelerometer operating modes
    NORMAL                  = 0
    LOWPWR                  = 1

' Clock sources
    INT20                   = 0
    AUTO                    = 1
    CLKSTOP                 = 7

' Temperature scales
    C                       = 0
    F                       = 1

VAR

    word    _abiasraw[ACCEL_DOF], _gbiasraw[GYRO_DOF]
    word    _ares, _gres, _temp_scale
    byte    _roomtemp_offs

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
    accelscale(4)
    gyroscale(500)
    tempscale(C)

PUB PresetIMUActive 'XXX tentatively named
' Preset settings:
    powered(true)
    clocksource(AUTO)
    accelopmode(NORMAL)
    accelaxisenabled(%111)
    acceldatarate(1127)
    accellowpassfilter(50)
    gyroaxisenabled(%111)
    gyrodatarate(1100)
    gyrolowpassfilter(51)
    tempenabled(true)
    tempscale(C)

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

    xyz_mask := (curr_mask & core#DISABLE_ACCEL_MASK) | xyz_mask
    writereg(core#PWR_MGMT_2, 1, @xyz_mask)

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

PUB AccelDataRate(Hz): curr_Hz
' Set accelerometer output data rate, in Hz
'   Valid values: 1..1127
'   Any other value polls the chip and returns the current setting
    case Hz
        1..1127:
            Hz := (1127 / Hz) - 1
            writereg(core#ACCEL_SMPLRT_DIV, 2, @Hz)
        other:
            curr_Hz := 0
            readreg(core#ACCEL_SMPLRT_DIV, 2, @curr_Hz)
            return 1127 / (curr_Hz + 1)

PUB AccelDataReady{}: flag
' Flag indicating new accelerometer data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlgdataready{}

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

PUB AccelLowPassFilter(cutoff_Hz): curr_setting | lpf_enable
' Set accelerometer output data low-pass filter cutoff frequency, in Hz
'   Valid values: 6, 12, 24, 50, 111, 246, 473
'   Any other value polls the chip and returns the current setting
    curr_setting := lpf_enable := 0
    readreg(core#ACCEL_CFG, 1, @curr_setting)
    case cutoff_Hz
        0:                                                  ' Disable/bypass the LPF
            lpf_enable := %0
        6, 12, 24, 50, 111, 246, 473:
            cutoff_Hz := lookdown(cutoff_Hz: 246, 111, 50, 24, 12, 6, 473) << core#ACCEL_DLPFCFG
            lpf_enable := %1
        other:
            if (curr_setting & %1) <> 1                     ' The LPF bypass bit is set, so
                return 0                                    '   return 0 (LPF bypassed/disabled)
            else
                curr_setting := (curr_setting >> core#ACCEL_DLPFCFG) & core#ACCEL_DLPFCFG_BITS
                return lookup(curr_setting: 246, 111, 50, 24, 12, 6, 473)

    cutoff_Hz := (curr_setting & core#ACCEL_DLPFCFG_MASK & core#ACCEL_FCHOICE_MASK) | cutoff_Hz | lpf_enable
    writereg(core#ACCEL_CFG, 1, @cutoff_Hz)

PUB AccelOpMode(mode): curr_mode
' Set accelerometer operating mode
'   Valid values:
'       NORMAL (0): Normal operating mode
'       LOWPWR (1): Low-power mode
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
    readreg(core#LP_CONFIG, 1, @curr_mode)
    case mode
        LOWPWR, NORMAL:
            mode <<= core#ACCEL_CYCLE
        other:
            return (curr_mode >> core#ACCEL_CYCLE) & %1

    mode := (curr_mode & core#ACCEL_CYCLE_MASK) | mode
    writereg(core#LP_CONFIG, 1, @mode)

PUB AccelScale(g): curr_scl
' Set accelerometer full-scale range, in g's
'   Valid values: *4, 8, 16, 30
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#ACCEL_CFG, 1, @curr_scl)
    case g
        4, 8, 16, 30:
            g := lookdownz(g: 4, 8, 16, 30) << core#ACCEL_FS_SEL
            _ares := lookupz(g >> core#ACCEL_FS_SEL: 0_000122, 0_000244, 0_000488, 0_000976)
            ' 1/8192 (LSB per g), 1/4096, 1/2048, 1/1024 * 1_000_000
        other:
            curr_scl := (curr_scl >> core#ACCEL_FS_SEL) & core#ACCEL_FS_SEL_BITS
            return lookupz(curr_scl: 4, 8, 16, 30)

    g := ((curr_scl & core#ACCEL_FS_SEL_MASK) | g) & core#ACCEL_CFG_MASK
    writereg(core#ACCEL_CFG, 1, @g)

PUB CalibrateAccel{} | tmpx, tmpy, tmpz, tmpbias[3], axis, samples, factory_bias[3], orig_scale, orig_datarate, orig_lpf
' Calibrate the accelerometer
'   NOTE: The accelerometer must be oriented with the package top facing up for this method to be successful
    longfill(@tmpx, 0, 14)                                  ' Initialize variables to 0
    orig_scale := accelscale(-2)                            ' Preserve the user's original settings
    orig_datarate := acceldatarate(-2)
    orig_lpf := accellowpassfilter(-2)

    accelscale(4)                                           ' Set accel to most sensitive scale,
    acceldatarate(1172)                                     '   fastest sample rate,
    accellowpassfilter(111)                                 '   and a low-pass filter of 111Hz

                                                            ' ICM20649 accel has factory bias offsets,
                                                            '   so read them in first
    accelbias(@factory_bias[X_AXIS], @factory_bias[Y_AXIS], @factory_bias[Z_AXIS], 0)

    samples := 40                                           ' # samples to use for averaging

    repeat samples
        repeat until acceldataready
        acceldata(@tmpx, @tmpy, @tmpz)
        tmpbias[X_AXIS] += tmpx
        tmpbias[Y_AXIS] += tmpy
        tmpbias[Z_AXIS] += tmpz - (1_000_000 / _ares)       ' Assumes sensor facing up!

    repeat axis from X_AXIS to Z_AXIS
        tmpbias[axis] /= samples
        tmpbias[axis] := (factory_bias[axis] - (tmpbias[axis]/8))

    accelbias(tmpbias[X_AXIS], tmpbias[Y_AXIS], tmpbias[Z_AXIS], W)

    accelscale(orig_scale)                                  ' Restore user settings
    acceldatarate(orig_datarate)
    accellowpassfilter(orig_lpf)

PUB CalibrateGyro{} | tmpx, tmpy, tmpz, tmpbiasraw[3], axis, samples, orig_scale, orig_datarate, orig_lpf
' Calibrate the gyroscope
    longfill(@tmpx, 0, 8)                                   ' Initialize variables to 0
    orig_scale := gyroscale(-2)                             ' Preserve the user's original settings
    orig_datarate := gyrodatarate(-2)
    orig_lpf := gyrolowpassfilter(-2)

    gyroscale(500)                                          ' Set gyro to most sensitive scale,
    gyrodatarate(1100)                                      '   fastest sample rate,
    gyrolowpassfilter(197)                                  '   and a low-pass filter of 197Hz
    gyrobias(0, 0, 0, W)                                    ' Reset gyroscope bias offsets
    samples := 40                                           ' # samples to use for average

    repeat samples                                          ' Accumulate samples to be averaged
        repeat until gyrodataready
        gyrodata(@tmpx, @tmpy, @tmpz)
        tmpbiasraw[X_AXIS] -= tmpx                          ' Bias offsets are _added_ by the chip, so
        tmpbiasraw[Y_AXIS] -= tmpy                          '   negate the samples
        tmpbiasraw[Z_AXIS] -= tmpz

                                                            ' Write offsets to sensor (scaled to expected range)
    gyrobias((tmpbiasraw[X_AXIS]/samples) / 4, (tmpbiasraw[Y_AXIS]/samples) / 4, (tmpbiasraw[Z_AXIS]/samples) / 4, W)

    gyroscale(orig_scale)                                   ' Restore user settings
    gyrodatarate(orig_datarate)
    gyrolowpassfilter(orig_lpf)

PUB ClockSource(src): curr_src
' Set sensor clock source
'   Valid values:
'       INT20 (0): Internal 20MHz oscillator
'      *AUTO (1): Automatically select best choice (PLL if ready, else internal oscillator)
'       CLKSTOP (7): Stop clock and hold in reset
    curr_src := 0
    readreg(core#PWR_MGMT_1, 1, @curr_src)
    case src
        INT20, AUTO, CLKSTOP:
        other:
            return curr_src & core#CLKSEL_BITS

    src := (curr_src & core#CLKSEL_MASK) | src
    writereg(core#PWR_MGMT_1, 1, @src)

PUB DeviceID{}: id
' Read device identification
    readreg(core#WHO_AM_I, 1, @id)

PUB FIFOEnabled(state): curr_state
' Enable the FIFO
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: This disables the interface to the FIFO, but the chip will still write data to it, if FIFO data sources are defined with FIFOSource()
    curr_state := 0
    readreg(core#USER_CTRL, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#FIFO_EN
            state := ((curr_state & core#FIFO_EN_MASK) | state) & core#USER_CTRL_MASK
            writereg(core#USER_CTRL, 1, @state)
        other:
            return ((curr_state >> core#FIFO_EN) & 1) == 1

PUB FIFOFull{}: flag
' Flag indicating FIFO is full
'   Returns: TRUE (-1) if FIFO is full, FALSE (0) otherwise
'   NOTE: If this flag is set, the oldest data has already been dropped from the FIFO
    readreg(core#INT_STATUS_2, 1, @flag)
    return ((flag >> core#FIFO_OVERFLOW_INT) & 1) == 1

PUB FIFOMode(mode): curr_mode
' Set FIFO mode
'   Valid values:
'       BYPASS (2): FIFO disabled
'       STREAM (0): FIFO enabled; when full, new data overwrites old data
'       FIFO (15): FIFO enabled; when full, no new data will be written to FIFO
'   Any other value polls the chip and returns the current setting
'   NOTE: If no data sources are set using FIFOSource(), the current mode returned will be BYPASS (0), regardless of what the mode was previously set to
    curr_mode := 0
    readreg(core#FIFO_MODE, 1, @curr_mode)
    case mode
        BYPASS:                                 ' If bypassing the FIFO, turn
            fifosource(%00000000)               '   off FIFO data collection
            return
        STREAM, FIFO:
        other:
            curr_mode := curr_mode & 1
            if fifosource(-2)                   ' If there's a mask set with FIFOSource(), return
                return                          '   either STREAM or FIFO as the current mode
            else
                return BYPASS                   ' If not, anything besides 0 (BYPASS) doesn't really matter or make sense
    mode := (curr_mode & core#FIFO_MODE_MASK) | mode
    writereg(core#FIFO_MODE, 1, @mode)

PUB FIFORead(nr_bytes, ptr_data)
' Read FIFO data
'   Structure of data stored in FIFO
'   (as applicable, depending on what sources are enabled with FIFOSource()):
'       Accel X MSB, LSB
'       Accel Y MSB, LSB
'       Accel Z MSB, LSB
'       Gyro X MSB, LSB
'       Gyro Y MSB, LSB
'       Gyro Z MSB, LSB
'       Temp MSB, LSB
    readreg(core#FIFO_R_W, nr_bytes, ptr_data)

PUB FIFOSource(mask): curr_mask
' Set FIFO source data, as a bitmask
'   Valid values: 1: enable source, 0: disable source
'       Bits: 43210
'           4: Accelerometer
'           3: Gyro Z-axis
'           2: Gyro Y-axis
'           1: Gyro X-axis
'           0: Temperature
'   Any other value polls the chip and returns the current setting
    case mask
        %00000..%11111:
            writereg(core#FIFO_EN_2, 1, @mask)
        other:
            curr_mask := 0
            readreg(core#FIFO_EN_2, 1, @curr_mask)
            return

PUB FIFOThreshold(level): curr_lvl
' Set FIFO threshold level
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_lvl := $00

PUB FIFOUnreadSamples{}: nr_samples
' Number of unread samples stored in FIFO
'   Returns: unsigned 13bit
    readreg(core#FIFO_COUNTH, 2, @nr_samples)

PUB GyroAxisEnabled(xyz_mask): curr_mask
' Enable data output for gyroscope (all axes)
'   Valid values: %000 (disable) or %001..%111 (enable), for all axes
'   Any other value polls the chip and returns the current setting
'   NOTE: All axes are affected. The xyz_mask parameter is used for
'       compatibility with other IMU drivers.
    curr_mask := $00
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case xyz_mask
        %000:
            xyz_mask := %111                            ' Chip logic is inverse
        %001..%111:                                     ' If any bit is set,
            xyz_mask := %000                            '   enable the gyro
        other:
            curr_mask &= core#DISABLE_GYRO_BITS
            return (curr_mask ^ %111)

    xyz_mask := (curr_mask & core#DISABLE_GYRO_MASK) | xyz_mask
    writereg(core#PWR_MGMT_2, 1, @xyz_mask)

PUB GyroBias(ptr_x, ptr_y, ptr_z, rw) | tmp[3]
' Read or write/manually set gyroscope calibration offset values
'   Valid values:
'       When rw == W (1, write)
'           ptr_x, ptr_y, ptr_z: -32768..32767
'       When rw == R (0, read)
'           ptr_x, ptr_y, ptr_z:
'               Pointers to variables to hold current settings for respective axes
    case rw
         W:
            writereg(core#XG_OFFS_USRH, 2, @ptr_x)
            writereg(core#YG_OFFS_USRH, 2, @ptr_y)
            writereg(core#ZG_OFFS_USRH, 2, @ptr_z)
            return
        R:
            readreg(core#XG_OFFS_USRH, 2, @tmp[X_AXIS])
            readreg(core#YG_OFFS_USRH, 2, @tmp[Y_AXIS])
            readreg(core#ZG_OFFS_USRH, 2, @tmp[Z_AXIS])
            long[ptr_x] := ~~tmp[X_AXIS]
            long[ptr_y] := ~~tmp[Y_AXIS]
            long[ptr_z] := ~~tmp[Z_AXIS]
        other:
            return

PUB GyroClearInt{} | tmp
' Clears out any interrupts set up on the Gyroscope and resets all Gyroscope interrupt registers to their default values.
    tmp := $00

PUB GyroData(gx, gy, gz) | tmp[2]
' Reads the Gyroscope output registers
    tmp := $00
    readreg(core#GYRO_XOUT_H, 6, @tmp)
    long[gx] := ~~tmp.word[2]
    long[gy] := ~~tmp.word[1]
    long[gz] := ~~tmp.word[0]

PUB GyroDataRate(Hz): curr_rate
' Set gyroscope output data rate, in Hz
'   Valid values: 1..1100
'   Any other value polls the chip and returns the current setting
    case Hz
        1..1100:
            Hz := (1100 / Hz) - 1
            writereg(core#GYRO_SMPLRT_DIV, 1, @Hz)
        other:
            curr_rate := 0
            readreg(core#GYRO_SMPLRT_DIV, 1, @curr_rate)
            return 1100 / (curr_rate + 1)

PUB GyroDataReady{}: flag
' Flag indicating new gyroscope data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    flag := $00
    return xlgdataready{}

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

PUB GyroLowPassFilter(cutoff_Hz): curr_setting | lpf_enable
' Set gyroscope output data low-pass filter cutoff frequency, in Hz
'   Valid values: 6, 12, 24, 51, 120, 152, 197, 361, *12106 (LPF disabled)
'   Any other value polls the chip and returns the current setting
    curr_setting := lpf_enable := 0
    readreg(core#GYRO_CFG1, 1, @curr_setting)
    case cutoff_Hz
        0, 12106:                                           ' Disable/bypass the LPF
            lpf_enable := %0
        6, 12, 24, 51, 120, 152, 197, 361:
            cutoff_Hz := lookdownz(cutoff_Hz: 197, 152, 120, 51, 24, 12, 6, 361) << core#GYRO_DLPFCFG
            lpf_enable := %1
        other:
            if (curr_setting & %1) <> 1                     ' The LPF bypass bit is clear, so
                return 12106                                '   return 12106 (LPF bypassed/disabled)
            else
                curr_setting := (curr_setting >> core#GYRO_DLPFCFG) & core#GYRO_DLPFCFG_BITS
                return lookupz(curr_setting: 197, 152, 120, 51, 24, 12, 6, 361)

    cutoff_Hz := (curr_setting & core#GYRO_DLPFCFG_MASK & core#GYRO_FCHOICE_MASK) | cutoff_Hz | lpf_enable
    writereg(core#GYRO_CFG1, 1, @cutoff_Hz)

PUB GyroScale(dps): curr_scl
' Set gyroscope full-scale range, in degrees per second
'   Valid values: *500, 1000, 2000, 4000
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#GYRO_CFG1, 1, @curr_scl)
    case dps
        500, 1000, 2000, 4000:
            dps := lookdownz(dps: 500, 1000, 2000, 4000) << core#GYRO_FS_SEL
            _gres := lookupz(dps >> core#GYRO_FS_SEL: 15_267, 30_487, 60_975, 121_951)
            ' _gres: (1/65.5, 1/32.8, 1/16.4, 1/8.2) = LSB/dps * 1_000_000
        other:
            curr_scl := (curr_scl >> core#GYRO_FS_SEL) & core#GYRO_FS_SEL_BITS
            return lookupz(curr_scl: 500, 1000, 2000, 4000)

    dps := ((curr_scl & core#GYRO_FS_SEL_MASK) | dps) & core#GYRO_CFG1_MASK
    writereg(core#GYRO_CFG1, 1, @dps)

PUB Interrupt{}: flag
' Read interrupt state
'   Bit 3210 (For each bit, 0: No interrupt, 1: Interrupt has been generated)
'       3: Wake-on-motion interrupt
'       2: PLL enabled and ready
'       1: DMP interrupt (INT1)
'       0: I2C master interrupt
    flag := 0
    readreg(core#INT_STATUS, 1, @flag)
    flag &= core#INT_STATUS_MASK

PUB IntMask(mask): curr_mask
' Set interrupt mask
'   Valid values:
'   Bit 3210 (For each bit, 0: disable interrupt, 1: enable interrupt)
'       3: Wake-on-motion interrupt
'       2: PLL enabled and ready
'       1: DMP interrupt (INT1)
'       0: I2C master interrupt
    case mask
        %0000..%1111:
            writereg(core#INT_ENABLE, 1, @mask)
        other:
            curr_mask := 0
            readreg(core#INT_ENABLE, 1, @curr_mask)
            return

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

PUB TempDataReady{}: flag
' Flag indicating new temperature sensor data available
'   Returns TRUE or FALSE
    flag := $00

PUB TempEnabled(enable): curr_state
' Enable the on-chip temperature sensor
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value returns the current setting
    curr_state := 0
    readreg(core#PWR_MGMT_1, 1, @curr_state)
    case ||(enable)
        0, 1:
            enable := (||(enable) ^ 1) << core#TEMP_DIS ' invert logic since
        other:                                  ' reg function is for disabling
            return (((curr_state >> core#TEMP_DIS) & %1) ^ 1) == 1 ' the sensor

    enable := ((curr_state & core#TEMP_DIS_MASK) | enable) & core#PWR_MGMT_1_MASK
    writereg(core#PWR_MGMT_1, 1, @enable)

PUB Temperature{}: temp
' Read temperature, in hundredths of a degree
    temp := 0
    readreg(core#TEMP_OUT_H, 2, @temp)
    temp := (( (temp-_roomtemp_offs) * 1_0000) / 333_87) + 21_00 'XXX unverified
    case _temp_scale
        F:
            return (temp * 9_00 / 5_00) + 32_00
        other:
            return

PUB TempOffset(u8): curr_offs
' Set room temperature offset for Temperature()
'   Valid values: 0..255
'   Any other value returns the current setting
    case u8
        0..255:
            _roomtemp_offs := u8
        other:
            return _roomtemp_offs

PUB TempScale(scale)
' Set temperature scale used by Temperature method
'   Valid values:
'       C (0): Celsius
'       F (1): Fahrenheit
'   Any other value returns the current setting
    case scale
        C, F:
            _temp_scale := scale
        other:
            return _temp_scale

PUB XLGDataRate(Hz): curr_rate
' Set output data rate, in Hz, of accelerometer and gyroscope
'   Valid values:
'   Any other value polls the chip and returns the current setting
    curr_rate := $00

PUB XLGDataReady{}: flag
' Flag indicating new gyroscope/accelerometer data is ready to be read
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
'   NOTE: The update rate of this flag depends upon the GyroDataRate() setting
'       AccelDataRate() has no effect
    readreg(core#INT_STATUS_1, 1, @flag)
    return (flag & %1) == 1

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
            return
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
            repeat tmp from nr_bytes-1 to 0
                byte[ptr_buff][tmp] := i2c.read(tmp == 0)

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
