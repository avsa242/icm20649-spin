{
    --------------------------------------------
    Filename: sensor.imu.6dof.icm20649.spin
    Author: Jesse Burt
    Description: Driver for the TDK/Invensense ICM20649 6DoF IMU
    Copyright (c) 2022
    Started Aug 28, 2020
    Updated Nov 20, 2022
    See end of file for terms of use.
    --------------------------------------------
}
#include "sensor.accel.common.spinh"
#include "sensor.gyroscope.common.spinh"
#include "sensor.temp.common.spinh"

CON

    SLAVE_WR                = core#SLAVE_ADDR
    SLAVE_RD                = core#SLAVE_ADDR|1

    DEF_SCL                 = 28
    DEF_SDA                 = 29
    DEF_HZ                  = 100_000
    DEF_ADDR_BITS           = 0
    I2C_MAX_FREQ            = core#I2C_MAX_FREQ

    DEF_SCK_FREQ            = 1_000_000

' Indicate to user apps how many Degrees of Freedom each sub-sensor has
'   (also imply whether or not it has a particular sensor)
    ACCEL_DOF               = 3
    GYRO_DOF                = 3
    MAG_DOF                 = 0
    BARO_DOF                = 0
    DOF                     = ACCEL_DOF + GYRO_DOF + MAG_DOF + BARO_DOF

' Scales and data rates used during calibration/bias/offset process
    CAL_XL_SCL              = 4
    CAL_G_SCL               = 500
    CAL_M_SCL               = 0
    CAL_XL_DR               = 200
    CAL_G_DR                = 200
    CAL_M_DR                = 0

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

    long _CS
    word _abias_fact[ACCEL_DOF]
    byte _roomtemp_offs
    byte _addr_bits

OBJ

{ SPI? }
#ifdef ICM20649_SPI
{ decide: Bytecode SPI engine, or PASM? Default is PASM if BC isn't specified }
#ifdef ICM20649_SPI_BC
    spi : "com.spi.25khz.nocog"                       ' BC SPI engine
#else
    spi : "com.spi.4mhz"                ' PASM SPI engine
#endif
#else
{ no, not SPI - default to I2C }
#define ICM20649_I2C
{ decide: Bytecode I2C engine, or PASM? Default is PASM if BC isn't specified }
#ifdef ICM20649_I2C_BC
    i2c : "com.i2c.nocog"                       ' BC I2C engine
#else
    i2c : "com.i2c"                             ' PASM I2C engine
#endif

#endif

    core: "core.con.icm20649"                   ' hw-specific low-level const's
    time: "time"                                ' basic timing functions

PUB null{}
' This is not a top-level object

#ifdef ICM20649_SPI
PUB startx(CS_PIN, SCK_PIN, MOSI_PIN, MISO_PIN): status
' Start using custom I/O pin settings
    if lookdown(CS_PIN: 0..31) and lookdown(SCK_PIN: 0..31) and {
}   lookdown(MOSI_PIN: 0..31) and lookdown(MISO_PIN: 0..31)
        if (status := spi.init(SCK_PIN, MOSI_PIN, MISO_PIN, core#SPI_MODE))
            _CS := CS_PIN
            time.usleep(core#G_START_COLD)      ' wait for device startup
            if (dev_id{} == core#DEVID_RESP)    ' validate device
                { read the factory accel bias }
                accelbias(@_abias_fact[X_AXIS], @_abias_fact[Y_AXIS], @_abias_fact[Z_AXIS], R)
                return
    ' if this point is reached, something above failed
    ' Re-check I/O pin assignments, bus speed, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

#elseifdef ICM20649_I2C

PUB start{}: status
' Start using "standard" Propeller I2C pins and 100kHz
    return startx(DEF_SCL, DEF_SDA, DEF_HZ, DEF_ADDR_BITS)

PUB startx(SCL_PIN, SDA_PIN, I2C_HZ, ADDR_BITS): status
' Start using custom I/O pin settings and bus frequency
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) and {
}   I2C_HZ =< core#I2C_MAX_FREQ
        if (status := i2c.init(SCL_PIN, SDA_PIN, I2C_HZ))
            _addr_bits := (||(ADDR_BITS <> 0)) << 1  ' if not 0, then it's 1
            time.usleep(core#G_START_COLD)      ' wait for device startup
            if (dev_id{} == core#DEVID_RESP)' validate device
                return
    ' if this point is reached, something above failed
    ' Re-check I/O pin assignments, bus speed, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

#endif

PUB stop{}
' Stop the driver
#ifdef ICM20649_SPI
    spi.deinit{}
#elseifdef ICM20649_I2C
    i2c.deinit{}
#endif
    _CS := 0
    wordfill(@_abias_fact, 0, 3)
    _roomtemp_offs := 0

PUB defaults{}
' Set factory defaults
    reset{}

PUB preset_active{}
' Like defaults(), but
'   * powers on sensor
'   * sets scaling factors
    reset{}
    powered(true)
    accel_scale(4)
    gyro_scale(500)
    temp_scale(C)

PUB accel_axis_ena(mask): curr_mask
' Enable data output for accelerometer (all axes)
'   Valid values: %000 (disable) or %001..%111 (enable), for all axes
'       (default: %111)
'   Any other value polls the chip and returns the current setting
'   NOTE: All axes are affected. The mask parameter is used for
'       compatibility with other IMU drivers.
    curr_mask := 0
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case mask
        %000:
            mask := %111 << core#DIS_ACCEL      ' Chip logic is inverse
        %001..%111:                             ' If any bit is set,
            mask := %000                        '   enable the accel
        other:
            curr_mask >>= core#DIS_ACCEL
            curr_mask &= core#DIS_ACCEL_BITS
            return (curr_mask ^ %111)

    mask := (curr_mask & core#DIS_ACCEL_MASK) | mask
    writereg(core#PWR_MGMT_2, 1, @mask)

PUB accel_bias(x, y, z) | tmp[3], tc_bit[3]
' Read accelerometer calibration offset values
'   NOTE: The ICM20649 accelerometer is pre-programmed with offsets, which may
'       or may not be adequate for your application
    { Discrete reads because the three axes aren't contiguous register pairs }
    readreg(core#XA_OFFS_H, 2, @tmp[X_AXIS])
    readreg(core#YA_OFFS_H, 2, @tmp[Y_AXIS])
    readreg(core#ZA_OFFS_H, 2, @tmp[Z_AXIS])

    long[x] := ~~tmp[X_AXIS]
    long[y] := ~~tmp[Y_AXIS]
    long[z] := ~~tmp[Z_AXIS]

PUB accel_set_bias(x, y, z) | tmp[3], tc_bit[3]
' Write accelerometer calibration offset values
    { read the offset regs to get the temp compensation bits }
    readreg(core#XA_OFFS_H, 2, @tmp[X_AXIS])
    readreg(core#YA_OFFS_H, 2, @tmp[Y_AXIS])
    readreg(core#ZA_OFFS_H, 2, @tmp[Z_AXIS])
    tc_bit[X_AXIS] := (tmp[X_AXIS] & 1)
    tc_bit[Y_AXIS] := (tmp[Y_AXIS] & 1)
    tc_bit[Z_AXIS] := (tmp[Z_AXIS] & 1)

    x := (_abias_fact[X_AXIS]-((x / 8) & $FFFE)) | tc_bit[X_AXIS]
    y := (_abias_fact[Y_AXIS]-((y / 8) & $FFFE)) | tc_bit[Y_AXIS]
    z := (_abias_fact[Z_AXIS]-((z / 8) & $FFFE)) | tc_bit[Z_AXIS]

    writereg(core#XA_OFFS_H, 2, @x)
    writereg(core#YA_OFFS_H, 2, @y)
    writereg(core#ZA_OFFS_H, 2, @z)
    return

PUB accel_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Reads the Accelerometer output registers
    readreg(core#ACCEL_XOUT_H, 6, @tmp)
    long[ptr_x] := ~~tmp.word[2]
    long[ptr_y] := ~~tmp.word[1]
    long[ptr_z] := ~~tmp.word[0]

PUB accel_data_rate(rate): curr_rate
' Set accelerometer output data rate, in Hz
'   Valid values: 1..1127 (default: 1127)
'   Any other value polls the chip and returns the current setting
    case rate
        1..1127:
            rate := (1127 / rate) - 1
            writereg(core#ACCEL_SMPLRT_DIV, 2, @rate)
        other:
            curr_rate := 0
            readreg(core#ACCEL_SMPLRT_DIV, 2, @curr_rate)
            return (1127 / (curr_rate + 1))

PUB accel_data_rdy{}: flag
' Flag indicating new accelerometer data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy{}

PUB accel_lpf_freq(freq): curr_freq | lpf_enable
' Set accelerometer output data low-pass filter cutoff frequency, in Hz
'   Valid values: 6, 12, 24, 50, 111, *246, 473
'   Any other value polls the chip and returns the current setting
    curr_freq := lpf_enable := 0
    readreg(core#ACCEL_CFG, 1, @curr_freq)
    case freq
        0:                                      ' Disable/bypass the LPF
            lpf_enable := %0
        6, 12, 24, 50, 111, 246, 473:
            freq := lookdown(freq: 246, 111, 50, 24, 12, 6, 473) << core#ACCEL_DLPFCFG
            lpf_enable := 1
        other:
            if (curr_freq & 1) <> 1
                return 0                        ' LPF bypass bit set; return 0
            else
                curr_freq := (curr_freq >> core#ACCEL_DLPFCFG) & core#ACCEL_DLPFCFG_BITS
                return lookup(curr_freq: 246, 111, 50, 24, 12, 6, 473)

    freq := (curr_freq & core#ACCEL_DLPFCFG_MASK & core#ACCEL_FCH_MASK) | {
}   freq | lpf_enable
    writereg(core#ACCEL_CFG, 1, @freq)

PUB accel_opmode(mode): curr_mode
' Set accelerometer operating mode
'   Valid values:
'      *NORMAL (0): Normal operating mode
'       LOWPWR (1): Low-power mode
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
    readreg(core#LP_CONFIG, 1, @curr_mode)
    case mode
        LOWPWR, NORMAL:
            mode <<= core#ACCEL_CYCLE
        other:
            return ((curr_mode >> core#ACCEL_CYCLE) & 1)

    mode := (curr_mode & core#ACCEL_CYCLE_MASK) | mode
    writereg(core#LP_CONFIG, 1, @mode)

PUB accel_scale(scale): curr_scl
' Set accelerometer full-scale range, in g's
'   Valid values: *4, 8, 16, 30
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#ACCEL_CFG, 1, @curr_scl)
    case scale
        4, 8, 16, 30:
            scale := lookdownz(scale: 4, 8, 16, 30) << core#ACCEL_FS_SEL
            _ares := lookupz(scale >> core#ACCEL_FS_SEL: 0_000122, 0_000244, 0_000488, 0_000976)
            ' 1/8192 (LSB per g), 1/4096, 1/2048, 1/1024 * 1_000_000
        other:
            curr_scl := (curr_scl >> core#ACCEL_FS_SEL) & core#ACCEL_FS_SEL_BITS
            return lookupz(curr_scl: 4, 8, 16, 30)

    scale := ((curr_scl & core#ACCEL_FS_SEL_MASK) | scale) & core#ACCEL_CFG_MASK
    writereg(core#ACCEL_CFG, 1, @scale)

PUB clock_src(src): curr_src
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
            return (curr_src & core#CLKSEL_BITS)

    src := (curr_src & core#CLKSEL_MASK) | src
    writereg(core#PWR_MGMT_1, 1, @src)

PUB dev_id{}: id
' Read device identification
    id := 0
    readreg(core#WHO_AM_I, 1, @id)

PUB fifo_ena(state): curr_state
' Enable the FIFO
'   Valid values: TRUE (-1 or 1), *FALSE (0)
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
            return (((curr_state >> core#FIFO_EN) & 1) == 1)

PUB fifo_full{}: flag
' Flag indicating FIFO is full
'   Returns: TRUE (-1) if FIFO is full, FALSE (0) otherwise
'   NOTE: If this flag is set, the oldest data has already been dropped from the FIFO
    readreg(core#INT_STATUS_2, 1, @flag)
    return (((flag >> core#FIFO_OVRFL_INT) & 1) == 1)

PUB fifo_mode(mode): curr_mode
' Set FIFO mode
'   Valid values:
'       BYPASS (2): FIFO disabled
'      *STREAM (0): FIFO enabled; when full, new data overwrites old data
'       FIFO (15): FIFO enabled; when full, no new data will be written to FIFO
'   Any other value polls the chip and returns the current setting
'   NOTE: If no data sources are set using FIFOSource(), the current mode returned will be BYPASS (0), regardless of what the mode was previously set to
    curr_mode := 0
    readreg(core#FIFO_MODE, 1, @curr_mode)
    case mode
        BYPASS:                                 ' If bypassing the FIFO, turn
            fifo_src(%00000000)               '   off FIFO data collection
            return
        STREAM, FIFO:
        other:
            curr_mode := curr_mode & 1
            if fifo_src(-2)                   ' If there's a mask set with FIFOSource(), return
                return                          '   either STREAM or FIFO as the current mode
            else
                return BYPASS                   ' If not, anything besides 0 (BYPASS) doesn't really matter or make sense
    mode := ((curr_mode & core#FIFO_MODE_MASK) | mode)
    writereg(core#FIFO_MODE, 1, @mode)

PUB fifo_read(nr_bytes, ptr_data)
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

PUB fifo_src(mask): curr_mask
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

PUB fifo_nr_unread{}: nr_samples
' Number of unread samples stored in FIFO
'   Returns: unsigned 13bit
    readreg(core#FIFO_COUNTH, 2, @nr_samples)

PUB gyro_axis_ena(mask): curr_mask
' Enable data output for gyroscope (all axes)
'   Valid values: %000 (disable) or %001..%111 (enable), for all axes
'       (default: %111)
'   Any other value polls the chip and returns the current setting
'   NOTE: All axes are affected. The mask parameter is used for
'       compatibility with other IMU drivers.
    curr_mask := 0
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case mask
        %000:
            mask := %111                        ' Chip logic is inverse
        %001..%111:                             ' If any bit is set,
            mask := %000                        '   enable the gyro
        other:
            curr_mask &= core#DIS_GYRO_BITS
            return (curr_mask ^ %111)

    mask := ((curr_mask & core#DIS_GYRO_MASK) | mask)
    writereg(core#PWR_MGMT_2, 1, @mask)

PUB gyro_bias(x, y, z) | tmp[3]
' Read gyroscope calibration offset values
'   x, y, z: pointers to copy offsets to
    longfill(@tmp, 0, GYRO_DOF)
    readreg(core#XG_OFFS_USRH, 2, @tmp[X_AXIS])
    readreg(core#YG_OFFS_USRH, 2, @tmp[Y_AXIS])
    readreg(core#ZG_OFFS_USRH, 2, @tmp[Z_AXIS])
    long[x] := ~~tmp[X_AXIS]
    long[y] := ~~tmp[Y_AXIS]
    long[z] := ~~tmp[Z_AXIS]

PUB gyro_set_bias(x, y, z)
' Write gyroscope calibration offset values
'   Valid values:
'   -32768..32767 (clamped to range)
    x := -((-32768 #> x <# 32767) / 4)
    y := -((-32768 #> y <# 32767) / 4)
    z := -((-32768 #> z <# 32767) / 4)

    writereg(core#XG_OFFS_USRH, 2, @x)
    writereg(core#YG_OFFS_USRH, 2, @y)
    writereg(core#ZG_OFFS_USRH, 2, @z)
    return

PUB gyro_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read the Gyroscope output registers
    tmp := 0
    readreg(core#GYRO_XOUT_H, 6, @tmp)
    long[ptr_x] := ~~tmp.word[2]
    long[ptr_y] := ~~tmp.word[1]
    long[ptr_z] := ~~tmp.word[0]

PUB gyro_data_rate(rate): curr_rate
' Set gyroscope output data rate, in Hz
'   Valid values: 1..1100 (default: 1100)
'   Any other value polls the chip and returns the current setting
    case rate
        1..1100:
            rate := (1100 / rate) - 1
            writereg(core#GYRO_SMPLRT_DIV, 1, @rate)
        other:
            curr_rate := 0
            readreg(core#GYRO_SMPLRT_DIV, 1, @curr_rate)
            return 1100 / (curr_rate + 1)

PUB gyro_data_rdy{}: flag
' Flag indicating new gyroscope data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy{}

PUB gyro_lpf_freq(freq): curr_freq | lpf_enable
' Set gyroscope output data low-pass filter cutoff frequency, in Hz
'   Valid values: 6, 12, 24, 51, 120, 152, 197, 361, *12106 (LPF disabled)
'   Any other value polls the chip and returns the current setting
    curr_freq := lpf_enable := 0
    readreg(core#GYRO_CFG1, 1, @curr_freq)
    case freq
        0, 12106:                               ' Disable/bypass the LPF
            lpf_enable := 0
        6, 12, 24, 51, 120, 152, 197, 361:
            freq := lookdownz(freq: 197, 152, 120, 51, 24, 12, 6, 361) << core#GYRO_DLPFCFG
            lpf_enable := 1
        other:
            if (curr_freq & 1) <> 1
                return 12106                    ' LPF bypass set; return 12106
            else
                curr_freq := (curr_freq >> core#GYRO_DLPFCFG) & core#GYRO_DLPFCFG_BITS
                return lookupz(curr_freq: 197, 152, 120, 51, 24, 12, 6, 361)

    freq := (curr_freq & core#GYRO_DLPFCFG_MASK & core#GYRO_FCH_MASK) | freq | lpf_enable
    writereg(core#GYRO_CFG1, 1, @freq)

PUB gyro_scale(scale): curr_scl
' Set gyroscope full-scale range, in degrees per second
'   Valid values: *500, 1000, 2000, 4000
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#GYRO_CFG1, 1, @curr_scl)
    case scale
        500, 1000, 2000, 4000:
            scale := lookdownz(scale: 500, 1000, 2000, 4000) << core#GYRO_FS_SEL
            _gres := lookupz(scale >> core#GYRO_FS_SEL: 15_267, 30_487, 60_975, 121_951)
            ' _gres: (1/65.5, 1/32.8, 1/16.4, 1/8.2) = LSB/scale * 1_000_000
        other:
            curr_scl := (curr_scl >> core#GYRO_FS_SEL) & core#GYRO_FS_SEL_BITS
            return lookupz(curr_scl: 500, 1000, 2000, 4000)

    scale := ((curr_scl & core#GYRO_FS_SEL_MASK) | scale) & core#GYRO_CFG1_MASK
    writereg(core#GYRO_CFG1, 1, @scale)

PUB interrupt{}: flag
' Read interrupt state
'   Bit 3210 (For each bit, 0: No interrupt, 1: Interrupt has been generated)
'       3: Wake-on-motion interrupt
'       2: PLL enabled and ready
'       1: DMP interrupt (INT1)
'       0: I2C master interrupt
    flag := 0
    readreg(core#INT_STATUS, 1, @flag)
    flag &= core#INT_STATUS_MASK

PUB int_mask(mask): curr_mask
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

PUB powered(state): curr_state
' Enable device power
'   Valid values: TRUE (-1 or 1), *FALSE (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(core#PWR_MGMT_1, 1, @curr_state)
    case ||(state)
        0, 1:
            state := (||(state) ^ 1) << core#SLEEP
        other:
            return (((curr_state >> core#SLEEP) & 1) == 0)

    state := (curr_state & core#SLEEP_MASK) | state
    writereg(core#PWR_MGMT_1, 1, @state)

PUB reset{} | tmp
' Reset the device
    tmp := (1 << core#DEV_RESET)
    writereg(core#PWR_MGMT_1, 1, @tmp)
    time.usleep(core#G_START_COLD)

PUB temp_data{}: adc_word
' Read temperature ADC data
    adc_word := 0
    readreg(core#TEMP_OUT_H, 2, @adc_word)

PUB temp_data_rdy{}: flag
' Flag indicating new temperature sensor data available
'   Returns TRUE or FALSE
'   NOT IMPLEMENTED (dummy method for API compatibility only)
    flag := 0

PUB temp_ena(state): curr_state
' Enable the on-chip temperature sensor
'   Valid values: *TRUE (-1 or 1), FALSE (0)
'   Any other value returns the current setting
    curr_state := 0
    readreg(core#PWR_MGMT_1, 1, @curr_state)
    case ||(state)
        0, 1:
            ' invert logic; set bit disables the sensor, clear enables
            state := (||(state) ^ 1) << core#TEMP_DIS
        other:
            return ((((curr_state >> core#TEMP_DIS) & 1) ^ 1) == 1)

    state := ((curr_state & core#TEMP_DIS_MASK) | state) & core#PWR_MGMT_1_MASK
    writereg(core#PWR_MGMT_1, 1, @state)

PUB temp_rmtemp_offset(u8): curr_offs
' Set room temperature offset for temperature()
'   Valid values: 0..255
'   Any other value returns the current setting
    case u8
        0..255:
            _roomtemp_offs := u8
        other:
            return _roomtemp_offs

PUB temp_word2deg(adc_word): temp
' Convert temperature ADC word to degrees in chosen scale
    temp := (( (adc_word-_roomtemp_offs) * 1_0000) / 333_87) + 21_00 'XXX unverified
    case _temp_scale
        F:
            return (temp * 9_00 / 5_00) + 32_00
        other:
            return

PUB xlg_data_rdy{}: flag
' Flag indicating new gyroscope/accelerometer data is ready to be read
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
'   NOTE: The update rate of this flag depends upon the gyro_data_rate() setting
'       accel_data_rate() has no effect
    readreg(core#INT_STATUS_1, 1, @flag)
    return ((flag & 1) == 1)

PRI bank_sel(bank_nr) | cmd_pkt
' Select ICM20649 register bank
'   Valid values: 0..3
'   Any other value is ignored
    if lookdown(bank_nr: 0..3)
#ifdef ICM20649_SPI
        cmd_pkt.byte[0] := core#REG_BANK_SEL
        cmd_pkt.byte[1] := bank_nr << core#USER_BANK
        outa[_CS] := 0
        spi.wrblock_lsbf(@cmd_pkt, 2)
        outa[_CS] := 1
#elseifdef ICM20649_I2C
        cmd_pkt.byte[0] := SLAVE_WR
        cmd_pkt.byte[1] := core#REG_BANK_SEL
        cmd_pkt.byte[2] := bank_nr << core#USER_BANK
        i2c.start{}
        i2c.wrblock_lsbf(@cmd_pkt, 3)
        i2c.stop{}
#endif

PRI readreg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt, tmp[2], i
' Read nr_bytes from the slave device
    case reg_nr                                 ' validate reg #
        core#ACCEL_XOUT_H..core#TEMP_OUT_H:     ' prioritize output data regs
#ifdef ICM20649_SPI
            cmd_pkt.byte[0] := reg_nr | core#R
            outa[_CS] := 0
            spi.wr_byte(cmd_pkt)
            spi.rdblock_lsbf(@tmp, nr_bytes)
            outa[_CS] := 1
            repeat i from 0 to nr_bytes-1
                byte[ptr_buff][i] := tmp.byte[nr_bytes-1-i]
#elseifdef ICM20649_I2C
            cmd_pkt.byte[0] := SLAVE_WR | _addr_bits
            cmd_pkt.byte[1] := reg_nr
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}
            i2c.wr_byte(SLAVE_RD | _addr_bits)
            i2c.rdblock_msbf(ptr_buff, nr_bytes, i2c.NAK)
            i2c.stop{}
#endif
            return
        $000, $003, $005, $006, $007, $00f..$013, $017, $019..$01c, $028,{
        } $029, $03b..$052, $066..$069, $070..$072, $074, $076, $102, $103,{
        } $104, $10e..$110, $114, $115, $117, $118, $11a, $11b, $128,{
        } $200..$209, $210..$215, $252..$254, $300..$317:
            bank_sel(reg_nr.byte[1])
#ifdef ICM20649_SPI
            cmd_pkt.byte[0] := reg_nr.byte[0] | core#R
            outa[_CS] := 0
            spi.wr_byte(cmd_pkt)
            spi.rdblock_lsbf(@tmp, nr_bytes)
            outa[_CS] := 1
            repeat i from 0 to nr_bytes-1
                byte[ptr_buff][i] := tmp.byte[nr_bytes-1-i]
#elseifdef ICM20649_I2C
            cmd_pkt.byte[0] := SLAVE_WR | _addr_bits
            cmd_pkt.byte[1] := reg_nr.byte[0]   ' Actual reg # is lower 8 bits
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)

            i2c.start{}
            i2c.wr_byte(SLAVE_RD | _addr_bits)
            i2c.rdblock_msbf(ptr_buff, nr_bytes, i2c.NAK)
            i2c.stop{}
#endif
            bank_sel(0)
        other:
            return

PRI writereg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt, tmp, i
' Write nr_bytes to the slave device
    case reg_nr                                 ' validate reg #
        $003, $005, $006, $007, $00f..$013, $066..$069, $072, $076, $102,{
        } $103, $104, $10e..$110, $114, $115, $117, $118, $11a, $11b, $128,{
        } $200..$209, $210..$215, $252..$254, $300..$316:
            bank_sel(reg_nr.byte[1])
#ifdef ICM20649_SPI
            repeat i from 0 to nr_bytes-1
                tmp.byte[i] := byte[ptr_buff][nr_bytes-1-i]

            cmd_pkt.byte[0] := reg_nr.byte[0]
            outa[_CS] := 0
            spi.wr_byte(cmd_pkt)
            spi.wrblock_lsbf(@tmp, nr_bytes)
            outa[_CS] := 1
#elseifdef ICM20649_I2C
            cmd_pkt.byte[0] := SLAVE_WR | _addr_bits
            cmd_pkt.byte[1] := reg_nr.byte[0]   ' Actual reg # is lower 8 bits
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.wrblock_msbf(ptr_buff, nr_bytes)
            i2c.stop{}
#endif
            bank_sel(0)
        other:
            return

DAT
{
Copyright 2022 Jesse Burt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}

