{
    --------------------------------------------
    Filename: core.con.icm20649.spin
    Author: Jesse Burt
    Description: Low-level constants
    Copyright (c) 2020
    Started Aug 28, 2020
    Updated Aug 28, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    I2C_MAX_FREQ        = 400_000
    SLAVE_ADDR          = $68 << 1
    TPOR                = 1_000                 ' usec

' Register definitions
    WHO_AM_I            = $00
        DEVID_RESP      = $E1

PUB Null
'' This is not a top-level object
