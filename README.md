# icm20649-spin 
---------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for the ICM20649 6DoF IMU.

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P). Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.


## Salient Features

* I2C connection at up to 400kHz
* SPI connection at a fixed 4MHz (P1), up to 7MHz (P2)
* Read accelerometer (raw, micro-g's), gyroscope (raw, micro-dps), temperature (centi-degrees)
* Set accel, gyro full-scale
* Data-ready flag
* Set bias offsets (manual, automatic)
* Set output data rates
* Set optional accel/gyro data low-pass filter
* FIFO modes, status flags


## Requirements

P1/SPIN1:
* spin-standard-library
* 1 extra core/cog for the PASM I2C or SPI engine (none if the bytecode engine is used)
* sensor.accel.common.spinh (provided by spin-standard-library)
* sensor.gyroscope.common.spinh (provided by spin-standard-library)
* sensor.temp.common.spinh (provided by spin-standard-library)

P2/SPIN2:
* p2-spin-standard-library
* sensor.accel.common.spin2h (provided by p2-spin-standard-library)
* sensor.gyroscope.common.spin2h (provided by p2-spin-standard-library)
* sensor.temp.common.spin2h (provided by p2-spin-standard-library)


## Compiler Compatibility

| Processor | Language | Compiler               | Backend      | Status                |
|-----------|----------|------------------------|--------------|-----------------------|
| P1        | SPIN1    | FlexSpin (6.9.4)       | Bytecode     | OK                    |
| P1        | SPIN1    | FlexSpin (6.9.4)       | Native/PASM  | Some runtime issues   |
| P2        | SPIN2    | FlexSpin (6.9.4)       | NuCode       | OK                    |
| P2        | SPIN2    | FlexSpin (6.9.4)       | Native/PASM2 | OK                    |

(other versions or toolchains not listed are __not supported__, and _may or may not_ work)


## Limitations

* TBD

