=============
To-Do
=============

------------
Drivers
------------

- Customize MS5611 driver to remove any Arduino libs.
- Implement MCP23008 IO functions
- Implement BNO055 IO functions
- Implement IIS2DPLC IO functions (and handle the two sensors !)
- Build GNSS driver from provided ST examples and drivers

-----------
Threads
-----------

- Implement threading management and init
    - Broke some links, restore them !

-----------
SAADC
-----------

- Implement mean per channel (and output to dbg com port)
- Python script to fetch every output and write it into a files
- Find max Fs for 8 all channels ?
    --> GBF ?