=====================================
notes to understand how this is done 
======================================

z_fetched : Base usefull folder
- teseo_liv3f : 
    - Base functions of the teseo module, define func ptr* to IO operations

- LibNMEA : 
    - Parse NMEA messages to standard ones.

- LibGNSS : 
    - Main GNSS lib

- wrapper :
    - Handle IO operations --> Shall be overidden ! (No read ?)
    - Get Message ?

To change : 
- RTOS selection, shall add zephyr IO op.


To do :
- GNSS1A1_GNSS_Init 
- GNSS_PARSER_Init
- Read and parse messages ?
- COmmand sending. In the app example, command is copied from one UART to the other !
