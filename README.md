# 

$idf.py build


## Hardware required :
Option 1:
PC (Modbus Slave app) + USB Serial adapter connected to USB port + RS485 line drivers + ESP32 WROVER-KIT board. 

Option 2:
Several ESP32 WROVER-KIT board flashed with modbus_slave example software to represent slave device with specific slave address (See CONFIG_MB_SLAVE_ADDR). The slave addresses for each board have to be configured as defined in "connection schematic" above.
One ESP32 WROVER-KIT board flashed with modbus_master example. All the boards require connection of RS485 line drivers (see below).

The MAX485 line driver is used as an example below but other similar chips can be used as well.
RS485 example circuit schematic for connection of master and slave devices into segment:
```
         VCC ---------------+                               +--------------- VCC
                            |                               |
                    +-------x-------+               +-------x-------+
         RXD <------| RO            | DIFFERENTIAL  |             RO|-----> RXD
                    |              B|---------------|B              |
         TXD ------>| DI   MAX485   |    \  /       |    MAX485   DI|<----- TXD
ESP32 WROVER KIT 1  |               |   RS-485 side |               |      External PC (emulator) with USB to serial or
         RTS --+--->| DE            |    /  \       |             DE|---+  ESP32 WROVER KIT 2 (slave)     
               |    |              A|---------------|A              |   |
               +----| /RE           |    PAIR       |            /RE|---+-- RTS
                    +-------x-------+               +-------x-------+
                            |                               |
                           ---                             --- 
                    Modbus Master device             Modbus Slave device
                           
```

## How to setup and use an example:

### Configure the application
Start the command below to setup configuration:
```
idf.py menuconfig
```
Configure the UART pins used for modbus communication using and table below.
Define the communication mode parameter for master and slave in Kconfig - CONFIG_MB_COMM_MODE (must be the same for master and slave devices in one segment).
Configure the slave address for each slave in the Modbus segment (the CONFIG_MB_SLAVE_ADDR in Kconfig).
```
  --------------------------------------------------------------------------------------------------------------------------
  | ESP32 Interface       | #define            | Default ESP32 Pin     | Default ESP32-S2 Pins | External RS485 Driver Pin |
  | ----------------------|--------------------|-----------------------|-----------------------|---------------------------|
  | Transmit Data (TxD)   | CONFIG_MB_UART_TXD | GPIO23                | GPIO20                | DI                        |
  | Receive Data (RxD)    | CONFIG_MB_UART_RXD | GPIO22                | GPIO19                | RO                        |
  | Request To Send (RTS) | CONFIG_MB_UART_RTS | GPIO18                | GPIO18                | ~RE/DE                    |
  | Ground                | n/a                | GND                   | GND                   | GND                       |
  --------------------------------------------------------------------------------------------------------------------------
```
Note: The GPIO22 - GPIO25 can not be used with ESP32-S2 chip because they are used for flash chip connection. Please refer to UART documentation for selected target.

Connect USB to RS485 adapter to computer and connect its D+, D- output lines with the D+, D- lines of RS485 line driver connected to ESP32 (See picture above).

The communication parameters of Modbus stack allow to configure it appropriately but usually it is enough to use default settings.
See the help string of parameters for more information.