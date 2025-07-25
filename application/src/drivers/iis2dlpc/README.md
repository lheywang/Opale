![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/iis2dlpc-pid.svg?color=brightgreen)

# 1 - Introduction

Sensor driver for IIS2DLPC sensor written in C programming language. This repository contains the sensor driver files (.h and .c) to be included, or linked directly as a git submodule, in your project. The driver is MISRA compliant and the documentation can be generated using the [Doxygen](http://www.doxygen.org/) tool.

In order to `clone` the complete content of the repository folder, use the command:

```
git clone https://github.com/STMicroelectronics/IIS2DLPC-PID/
```

Some examples of driver usage can be found [here](https://github.com/STMicroelectronics/STMems_Standard_C_drivers).

---

# 2 - Integration details

The driver is platform-independent, you only need to define two functions for read and write transactions from the sensor hardware bus (ie. SPI or I²C) and an optional one to implement a delay of millisecond granularity. **A few devices integrate an extra bit in the communication protocol in order to enable multi read/write access, this bit must be managed in the read and write functions defined by the user.** Please refer to the read and write implementation in the [reference examples](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/iis2dlpc_STdC/examples).

### 2.a Source code integration

- Include in your project the driver files of the sensor (.h and .c)
- Define in your code the read and write functions that use the I²C or SPI platform driver like the following:

```
/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)

/** Optional (may be required by driver) **/
void platform_delay(uint32_t millisec)
```

- Declare and initialize the structure of the device interface:

```
xxxxxxx_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
dev_ctx.write_reg = platform_write;
dev_ctx.read_reg = platform_read;
dev_ctx.mdelay = platform_delay;

```

- If needed by the platform read and write functions, initialize the handle parameter:

```
dev_ctx.handle = &platform_handle;
```

Some integration examples can be found [here](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/iis2dlpc_STdC/examples).

### 2.b Required properties

> - A standard C language compiler for the target MCU
> - A C library for the target MCU and the desired interface (ie. SPI, I²C)

---

**More Information: [http://www.st.com](http://st.com/MEMS)**

**Copyright (C) 2021 STMicroelectronics**
