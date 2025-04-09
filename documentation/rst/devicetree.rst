===========================
**Device trees**
===========================

Devices tree describe all of the hardware in a specific environement.

Zephyr RTOS use them for advanced peripheral configuration prior 
to the user code execution, and advanced security.

Let's dig into the boards/topaze folder !

---------------------------
**Files**
---------------------------
First, there is a lot of files in this folder ! Each one has it's own usage...

#. Generated files

 * board.cmake 
 * board.yml 
 * Kconfig.defconfig 
 * Kconfig.topaze
 * pre_dt_board.cmake

#. Library config files

 * topaze_nrf5340_cpuapp_defconfig
 * topaze_nrf5340_cpuapp_ns_defconfig
 * topaze_nrf5340_cpunet_defconfig
 
#. Description files

 * topaze_nrf5340_cpuapp.yml
 * topaze_nrf5340_cpuapp_ns.yml
 * topaze_nrf5340_cpunet.yml

#. Device tree files

 * topaze_nrf5340_cpuapp.dts
 * topaze_nrf5340_cpuapp_ns.dts
 * topaze_nrf5340_cpunet.dts
 * topaze_cpuapp_partitionning.Description
 * DTSI/topaze-pinctrl.dtsi 
 * topaze-shared_sram.dtsi 

First, generated files should not be modified by any one, 
they describe mostly arguments and parameters related to the buid system,
to match the system specifications / capabilities. 

Secondly, Library config files are quite important. 
Theses enable the possibility to include when compiling the image 
somes files (=drivers !). You can see that we enable PWM for example.

Third, there is Description files. They describe some of the hardware specs 
of the DTS, and are mostly used for checking the inputs on the DTS files.

Finally, Devicetree files.

There is two types of them :

* .dts
* .dtsi

^^^^^^^^^^^^^^^^^^^^^^^^^^
**DTS**
^^^^^^^^^^^^^^^^^^^^^^^^^^
This first file is the base device tree file. The compiler point on 
theses files for know the hardware.

Theses are generated, and pretty short since they include most 
of their final content for DTSI files.

^^^^^^^^^^^^^^^^^^^^^^^^^^
**DTSI**
^^^^^^^^^^^^^^^^^^^^^^^^^^
Theses file can only be included, and contain some "patches" for the 
base DTS file.

Theses user editable files are located on the boards/topaze/DTSI folder.
All others shall not be modified by the user.

I've used an multi file architecture to make file easier to read.

---------------------------------------
**How to configure a new peripheral**
---------------------------------------
To add a new peripheral, first create it's .dtsi file in the ../DTSI folder.
Name it topaze-[peripheral name].dtsi.

Don't forget to include it on the topaze-pinctrl file !

Then, enable and configure the peripheral on the device tree.
For example, this may be enough :

.. code:: 

    &gpio0 {
        status = "okay";
    };

.. warning:: 

    Some peripheral may require more configuration ! 
    Check the documentation to ensure that need.

    If not, you will most likely end up with a DTC compiler error.

Then, you can add a pinctrl properties to handle some pin muxing, if needed (most likely yes !)
This is done with the following snipplet :

.. code::

    &pinctrl {
        leds {
            compatible = "gpio-leds";
            led0: led_0 {
                gpios = <&gpio0 28 GPIO_ACTIVE_LOW>;
                label = "Green LED 0";
            };
        };
    };

Which declare a led controller with some specified parameters. 

.. note::
    You're doing here pin configuration, not a peripheral configuration.
    You only think about external signals for this peripheral, not the internals.

Then, it may sometimes be needed to add a root property. For example, for the PWM peripheral :

.. code::

    /{
        pwmleds {
            compatible = "pwm-leds";
            servo1: pwm_led_0 {
                pwms = <&pwm0 0 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
            };
        };
    };

.. note::

    You're configuring some behavior of the peripheral here, for a single pin.

Now, enable the driver for Zephyr build, in the \*\_defconfig file, for example 
here we enable the pwm driver

.. code::

    # enable uart driver
    CONFIG_PWM=y

.. note::

    If you're not doing this, you will be able to compile the code, and the peripheral
    will operate in the default state, but you won't be able to interact with it.

And to conclude, don't forget to aliase youre node into the topaze-aliases.dtsi file,
nor you won't be able to call your peripheral from the main file !
(And sometimes a bindings)

-----------------------
**Bindings**
-----------------------

In some specific cases, you may need to create a new binding under the boards/topaze/dts/bindings folder.
Theses are used by the DTC compiler to ensure the required properties are present on the device tree, and 
thus validate the compilation of the device tree.

For example, this is done for the pwm-servos where we want to add two more field on the device tree :

.. code::

    description: PWM Servos parent node

    compatible: "pwm-servo"

    child-binding:
    description: PWM servo child node
    properties:
        pwms:
            required: true
            type: phandle-array
            description: ...

        max-pulse:
            required: false
            type: int
            description: ...


We can specify here any number of properties, and if they're needed, or not !

--------------------------------------
**Getting peripherals from C  / C++**
--------------------------------------
The last step is on the compiled software. To fetch a peripheral structure, 
there is package, included on init/init.h file.

This file expose some functions that can be called from anywhere in the code,
and return a pointer to the struct.
AND, theses functions included a mutex that is threadproof, to ensure the peripheral
is only accessed from one thread at a time.

This may look like :

.. code::

    gpio_dt_spec *peripheral_reset = INIT_GetAGPIO(GPIOS::PERIPHERAL_RESET);
    pwm_dt_spec *pwm_wings = INIT_GetAPWM(PWMS::SERVOS);
    pwm_dt_spec *pwm_rgb = INIT_GetAPWM(PWMS::RGB);

You may notice the enums (PWMS::RGB) for examples that are used to request THIS peripheral.
This leave the real peripheral used at the discretion of the device tree.

The opposite functions are also possibles : INIT_FreeA[name].