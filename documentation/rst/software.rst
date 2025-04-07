================
**Software**
================

Now, let's describe how the software is written and organized.

----------------------
Peripherals bindings
----------------------

Since we're using the devicetree to enable and configure the main 
part of the peripherals, we need to work with it once in the C land.

First, we need to include the driver in the C code :

.. code::

    #include <zephyr/drivers/pwm.h>

And then, fetch the entity :

.. code::

    #define SERVO_0     DT_ALIAS(servo1)

.. note::

    This entity name shall be passed as the name without \"\", and must
    correspond to the aliases name you gave on the topaze-aliases.dtsi file.

Then, you fetch the struct using another macro :

.. code::

    static const struct pwm_dt_spec     pwm0_servo0    = PWM_DT_SPEC_GET(SERVO_0);

.. note:: 

    You gain by declaring theses struct static const, they will then be resolved by
    GCC as a single address more than a variable. Thus, using them as global variables 
    won't cause anything.

.. note::

    If you're dealing with multiple threads, just make sure to not use them in parallel.

----------------------------
Peripheral initialization
----------------------------

Once the peripheral were bound to the C code, you need to ensure they're correctly initialized.
(To be complete, this is the job of the do the job, according to the settings you passed on the device tree).

This is done by the function below, that check for the leds.

.. code::

    int CheckLedsPeripherals();

Check the return code, and you're fine !
You can now interract wit the peripheral.

.. note::

    All of this stuff is done on the application/src/init/init.h and application/src/init/init.c files !


