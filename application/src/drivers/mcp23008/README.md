

<div align=center>
<img src="Document/Image/avento.png" width="400" height="200"/>
</div>

## Library Maters MCP23008 

The MCP23008 is a general purpose programmable 8-Channel digital input/output controller.
Its I2C interface provides several preconfigured I2C addresses while maintaining an ultra-low cost budget. 
The MCP23008 consists of multiple 8-bit configuration registers for input,
output and polarity selection. The system master can enable the I/Os as either inputs or outputs by writing the I/O configuration bits.
The data for each input or output is kept in the corresponding Input or Output register.
The polarity of the Input Port register can be inverted with the Polarity Inversion register. 
All registers can be read by the system master.

The library is the full-function driver of the MCP23008 I/O expansion controller, The driver is written in C language with Code Blocks, It provides the functions to readand write the controller

### Table of Contents

  - [Install](#Install)
  - [Examples](#Examples)
  - [Usage](#Usage)
    - [example basic](#example-basic)
    - [example interface](#example-interface)
  - [Document](#Document)
  - [How to contribute](#Contribute)
  - [License](#License)
  - [Contact Us](#Contact-Us)
  - [Acknowledgements](#Acknowledgements)

  ### Install
  - The interface .C file expects below functions to correctly link the driver 
  ```
    - i2c_initialize function  ( optional )
    - i2c_deinitialize function ( optional )
    - i2c_read function ( Mandatory )
    - i2c_write function ( Mandatory )
    - delay function ( Mandatory )
    - print function (optional )
  ```
  - refer to the video: "Coming soon..."
  - Use example project 



  ### Examples
  - [STM32L432 (STM32CubeIDE)](https://github.com/LibraryMasters/mcp23008/tree/067cdd01a78c673b57248ec95830230b3738f72f/Project%20Example/STM32L434KCU6)
  - [SAMD21G18 (Atmel studio 7)](https://github.com/LibraryMasters/mcp23008/tree/067cdd01a78c673b57248ec95830230b3738f72f/Project%20Example/ATSAMD21G18A/mcp23x08_io_expander_SAMD21)
  ### Usage
  #### example basic

  ```C
#include "mcp23008_driver/mcp23008_driver_basic.h"

/**< interface function definition */
void print(char *const pBuffer, size_t u8Length);
uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint16_t len);
uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint16_t len);
void gpio_toggle_pin_blue(void);
void gpio_toggle_pin_green(void);

typedef enum{                   /**< driver test state chine */

	SET_GPIO_DIR,
	GPIO_WRITE,
	GPIO_READ,
	GPIO_TOGGLE,
	GPIO_EXT_INT,
	PORT_WRITE,

}driver_example_t;

typedef struct test_s{
	driver_example_t state;
}test_t;

test_t test;

uint8_t int_flag;
int btn_press_status;

mcp23008_irq_callback_t mcp23008_irq_cb = mcp23008_basic_irq_handler;		/**< define a callback function for external interrupt */

int main(void)
{
	/* Initializes MCU, drivers and middle-ware */
	atmel_start_init();
	
	 gpio_set_pin_level(mcp23008_reset_pin, true);  /**< make the reset pin on the slave device is constantly high during communication */
	
	 mcp23008_basic_initialize(MCP23008_I2C_ADDRESS_PIN_A110);				/**< initialize chip and set i2c pin level (A2, A1, A0) */
	 mcp23008_info(&mcp23008_handle);

	 mcp23008_interface_debug_print("Chip name :\t%s\n\r", mcp23008_handle.info.chip_name);
	 mcp23008_interface_debug_print("Manufacturer: \t%s\n\r",  mcp23008_handle.info.manufacturer_name);

	 mcp23008_interface_debug_print("Interface: \t%s\n\r",  mcp23008_handle.info.interface);
	 mcp23008_interface_debug_print("Supply voltage max : \t%0.2fV\n\r",  mcp23008_handle.info.supply_voltage_max_v);
	 mcp23008_interface_debug_print("Supply voltage min: \t%0.2fV\n\r",  mcp23008_handle.info.supply_voltage_min_v);
	 mcp23008_interface_debug_print("Maximum current: \t%0.1fmA\n\r",  mcp23008_handle.info.max_current_ma);
	 mcp23008_interface_debug_print("Temperature Max: \t%.1fC\n\r",  mcp23008_handle.info.temperature_max);
	 mcp23008_interface_debug_print("Temperature Min: \t%.1fC\n\r",  mcp23008_handle.info.temperature_min);
	 mcp23008_interface_debug_print("Driver version: \tV%.1f.%.2d\n\r", ( mcp23008_handle.info.driver_version / 1000), (uint8_t)( mcp23008_handle.info.driver_version - (uint8_t)( mcp23008_handle.info.driver_version / 100)*100));
	 
	 //ext_irq_register(PIN_PA24, mcp23008_irq_cb);
	  
	/* Replace with your application code */
	while (1) {
		
		switch((int)test.state)
		{
			
			case SET_GPIO_DIR:
			{
				/**< set GPIO0 and 5, 6 and 7 as output */
				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_7, MCP23008_OUTPUT);
				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_6, MCP23008_OUTPUT);
				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_5, MCP23008_OUTPUT);
				
				/**< set GPIO0 and 0 and 1 as input */
				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_0, MCP23008_INPUT);
				mcp23008_basic_gpio_set_direction(MCP23008_GPIO_PIN_1, MCP23008_INPUT_PULLUP);
					
				break;	
			}

			case GPIO_WRITE:
			{
				/**< Write gpio logic level */
				mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_7, MCP23008_GPIO_HIGH);
				mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_LOW);
				
				break;
			}
			
			case GPIO_READ:
			{
				/**< read gpio pin !!CONCIDER DEBOUNCING */      
				if(mcp23008_basic_gpio_read(MCP23008_GPIO_PIN_1))
				{
					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_5, MCP23008_GPIO_HIGH);	
				}else{
				  	mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_5, MCP23008_GPIO_LOW);	
				}	
				
				btn_press_status = mcp23008_basic_gpio_read(MCP23008_GPIO_PIN_0);
				if(btn_press_status == MCP23008_GPIO_LOW){
					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_HIGH);
				}else{
					mcp23008_basic_gpio_write(MCP23008_GPIO_PIN_6, MCP23008_GPIO_LOW);
				}
				break;
			}
						
			case GPIO_TOGGLE:
			{
				/**< gpio toggle pin */
				mcp23008_basic_gpio_toggle(MCP23008_GPIO_PIN_5);
				mcp23008_interface_delay_ms(500);						
				break;
			}
									
			case GPIO_EXT_INT:
			{
				/**< enable interrupt on GPIO 0 as falling edge, disable interrupt on gpio 1*/
				mcp23008_basic_interrupt_enable(MCP23008_GPIO_PIN_0, MCP23008_interrupt_FALLING_EDGE);
				mcp23008_basic_interrupt_disable(MCP23008_GPIO_PIN_1);	
				
				/**< read interrupt flag status*/
				mcp23008_basic_get_interrupt_flag(MCP23008_GPIO_PIN_0, &int_flag);
				if(int_flag){
					mcp23008_basic_clr_interrupt_flag();
				}			
				
				//mcp23008_basic_gpio_irq_callBack(mcp23008_irq_cb);				/**< interrupt callback function (to be called in the external interrupt callback function) */			
				break;
			}
												
			case PORT_WRITE:
			{
				mcp23008_basic_pin_write_all(MCP23008_GPIO_HIGH);
				mcp23008_interface_delay_ms(500);
				mcp23008_basic_pin_write_all(MCP23008_GPIO_LOW);
				mcp23008_interface_delay_ms(500);												
				break;
			}
			
		}
		
	}
}

  ```

  #### example interface
  
  ```C
  ...

uint8_t mcp23008_interface_i2c_init(void) {
    /*call your i2c initialize function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

uint8_t mcp23008_interface_i2c_deinit(void) {
    /*call your i2c de-initialize function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

uint8_t mcp23008_interface_i2c_read(uint8_t u8Addr, uint8_t *pBuf, uint8_t u8Length) {
    /*call your i2c read function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

uint8_t mcp23008_interface_i2c_write(uint8_t u8Addr, uint8_t *pBuf, uint8_t u8Length) {
    /*call your i2c write function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

void mcp23008_interface_delay_ms(uint32_t U32Ms){
    /*call your delay function here*/
    /*user code begin */

    /*user code end*/
}

void mcp23008_interface_debug_print(const char *const fmt, ...) {
    /*call your call print function here*/
    /*user code begin */
#ifdef MCP23008_DEBUG_MODE
    volatile char str[256];
    volatile uint8_t len;
    va_list args;

    memset((char *) str, 0, sizeof (char)*256);
    va_start(args, fmt);
    vsnprintf((char *) str, 256, (char const *) fmt, args);
    va_end(args);

    len = strlen((char *) str);
//    EUSART1_Write_Text((const char *) str, len);
    (void)printf((uint8_t *)str, len);

    /*user code end*/
#endif
}

  ...
  
  ```

  ### Document
  [datasheet](https://github.com/LibraryMasters/mcp23008/blob/master/Document/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf)
  
  ### Contribute
   1. Clone repo and create a new branch: ```https://github.com/LibraryMasters/mcp23008_PR.git```
   2. Make changes and test
   3. Submit a Pull Request with a comprehensive description of changes
  ### License
  [MIT](https://choosealicense.com/licenses/mit/)
### Contact Us

Email address: cedricmalyam@gmail.com

### Acknowledgements 
- @PeterHenderson https://dribbble.com/peterhenderson for the logo
