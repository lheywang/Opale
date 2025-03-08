#include <atmel_start.h>
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

void print(char *const pBuffer, size_t u8Length)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);

	io_write(io, (char *)pBuffer, u8Length);
	
}

uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint16_t len)
{
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, addr, I2C_M_SEVEN);
	io_read(I2C_0_io, buf,  len);
	return 0;
}

uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint16_t len)
{
	struct io_descriptor *I2C_0_io;
	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, addr, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)buf, len);
	return 0;
}

void gpio_toggle_pin_green(void)
{
	gpio_toggle_pin_level(user_led_green);
}

void gpio_toggle_pin_blue(void)
{
	gpio_toggle_pin_level(user_led_blue);
}