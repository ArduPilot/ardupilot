
#ifndef __AP_HAL_YUNEEC_I2CDRIVER_H__
#define __AP_HAL_YUNEEC_I2CDRIVER_H__

#include <AP_HAL_YUNEEC.h>

class YUNEEC::YUNEECI2CDriver : public AP_HAL::I2CDriver {
public:
    YUNEECI2CDriver(I2C_TypeDef* i2c, GPIO_TypeDef* port,
    				const uint32_t i2cClk,	const uint32_t portClk,
					const uint16_t scl_bit, const uint16_t sda_bit,
					const uint8_t scl_pinSource, const uint8_t sda_pinSource,
					AP_HAL::Semaphore* semaphore);

    void begin();
    void end();
    void setTimeout(uint16_t ms);
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);
    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
	// Specify info of I2C port
	struct I2C_Info {
		I2C_TypeDef* 	i2c;
		GPIO_TypeDef*	port;
		const uint32_t	i2cClk;
		const uint32_t	portClk;
		const uint16_t 	scl_bit;
		const uint16_t 	sda_bit;
		const uint8_t 	scl_pinSource;
		const uint8_t 	sda_pinSource;
	} _i2c_info;

    AP_HAL::Semaphore* _semaphore;
    uint8_t _lockup_count;
    bool _ignore_errors;
    uint16_t _timeout;

    static void _i2c_bus_reset(struct I2C_Info &i2c_info);
    static void _i2c_config(struct I2C_Info &i2c_info);
};

//----------------------------------------------------------------------------
// I2C Instance
//----------------------------------------------------------------------------
#define I2C1_PORT				GPIOB
#define I2C1_CLK				RCC_APB1Periph_I2C1
#define I2C1_GPIOCLK			RCC_AHBPeriph_GPIOB
#define I2C1_SCL_BIT			GPIO_Pin_6
#define I2C1_SDA_BIT			GPIO_Pin_7
#define I2C1_SCL_PINSOURCE 		GPIO_PinSource6
#define I2C1_SDA_PINSOURCE 		GPIO_PinSource7

#define I2C2_PORT				GPIOF
#define I2C2_CLK				RCC_APB1Periph_I2C2
#define I2C2_GPIOCLK			RCC_AHBPeriph_GPIOF
#define I2C2_SCL_BIT			GPIO_Pin_6
#define I2C2_SDA_BIT			GPIO_Pin_7
#define I2C2_SCL_PINSOURCE 		GPIO_PinSource6
#define I2C2_SDA_PINSOURCE 		GPIO_PinSource7

#define YUNEECI2CDriverInstance(I2Cx, semaphore)                            	\
YUNEECI2CDriver I2Cx##Driver((I2C_TypeDef*) I2Cx, (GPIO_TypeDef*) I2Cx##_PORT,	\
							 I2Cx##_CLK, I2Cx##_GPIOCLK,						\
							 I2Cx##_SCL_BIT, I2Cx##_SDA_BIT,					\
							 I2Cx##_SCL_PINSOURCE, I2Cx##_SDA_PINSOURCE,		\
							 semaphore)

#endif // __AP_HAL_YUNEEC_I2CDRIVER_H__
