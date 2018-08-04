/*****************************************************************************************
									Drona Aviation
 *****************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#include "platform.h"

#include "drivers/bus_i2c.h"


#define STATUS_OK              0x00
#define STATUS_FAIL            0x01



int32_t VL53L0X_i2c_init(void) 
{
	int32_t status = STATUS_OK;
    
	i2cInit(I2CDEV_1);
	
    return status;
}

int32_t VL53L0X_comms_close(void)
{
	int32_t status = STATUS_OK;
	
	
    return status;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t reg, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
	bool temp;
	uint8_t data = 0;
	
	temp = i2cWriteBuffer(address, reg, count, pdata);
	if(!temp){
			status = STATUS_FAIL;
	}
    
	return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    bool temp;
	uint8_t len = (uint8_t)count;
	
	temp = i2cRead(address, index, len, pdata);
	if(!temp){
		status = STATUS_FAIL;
	}	
    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
	bool temp;
	
	temp = i2cWrite(address, index, data);
	 if(!temp)
		status=STATUS_FAIL;
	
    return status;

}

 
int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[2];
	bool temp;
	
    // Split 16-bit word into MS and LS uint8_t
    buffer[1] = (uint8_t)(data & 0xFF);
	buffer[0] = (uint8_t)((data >> 8) & 0xFF);
    
	temp=i2cWriteBuffer(address, index, 2, (uint8_t *)buffer);
	
    if(!temp)
		status=STATUS_FAIL;
	
	return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[4];
	bool temp;

    // Split 32-bit word into MS ... LS bytes
    buffer[3] = (uint8_t) (data &  0xFF);
	buffer[2] = (uint8_t) ((data >> 8) & 0xFF);
	buffer[0] = (uint8_t) ((data >> 24) & 0xFF);
    buffer[1] = (uint8_t) ((data >> 16) & 0xFF);
    
	temp=i2cWriteBuffer(address, index, 4, (uint8_t *)buffer);
	
	if(!temp)
		status=STATUS_FAIL;    

    return status;
} 

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    bool temp;
	
	temp = i2cRead(address, index, 1, (uint8_t *)pdata);
	if(!temp){
		status = STATUS_FAIL;
	}	
	
    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
	uint8_t  buffer[2];
	bool temp;
    
	temp = i2cRead(address, index, 2, (uint8_t *)buffer);
	if(!temp){
		status = STATUS_FAIL;
	}
	
	*pdata = ((uint16_t)buffer[0]<<8) | buffer[1];
	
	return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[4];
	bool temp;
	
	temp = i2cRead(address, index, 4, (uint8_t *)buffer);
    if(!temp){
		status = STATUS_FAIL;
	}
	
	*pdata = ((uint32_t)buffer[0]<<24) | ((uint32_t)buffer[1]<<16) | ((uint32_t)buffer[2]<<8) | buffer[3];

	return status;
}

/*

// 16 bit address functions


int32_t VL53L0X_write_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    
    return status;
}

int32_t VL53L0X_read_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    
    return status;
}



int32_t VL53L0X_write_byte16(uint8_t address, uint16_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    
    return status;

}


int32_t VL53L0X_write_word16(uint8_t address, uint16_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    
    return status;

}


int32_t VL53L0X_write_dword16(uint8_t address, uint16_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    
    return status;

}


int32_t VL53L0X_read_byte16(uint8_t address, uint16_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    
    return status;

}


int32_t VL53L0X_read_word16(uint8_t address, uint16_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
    
    return status;

}

int32_t VL53L0X_read_dword16(uint8_t address, uint16_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
    
    return status;

}




int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    int32_t status = STATUS_OK;
    float wait_ms = (float)wait_us/1000.0f;

    return status;

}


int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    int32_t status = STATUS_OK;


    return status;

}


int32_t VL53L0X_set_gpio(uint8_t level)
{
    int32_t status = STATUS_OK;
    
    return status;

}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    int32_t status = STATUS_OK;

    return status;
}


int32_t VL53L0X_release_gpio(void)
{
    int32_t status = STATUS_OK;

    return status;

}

int32_t VL53L0X_cycle_power(void)
{
    int32_t status = STATUS_OK;

	return status;
}


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
       
       return STATUS_FAIL;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
       
       return STATUS_FAIL;
}
 */