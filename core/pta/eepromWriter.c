/*
 * PTA provides functions for writing to an EEPROM using the
 * rpi3_i2c driver.
 *
 * Author: Raphael Andree
 */

#include <kernel/pseudo_ta.h>
#include <drivers/rpi3_i2c.h>
#include <tee_api_types.h> //used for TEE_Param functionality
#include <kernel/delay.h>
#include <string.h>

#define TA_NAME		"eepromWriter.c"

#define PTA_CMD_READ	0
#define PTA_CMD_WRITE	1
#define PTA_CMD_INIT	2

#define EEPROM_PAGE_DEFAULT_SIZE	U(64)
#define EEPROM_ADDR_SIZE		2

/* EEPROMWriter UUID */
#define EPW_UUID \
	{0x2b6ea7b2, 0xaf6a, 0x4387, \
		{0xaa, 0xa7, 0x4c, 0xef, 0xcc, 0x4a, 0xfc, 0xbd}}
static struct bcm2835_i2c_data i2c_data;

/*
 * Initializes i2c controller, the controller base address
 * gets written into the static i2c_data struct.
 */
static TEE_Result init_controller(void)
{
	return i2c_init(&i2c_data);
}

/*
 * writes a 16 bit integer address to a char array
 */
static void int2charArr(char *addressarr, uint32_t addressint){
    addressarr[0] = (addressint >> 8) & 0xFF;
    addressarr[1] = (addressint) & 0xFF;
}


/* 
 * Performs a random read on the EEPROM.
 *
 * paramTypes		Defines type of the array elements of params
 * params[0]		Pointer to memory which contains the memory address of which 
 *			to be read from the EEPROM
 * params[1]		Pointer to a buffer into which the memory content of the EEPROM
 * 			is written into after the read process.
 * params[2]		32-Bit integer that contains the slave address.
 */
static TEE_Result read_from_eeprom(uint32_t paramTypes, TEE_Param params[TEE_NUM_PARAMS])
{
	if (paramTypes !=
			TEE_PARAM_TYPES(
				TEE_PARAM_TYPE_MEMREF_OUTPUT,
				TEE_PARAM_TYPE_VALUE_INPUT,
				TEE_PARAM_TYPE_NONE,
				TEE_PARAM_TYPE_NONE))
	{
		return TEE_ERROR_BAD_PARAMETERS;
	}
	struct i2c_operation operation[2];
	uint32_t eepromAddress;
	char addrbuf[2];

	/* check pointers */
	if (params[0].memref.buffer == NULL || params[1].memref.buffer == NULL){
		EMSG("NULLPOINTER error in PTA in read_from_EEPROM");
		return TEE_ERROR_BAD_PARAMETERS;
	}

	eepromAddress = params[2].value.b; 
	int2charArr(addrbuf, eepromAddress);

	/* write address we want to read from EEPROM to EEPROM */
	operation[0].flags = I2C_FLAG_WRITE;
	operation[0].length_in_bytes = EEPROM_ADDR_SIZE;
	operation[0].buffer = (void *)addrbuf;

	/* Perform read operation on EEPROM */
	operation[1].flags = I2C_FLAG_READ;
	operation[1].length_in_bytes = params[0].memref.size;
	operation[1].buffer = params[0].memref.buffer;

	return i2c_bus_xfer(i2c_data.base, params[1].value.a, operation, 2);
}

/*
 * Performs a byte write into the EEPROM
 *
 * params[0]		Pointer to buffer that contains data which will be written into the
 * 			EEPROM
 * params[1]		32-Bit integer that contains the slave address (HW-address of EEPROM)
 */
static TEE_Result write_to_eeprom(uint32_t paramTypes, TEE_Param params[TEE_NUM_PARAMS])

{
	//third parameter contains the buffer which contains the EEPROM address
	if (paramTypes !=
			TEE_PARAM_TYPES(
				TEE_PARAM_TYPE_MEMREF_INPUT,
				TEE_PARAM_TYPE_VALUE_INPUT,
				TEE_PARAM_TYPE_NONE,
				TEE_PARAM_TYPE_NONE))
	{
		return TEE_ERROR_BAD_PARAMETERS;
	}

	uint32_t page_nr;
	uint32_t page_offset;
	uint32_t eepromAddress;
	uint32_t i;
	char tempbuf[66];
	char* databuffer;
	TEE_Result res;

	struct i2c_operation operation;

	/* check pointers */
	if (params[0].memref.buffer == NULL){
		EMSG("NULLPOINTER error in PTA in write_to_EEPROM");
		return TEE_ERROR_BAD_PARAMETERS;
	}

	/* data buffer to write to the EEPROM */
	databuffer = (char *)params[0].memref.buffer;

	/* address on EEPROM as integer */
	eepromAddress = params[1].value.b;

	/* 
	 * Determine the number of 64 Byte pages necessary for
	 * writing the buffer to the EEPROM.
	 */	
	page_nr = params[0].memref.size / EEPROM_PAGE_DEFAULT_SIZE;
	page_offset = params[0].memref.size % EEPROM_PAGE_DEFAULT_SIZE;

	for (i = 0; i < page_nr; i++){

		//init_controller();
		/* Write EEPROM address to the beginning of the writeBuffer */
		int2charArr(tempbuf, eepromAddress);
		
		/* Write 64 Byte page to tempbuf after address */
		memcpy(&tempbuf[2], databuffer,
				EEPROM_PAGE_DEFAULT_SIZE);

		operation.flags = I2C_FLAG_WRITE;
		operation.length_in_bytes = sizeof(tempbuf);
		operation.buffer = (void *) tempbuf;

		/* Perform writing onto EEPROM */
		res = i2c_bus_xfer(i2c_data.base, params[1].value.a, &operation, 1);

		if (res)
			return TEE_ERROR_GENERIC;

		/* increment buffer and eeprom address by 64 bytes */
		databuffer += EEPROM_PAGE_DEFAULT_SIZE;
		eepromAddress += EEPROM_PAGE_DEFAULT_SIZE;

		/* small 100ms delay for the i2c Controller to process */
		udelay(100000);
	}

	if (page_offset > 0){

		//init_controller();
		operation.flags = I2C_FLAG_WRITE;

		/* add eeprom address size to the size of the write operation length */
		operation.length_in_bytes = page_offset + EEPROM_ADDR_SIZE;
	
		/* write eeprom address to the beginning of the write buffer */
		int2charArr(tempbuf, eepromAddress);

		/* Write data to the EEPROM */
		memcpy(&tempbuf[2], databuffer, page_offset);

		/* The buffer contains the Data and the address onto which we write on the EEPROM */
		operation.buffer = (void *)tempbuf;

		/*
		 * Perform write operation.
		 */
		res = i2c_bus_xfer(i2c_data.base, params[1].value.a, &operation, 1);
		DMSG("RES Later, %d", res);
		return res;
	}
	return TEE_SUCCESS;
}


/*
 * Redirects the command from the Client Application to the correct function
 *
 * cmd		32-Bit integer which defines the operation to be executed
 * ptypes	Parameter types of the params array values
 * params	Parameters from the secure world	
 */
static TEE_Result invoke_command(void *psess __unused, uint32_t cmd,
		uint32_t ptypes,
		TEE_Param params[TEE_NUM_PARAMS])
{
	switch(cmd){
		case PTA_CMD_READ:
		       	return read_from_eeprom(ptypes, params);
		case PTA_CMD_WRITE:
			return write_to_eeprom(ptypes, params);
		case PTA_CMD_INIT:
			return init_controller();
		default:
			break;
	}
	return TEE_ERROR_BAD_PARAMETERS;
}

/* register TA in OP-TEE */
pseudo_ta_register(.uuid = EPW_UUID, .name = TA_NAME,
		.flags = PTA_DEFAULT_FLAGS,
		.invoke_command_entry_point = invoke_command);
