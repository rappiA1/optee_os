/*
 * PTA provides functions for writing to an EEPROM using the
 * rpi3_i2c driver.
 *
 * Author: Raphael Andree
 */

#include <kernel/pseudo_ta.h>
#include <drivers/rpi3_i2c.h>
#include <tee_api_types.h> //used for TEE_Param functionality

#define TA_NAME		"eepromWriter.c"

#define PTA_CMD_READ	0
#define PTA_CMD_WRITE	1
#define PTA_CMD_INIT	2

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
	EMSG("Inside init_controller\n");
	return i2c_init(&i2c_data);
}


/* 
 * Performs a random read on the EEPROM.
 *
 * params[0]		Pointer to memory which contains the memory address of which 
 *			to be read from the EEPROM
 * params[1]		Pointer to a buffer into which the memory content of the EEPROM
 * 			is written into after the read process.
 * params[2]		32-Bit integer that contains the slave address.
 */
static TEE_Result read_from_eeprom(uint32_t paramTypes, TEE_Param params[TEE_NUM_PARAMS])
{
	EMSG("Inside read_from EEPROM\n");
	if (paramTypes !=
			TEE_PARAM_TYPES(
				TEE_PARAM_TYPE_MEMREF_INPUT,
				TEE_PARAM_TYPE_MEMREF_OUTPUT,
				TEE_PARAM_TYPE_VALUE_INPUT,
				TEE_PARAM_TYPE_NONE))
	{
		return TEE_ERROR_BAD_PARAMETERS;
	}
	TEE_Result res;
	struct i2c_operation operation;

	/* write the address we want to read from to the EEPROM -> see 24LC256 documentation */
	operation.flags = I2C_FLAG_WRITE;
	operation.length_in_bytes = params[0].memref.size;
	operation.buffer = params[0].memref.buffer;

	res = i2c_bus_xfer(i2c_data.base, params[2].value.a, &operation, 1);

	if (res != 0)
		return res;

	/* Perform read operation on EEPROM */
	operation.flags = I2C_FLAG_READ;
	operation.length_in_bytes = params[1].memref.size;
	operation.buffer = params[1].memref.buffer;

	return i2c_bus_xfer(i2c_data.base, params[2].value.a, &operation, 1);
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
//	EMSG("In write_to_eeprom\n");
	if (paramTypes !=
			TEE_PARAM_TYPES(
				TEE_PARAM_TYPE_MEMREF_INPUT,
				TEE_PARAM_TYPE_VALUE_INPUT,
				TEE_PARAM_TYPE_NONE,
				TEE_PARAM_TYPE_NONE))
	{
		return TEE_ERROR_BAD_PARAMETERS;
	}

	struct i2c_operation operation;
	operation.flags = I2C_FLAG_WRITE;
	operation.length_in_bytes = params[0].memref.size;

	/* The buffer contains the Data and the address onto which we write on the EEPROM */
	operation.buffer = params[0].memref.buffer;

	/*
	 * Perform write operation.
	 * 	 
	 * for now only we perform only one operation -> hardcoded
	 */
	return i2c_bus_xfer(i2c_data.base, params[1].value.a, &operation, 1);
}


/*
 * Redirects the command from the Client Application to the correct function
 */
static TEE_Result invoke_command(void *psess __unused, uint32_t cmd,
		uint32_t ptypes,
		TEE_Param params[TEE_NUM_PARAMS])
{
	EMSG("Inside invoke_command\n");
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
