/* small test application to test UART access via PTA
 *
 * Author: Raphael Andree
 * 
 * PTA MUST BE REGISTERED
 */

#include <kernel/pseudo_ta.h>
#include <trace.h> //for DMSG, EMSG, etc.

#define TA_NAME		"uartAccess.ta"

#define PTA_CMD_PRINTTEXT 	0

#define UACCESS_UUID \
	{0x45cdb822, 0xa2ea, 0x483f, \
		{0x9e, 0x63, 0xd5, 0x9e, 0x11, 0x0e, 0xa5, 0x3f} }
/*
 * already included in pseudo_ta.h:
 * #include <tee_api_types.h>
 * #include <compiler.h> //contains compiler directives like __unused
*/
static TEE_Result invoke_command(void *psess __unused, uint32_t cmd,
	       	__unused uint32_t ptypes, 
		__unused TEE_Param params[TEE_NUM_PARAMS])
{
	switch(cmd){
		case PTA_CMD_PRINTTEXT:
			EMSG("PTA was called");
			return TEE_SUCCESS;
		default:	
			return TEE_ERROR_NOT_SUPPORTED;
	}

}

/* function-like macro to register PTA is located in pseudo_ta.h */
pseudo_ta_register(.uuid = UACCESS_UUID, .name = TA_NAME,
		.flags = PTA_DEFAULT_FLAGS,
		.invoke_command_entry_point = invoke_command);
