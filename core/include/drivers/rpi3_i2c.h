/*
 * I2C driver for the BSC of the BCM2835.
 * Author: Raphael Andree
 */

#ifndef __DRIVERS_BCM_I2C_H
#define __DRIVERS_BCM_I2C_H

#include <io.h>
#include <stdint.h>
#include <tee_api_types.h>

/*
 * base address of the BSC0 controller on BCM2835
 */
#define BSC0_BASE U(0xf220500)

/* 
 * I2C Enable
 * 0 - BSC controller disabled.
 * 1 - BSC controller enabled.
 */
#define I2C_C_I2CEN U(0x00008000)

/*
 * Interrupt on RX
 * 0 - Don't generate interrupts on RXR
 * 1 - Generate interrupt while rxr= 1
 */
#define I2C_C_INTR U(0x00000400)

/*
 * Interrupt on TX
 * 0 - Don't generate interrupts on TXW condition
 * 1 - Generate interrupt while TXW = 1
 */
#define I2C_C_INTT U(0x00000200)

/*
 * Interrupt on DONE
 * 0 - Don't generate interrupts on DONE
 * 1 - Generate interrupts while DONE = 1
 */
#define I2C_C_INTD U(0x00000080)

/*
 * ST Start Transfer
 * 0 - No action
 * 1 - Start a new transfer
 */
#define I2C_C_ST U(0x00000080)

/*
 * READ read transfer
 * 0 - Write Packet transfer
 * 1 - Read Packet transfer
 */
#define I2C_C_READ U(0x00000001)

/* BSC status register bits, more info in bcm2835 peripherals documentation */
#define I2C_S_CLKT U(0x00000200)
#define I2C_S_ERR U(0x00000100)
#define I2C_S_RXD U(0x00000020)
#define I2C_S_TXD U(0x00000010)
#define I2C_S_RXR U(0x00000008)
#define I2C_S_TXW U(0x00000004)
#define I2C_S_DONE U(0x00000002)

/* defines for flag in operation struct. */
#define I2C_FLAG_WRITE U(0x00000000)
#define I2C_FLAG_READ U(0x00000001)

/*
 * contains the register mapping, if an i2c_regs struct points to the 
 * base address of the i2c MMIO.
 */
struct i2c_regs {
        uint32_t i2c_c;		/* I2C Control register */
        uint32_t i2c_s;		/* I2C Status register */
        uint32_t i2c_dlen;	/* I2C Data length register */
        uint32_t i2c_a; 	/* I2C Slave address register */
        uint32_t i2c_fifo; 	/* I2C Data FIFO register */
        uint32_t i2c_div;	/* I2C clock divider register */
        uint32_t i2c_del;	/* I2C data delay register */
        uint32_t i2c_clkt;	/* I2C clock stretch timeout register */
};


/*
 * structure represents an I2C controller.
 */
struct bcm2835_i2c_data {
	/* number of the I2C Controller to initialize */
	uint8_t i2c_controller;
	
	/*
	 * base will be filled by i2c_init() which will be used in
	 * subsequent calls for reading/writing data.
	 */
	vaddr_t base;
	
	/* Raspberry pi core clock is nominally 150 MHz */
	uint64_t i2c_core_clk;

	/* Desired I2C speed to be able to calculate clock divider*/
	uint64_t speed;
};


struct i2c_operation {
	/* Flags to quailfy the I2c operation, e.g. read/write (see macros). */
	uint32_t flags;

	/*
	 * Number of bytes to send or receive from the I2C device. A ping
	 * is indicated by setting the length_in_bytes to zero
	 */
	unsigned int length_in_bytes;

	/*
	 * pointer to an array that can hold 8 bit data elements,
	 * containing the data to send or to receive from the I2C device.
	 * The buffer must be at least length_in_bytes size. The array
	 * element size is 8 bit because 
	 * 
	 * Buffer gets allocated upon creation of the i2c_operation object.
	 */
	uint8_t *buffer;
};

/*
 * Structure to fill for I2C read/write operation
 * Has to be defined in the client application.
 */
struct i2c_reg_request {
	 /* Number of operations to perform */
	unsigned int operation_count;
	/* Operation/-s to perform */
	struct i2c_operation *operation;
};
	
/*
 * Initialize I2C controller, based on data passed
 * in i2c_data.
 */
TEE_Result i2c_init(struct bcm2835_i2c_data *i2c_data);

/*
 * Software reset of the entire I2C module.
 * The module is reset and disabled.
 * Status register fields I2C_S are cleared.
 * base		Base address of the I2C controllers registers
 */
void i2c_reset(vaddr_t base);

/*
 * Transfer data to/from I2c slave device
 * base			Base Address of I2C controllers registers
 * slave_address	Slave Address from which data is to be read
 * i2c_operation	Pointer to an i2c_operation structure/array of them
 * operation_count	Number of operations
 */
TEE_Result i2c_bus_xfer(vaddr_t base, uint32_t slave_address,
		struct i2c_operation *i2c_operation,
		unsigned int operation_count);

#endif /* __DRIVERS_BCM_I2C_H */
