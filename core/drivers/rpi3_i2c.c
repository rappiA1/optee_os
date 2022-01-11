// SPDX-License-Identifier: BSD-2-Clause
/*
 * I2C driver for BCM 2835 and similar
 *
 * Author: Raphael Andree
 *
 */

#include <assert.h>
#include <drivers/rpi3_i2c.h>
#include <io.h>
#include <kernel/boot.h>
#include <kernel/dt.h>
#include <kernel/delay.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <string.h>

static vaddr_t fsel0_base;

/*
 * Reset all register values in status and control register and disable i2c controller.
 *
 * base		BSC0 base address
 */
void i2c_reset(vaddr_t base)
{
	struct i2c_regs *regs = (struct i2c_regs *)base;

	/* disable BSC module */
	io_clrbits32((vaddr_t)&regs->i2c_c, I2C_C_I2CEN);

	/* clear all interrupt configuration in control register. */
//	io_clrbits32((vaddr_t)&regs->i2c_c, I2C_C_INTR | I2C_C_INTT | I2C_C_INTD);	

	/* clear all control register configuration. */
	io_write32((vaddr_t)&regs->i2c_c, 0);

	/* reset potentially set bits in status register */
	io_setbits32((vaddr_t)&regs->i2c_s, I2C_S_CLKT | I2C_S_ERR | I2C_S_DONE);                                                

	/* Set clock frequency to 400 kHz by setting clock divider to 0x09C4 */
	io_write32((vaddr_t)&regs->i2c_div, 2500);
}

/*
 * set i2c device base address, reset status and control registers
 *
 * i2c_data		pointer to an i2c_data struct
 */
TEE_Result i2c_init(struct bcm2835_i2c_data *i2c_data)
{
	/* 
	 * Map physical BSC0 base address into the virtual address space.
	 */
	struct io_pa_va base_reg = { .pa = BSC0_BASE };
	vaddr_t ctrl_base = io_pa_or_va(&base_reg, 32); 
	/* set base address in i2c_data structure */
	i2c_data->base = ctrl_base;

	/* 
	 * translate gpio function select 0 register base address to
	 * optee virtual address 
	 */
	struct io_pa_va gpio_fsel0 = {.pa = 0x3f200000 };
	/* map four bytes into OP-TEE memory */
	fsel0_base = io_pa_or_va(&gpio_fsel0, 4);

	/* 
	 * set gpio pins 2 and 3 to alternative function 0,
	 * for definition see bcm2835 arm peripherals documentation
	 */
	io_setbits32(fsel0_base, 0x00000900);
	
	/* reset BSC Registers */
	i2c_reset(ctrl_base);

	return TEE_SUCCESS;
}

/*
 * Generate Stop Signal and disable I2C controller.
 *
 * regs		pointer to I2c controller registers
 */
static TEE_Result i2c_stop(struct i2c_regs *regs)
{
	/* disable I2C controller */
	io_clrbits32((vaddr_t)&regs->i2c_c, I2C_C_I2CEN);

	return TEE_SUCCESS;
}

/*
 * fills up the write fifo for writing
 *
 * regs			pointer to the BSC 0 registers
 * i2c_operation	contains  write buffer and transfer info
 */
static void i2c_fill_txfifo(struct i2c_regs *regs,
		struct i2c_operation *i2c_operation)
{
	uint32_t val;

	while (i2c_operation->length_in_bytes) {
		val = io_read32((vaddr_t)&regs->i2c_s);

		/* check if FIFO can accept data. */
		if(!(val & I2C_S_TXD))
			break;

		/* write 8 bit value from buffer into fifo. */
		io_write32((vaddr_t)&regs->i2c_fifo, *i2c_operation->buffer);

		/* 
		 * increment pointer buffer by one byte, decrease
		 * the number of bytes to be read 
		 */
		i2c_operation->buffer++;
		i2c_operation->length_in_bytes--;
	}
}

/* 
 * reads all data currently in the FIFO into the buffer of the
 * i2c_operation object.
 *
 * i2c_regs		struct that maps the register addresses.
 * i2c_operation	the operation object that contains operation info and buffer.
 */
static void i2c_drain_rxfifo(struct i2c_regs *regs,
		struct i2c_operation *i2c_operation)
{
	uint32_t val;

	/* while not all data is read */
	while (i2c_operation->length_in_bytes){

		val = io_read32((vaddr_t)&regs->i2c_s);

		/* check if FIFO still contains data */
		if (!(val & I2C_S_RXD))
			break;
			
		*i2c_operation->buffer = io_read32((vaddr_t)&regs->i2c_fifo);

		/*
		 * increment buffer pointer by one byte, decrease 
		 * the number of bytes to be read. 
		 */
		i2c_operation->buffer++;
		i2c_operation->length_in_bytes--;
	}
}

/*
 * performs a write operation on the EEPROM.
 *
 * c_reg_flags		32-Bit value the control register will be set to
 * regs			pointer to i2c registers
 * operation		contains buffer and operation info
 * slave_address	i2c address of the EEPROM
 */
static TEE_Result i2c_write(uint32_t c_reg_flags, struct i2c_regs *regs,
		struct i2c_operation *i2c_operation,
		unsigned int slave_address)
{
	uint32_t s_reg;

	/* write slave address into address register. */
	io_write32((vaddr_t)&regs->i2c_a, slave_address);

	/* set data length register */
	io_write32((vaddr_t)&regs->i2c_dlen, i2c_operation->length_in_bytes);

	/* clear READ  in control register*/
	io_clrbits32((vaddr_t)&regs->i2c_c, I2C_C_READ);

	/* activate BSC, start transfer */
	io_setbits32((vaddr_t)&regs->i2c_c, c_reg_flags);

	/* 
	 * Do polling on TXW and DONE in status register to check if you
	 * need to write to the fifo.
	 */
	while (1){
		s_reg = io_read32((vaddr_t)&regs->i2c_s);

		/* if fifo needs writing */
		if (s_reg & I2C_S_TXW){
			i2c_fill_txfifo(regs, i2c_operation);
		}
		else if (s_reg & I2C_S_DONE){
			/* reset done flag */
			io_setbits32((vaddr_t)&regs->i2c_s, I2C_S_DONE);
			return TEE_SUCCESS;
		}
		else if (s_reg & I2C_S_ERR){
			return TEE_ERROR_GENERIC;
		}
	}
}

/*
 * Read from the I2C bus into the buffer of the operation struct
 * until all data is read from the I2C slave.
 *
 * c_reg_flags		Control register flags for activating BSC and read mode
 * regs			Struct that points to the BSC registers.
 * i2c_operation	Operation struct that contains the buffer to be written to
 * slave_address	Address of the I2C slave (EEPROM)
 */
static TEE_Result i2c_read(uint32_t c_reg_flags, struct i2c_regs *regs,
		struct i2c_operation *i2c_operation,
		unsigned int slave_address)
{
	uint32_t s_reg;
	c_reg_flags |= I2C_C_READ;
	
	/*
	 * write slave address into address register, only 7 least significant
	 * bit are relevant in A-Register.
	 */
	io_write32((vaddr_t)&regs->i2c_a, slave_address);
	
	/* set data length register */
	io_write32((vaddr_t)&regs->i2c_dlen, i2c_operation->length_in_bytes);

	/* activate BSC, start transfer and set read mode. */
	io_setbits32((vaddr_t)&regs->i2c_c, c_reg_flags);

	/* 
	 * do polling on status register to check if fifo must be cleared or
	 * transaction is done
	 */
	while (1){
		s_reg = io_read32((vaddr_t)&regs->i2c_s);
		if (s_reg & I2C_S_RXR){
			i2c_drain_rxfifo(regs, i2c_operation);
		}
		else if (s_reg & I2C_S_DONE){
			i2c_drain_rxfifo(regs, i2c_operation);

			/* reset done flag */
			io_setbits32((vaddr_t)&regs->i2c_s, I2C_S_DONE);
			return TEE_SUCCESS;
		}
		/* check for ACK error */
		else if (s_reg & I2C_S_ERR){
			return TEE_ERROR_GENERIC;
		}
	}
}

/*
 * Sets the needed bits and starts a transfer.
 * regs			pointer to I2C controller registers
 * i2c_operation	pointer to I2C operation struct
 * slave_address	address of I2C slave (EEPROM)
 */
static TEE_Result i2c_start_transfer(struct i2c_regs *regs,
		struct i2c_operation *i2c_operation,
		unsigned int slave_address)
{
	uint32_t c = I2C_C_ST | I2C_C_I2CEN;

	if (i2c_operation->flags & I2C_FLAG_READ)
		return i2c_read(c, regs, i2c_operation, slave_address);
	else 
		return i2c_write(c, regs, i2c_operation, slave_address);
}

/*
 * transfer data to/from the I2C slave device.
 *
 * base			BSC0 base address
 * slave_address	i2c address of EEPROM
 * operation_count	number of read/write operations to be performed
 */
TEE_Result i2c_bus_xfer(vaddr_t base, uint32_t slave_address,
			struct i2c_operation *i2c_operation,
			unsigned int operation_count)
{
	if (!base || !i2c_operation){
		EMSG("NULLPOINTER error in i2c_bus_xfer");
		return TEE_ERROR_BAD_PARAMETERS;
	}


	unsigned int n = 0;
	struct i2c_regs *regs = (struct i2c_regs *)base;
	struct i2c_operation *operation = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	/* check that there is no read operation before the last operation */
	for (n = 0, operation = i2c_operation;
			n < (operation_count - 1); n++, operation++){
		if (operation->flags & I2C_FLAG_READ){
			DMSG("Only one read message supported, has to be last.");
			goto out;
		}
	}
	
	/* go through all operation structs in the array and do the transfers */
	for (n = 0, operation = i2c_operation;
			n < operation_count; n++, operation++){

		if (i2c_operation->buffer == NULL){
			EMSG("NULLPOINTER error in i2c_bus_xfer");
			return TEE_ERROR_BAD_PARAMETERS;
		}

		/* read/write data */
		res = i2c_start_transfer(regs, operation, slave_address);
		if (res)
			goto out;
	}

out:
	i2c_stop(regs);

	return res;
}

