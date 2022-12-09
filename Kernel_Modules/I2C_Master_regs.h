// I2C_Master IP Library Registers
// Mourya

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: DE1-SoC Board
// Target uC:       -
// System Clock:    -

// Hardware configuration:
// I2C_Master IP core connected to light-weight Avalon bus

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef I2C_REGS_H_
#define I2C_REGS_H_

#define OFS_ADDRESS           0
#define OFS_DATA              1
#define OFS_STATUS            2
#define OFS_CONTROL           3

typedef enum
{
    reg_address	,
    reg_data	,
    reg_status	,
    reg_control
}I2C_Master_reg	;

#define I2C_Master_SPAN_IN_BYTES 16

#endif

