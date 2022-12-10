// I2C Driver 
// I2C Driver  (I2C_driver.c)
// Mourya Chandra 

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: DE1-SoC Board


// Load kernel module with insmod I2C_driver.ko [param=___]

//-----------------------------------------------------------------------------

#include <linux/kernel.h>     // kstrtouint
#include <linux/module.h>     // MODULE_ macros
#include <linux/init.h>       // __init
#include <linux/kobject.h>    // kobject, kobject_atribute,
                              // kobject_create_and_add, kobject_put
#include <asm/io.h>           // iowrite, ioread, ioremap_nocache (platform specific)
#include "address_map.h"      // overall memory map
#include "I2C_Master_regs.h"  // register offsets in I2C IP

//-----------------------------------------------------------------------------
// Kernel module information
//-----------------------------------------------------------------------------

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mourya");
MODULE_DESCRIPTION("I2C IP Driver");

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

static unsigned int *base = NULL;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void Control_Reg_Write(uint8_t Control_Reg_Data,uint8_t bit_position,uint32_t mask)
{    
    unsigned int value = ioread32(base + OFS_CONTROL);
    value &= ~ mask	;
    iowrite32(value | (Control_Reg_Data << bit_position), base + OFS_CONTROL);
}

int32_t Control_Reg_Read(uint8_t bit_position,uint32_t mask)
{    
    unsigned int value = ioread32(base + OFS_CONTROL);
    value = value & mask ;
    return value >> bit_position;	;
}

void Send_Data(uint32_t DATA)
{
    iowrite32(DATA, base + OFS_DATA);
}

int32_t Get_Data(uint8_t temp)
{
    unsigned int value = ioread32(base + OFS_STATUS) ;
    
    if((value  & 0x00000004) == 0x00000004)
    	return -1 ;
    else
    	return (ioread32(base + OFS_DATA) & 0x000000FF);
}

void Send_Addr(uint32_t Addr)
{
	iowrite32(Addr, base + OFS_ADDRESS);
}

int32_t Read_Addr(uint8_t temp)
{
	return ioread32(base + OFS_ADDRESS);
}
//-----------------------------------------------------------------------------
// Kernel Objects
//-----------------------------------------------------------------------------

// mode
static bool mode = 0;
module_param(mode, bool, S_IRUGO);
MODULE_PARM_DESC(mode, " read or write");

static ssize_t modeStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    if (strncmp(buffer, "read", count-1) == 0)
    {
        Control_Reg_Write(1,0,0x00000001)		;
        printk(KERN_INFO "In read\n");
    }
    else
        if (strncmp(buffer, "write", count-1) == 0)
        {
             Control_Reg_Write(0,0,0x00000001)	;
        printk(KERN_INFO "In write\n");
        }
    return count;
}

static ssize_t modeShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
   if(Control_Reg_Read(0,0x00000001) == 0x00000001)
        strcpy(buffer, "read\n");
    else
        strcpy(buffer, "write\n");
    return strlen(buffer);
}

static struct kobj_attribute modeAttr = __ATTR(mode, 0664, modeShow, modeStore);


// byte count 
static int byte_count = 0;
module_param(byte_count, int, S_IRUGO);
MODULE_PARM_DESC(byte_count, " number of bytes to be read or write ");

static ssize_t byte_countStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    int result = kstrtouint(buffer, 0, &byte_count);
    if (result == 0)
    	Control_Reg_Write(byte_count,1,0x0000007E);
    return count;
}

static ssize_t byte_countShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    byte_count = Control_Reg_Read(1,0x0000007E);
    return sprintf(buffer, "%d\n", byte_count);
}

static struct kobj_attribute byte_countAttr = __ATTR(byte_count, 0664, byte_countShow, byte_countStore);


// register 
static int register_addr = 0;
module_param(register_addr, int, S_IRUGO);
MODULE_PARM_DESC(register_addr, " slave register address");

static ssize_t register_addrStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    int result = kstrtouint(buffer, 0, &register_addr);
    if (result == 0)
    	Control_Reg_Write(register_addr,8,0x0000FF00);
    return count;
}

static ssize_t register_addrShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    register_addr = Control_Reg_Read(8,0x0000FF00);
    return sprintf(buffer, "%d\n", register_addr);
}

static struct kobj_attribute register_addrAttr = __ATTR(register_addr, 0664, register_addrShow, register_addrStore);


// Address
static int address = 0;
module_param(address, int, S_IRUGO);
MODULE_PARM_DESC(address, " slave  address");

static ssize_t addressStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    int result = kstrtouint(buffer, 0, &address);
    if (result == 0)
    	Send_Addr(address)	;
    return count;
}

static ssize_t addressShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    address = Read_Addr(0);
    return sprintf(buffer, "%d\n", address);
}

static struct kobj_attribute addressAttr = __ATTR(address, 0664, addressShow, addressStore);

// repeated start
static bool use_repeated_start = 0;
module_param(use_repeated_start, bool, S_IRUGO);
MODULE_PARM_DESC(use_repeated_start, " use_repeated_start transaction ");

static ssize_t use_repeated_startStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    if (strncmp(buffer, "true", count-1) == 0)
    {
        Control_Reg_Write(1,16,0x00010000)		;
    }
    else
         Control_Reg_Write(0,16,0x00010000)	;
         
    return count;
}

static ssize_t use_repeated_startShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    use_repeated_start = Control_Reg_Read(16,0x00010000);
    if (use_repeated_start == 1)
        strcpy(buffer, "true\n");
    else
        strcpy(buffer, "false\n");
        
    return strlen(buffer);
}

static struct kobj_attribute use_repeated_startAttr = __ATTR(use_repeated_start, 0664, use_repeated_startShow, use_repeated_startStore);


// start
static bool start = 0;
module_param(start, bool, S_IRUGO);
MODULE_PARM_DESC(start, " start transaction ");

static ssize_t startStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    if (strncmp(buffer, "true", count-1) == 0)
    {
        Control_Reg_Write(0,17,0x00020000);
        Control_Reg_Write(1,17,0x00020000);
    }
    else
         Control_Reg_Write(0,17,0x00020000);
         
    return count;
}

static ssize_t startShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    start = Control_Reg_Read(17,0x00020000);
    if (start == 1)
        strcpy(buffer, "true\n");
    else
        strcpy(buffer, "false\n");
        
    return strlen(buffer);
}

static struct kobj_attribute startAttr = __ATTR(start, 0664, startShow, startStore);


// use register 
static bool use_reg = 0;
module_param(use_reg, bool, S_IRUGO);
MODULE_PARM_DESC(use_reg, " use_reg transaction ");

static ssize_t use_regStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    if (strncmp(buffer, "true", count-1) == 0)
    {
        Control_Reg_Write(1,7,0x00000080);
    }
    else
         Control_Reg_Write(0,7,0x00000080);
         
    return count;
}

static ssize_t use_regShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    use_reg = Control_Reg_Read(7,0x0000080);
    if (use_reg == 1)
        strcpy(buffer, "true\n");
    else
        strcpy(buffer, "false\n");
        
    return strlen(buffer);
}

static struct kobj_attribute use_regAttr = __ATTR(use_reg, 0664, use_regShow, use_regStore);


// tx_data
static int tx_data = 0;
module_param(tx_data, int, S_IRUGO);
MODULE_PARM_DESC(tx_data, " slave register address");

static ssize_t tx_dataStore(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
    int result = kstrtouint(buffer, 0, &tx_data);
    if (result == 0)
    	Send_Data(tx_data);
    return count;
}

static struct kobj_attribute tx_dataAttr = __ATTR(tx_data, 0664, NULL, tx_dataStore);


// rx_data

static int rx_data = 0;
module_param(rx_data, int, S_IRUGO);
MODULE_PARM_DESC(rx_data, " slave register address");

static ssize_t rx_dataShow(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
    rx_data = Get_Data(0);
    return sprintf(buffer, "%d\n", rx_data);
}

static struct kobj_attribute rx_dataAttr = __ATTR(rx_data, 0664, rx_dataShow, NULL);


// Attributes

static struct attribute *attrs0[] = {&modeAttr.attr, &startAttr.attr, &byte_countAttr.attr, &register_addrAttr.attr, &use_repeated_startAttr.attr, &addressAttr.attr, &tx_dataAttr.attr, &rx_dataAttr.attr, &use_regAttr.attr,NULL};

static struct attribute_group group0 =
{
    .name = "i2c0",
    .attrs = attrs0
};

static struct kobject *kobj;

//-----------------------------------------------------------------------------
// Initialization and Exit
//-----------------------------------------------------------------------------

static int __init initialize_module(void)
{
    int result;

    printk(KERN_INFO "I2C driver: starting\n");

    // Create i2c directory under /sys/kernel
    kobj = kobject_create_and_add("i2c", kernel_kobj);
    if (!kobj)
    {
        printk(KERN_ALERT "I2C driver: failed to create and add kobj\n");
        return -ENOENT;
    }

    // Create i2c0 group
    result = sysfs_create_group(kobj, &group0);
    if (result !=0)
        return result;

    // Physical to virtual memory map to access gpio registers
    base = (unsigned int*)ioremap_nocache(LW_BRIDGE_BASE + I2C_Master_BASE_OFFSET,
                                          I2C_Master_SPAN_IN_BYTES);
    if (base == NULL)
        return -ENODEV;

    printk(KERN_INFO "I2C driver: initialized\n");

    return 0;
}

static void __exit exit_module(void)
{
    kobject_put(kobj);
    printk(KERN_INFO "I2C driver: exit\n");
}

module_init(initialize_module);
module_exit(exit_module);

