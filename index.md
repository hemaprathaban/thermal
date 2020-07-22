``Intel thermal device management

The document explains the work carried for Linux thernal drivers for Intel internship.

The generic thermal management uses a concept of cooling states. 
The intent of a cooling state is to define thermal modes for supporting devices. 
The higher the cooling state, the lower the device/platform temperature would be. 
This can be used for both passive and active cooling devices.

    • Passive cooling -  does not use a fan or other means of forced-air cooling.  Relies on natural convection cooling.
    • Active cooling -   uses a fan directly mounted onto the heat sink for forced-air cooling. 
  
Sysfs driver will redirect all the control requests to the appropriate cooling device driver when the user application sets a new cooling state. 

It is up to the cooling device driver to  implement the actual cooling control.

Thermal management in user space would imply that all the policy decisions will be taken from user space and the kernel’s job would be only to facilitate those decisions.

Below explains the brief about thermal code how they are used how to register the cooling device

Intel Thermal drivers code flow when you insmod 
 1. static int __init init(void) is called, this just calls pci_register_driver() to register intel_pch_thermal_driver.

        a.intel_pch_thermal_driver is object of pci_driver structure and it populates
            ▪ all the functions pointer, like probe, remove, shutdown. 
            ▪ name of the pci driver (name)
            ▪ pci_id of the device.	
        b.So pci framework will automatically call our probe function when it sees a pci device with the device id we defined. 
      
    2.intel_pch_thermal_probe() is called by pci framework.

        a. it does pci device initialization, (it is understandable by reading)
        b. initializes the intel_pch_thermal_device object ptdev.
            ▪ get the memory to operate for register read/write by calling pci_iomap
            ▪ assign particular device operation pch_dev_ops_cpt_ppt or pch_dev_ops_lpt depending on the device (ppt or lpt) in ptdev->ops.                   
        c. register this device as thermal device by calling thermal_zone_device_register()
           we pass tzd_ops object as thermal_zone_device_ops, so the functions mapped in this structure is called when related thermal 
          function is called.
    3.when you call cat /sys/class/thermal/thermal_zone0/temp

        a. since its a thermal core sysfs entry, 
              thermal core (thermal_zone_get_temp() in thermal_core.c) calls pch_thermal_get_temp function.
              which inturn calls pch_cpt_ppt_get_temp , which inturn calls pch_thermal_mem_readb to read the register at the offset TSTR_PPT to get the temperature. 
     

    4. device registers and their bitmasks for each bit or combination of bits are defined in the code. for example

/* Register address for enabling Thermal Sensor*/
#define TSE_PPT	0x01 /* was TSC0 in DPTF */and its bitmasks are,

/* Bit 7: Thermal sensor enable - must be enabled */
#define TSE_PPT_EN	(1 << 7)  -- set the 7th bit to enable Thermal sensor

/* Bit 6: Analog hysteresis control - must be disabled */
#define TSE_PPT_AHC   (1 << 6) -- set the 6th bit to disable Analog hysteresis control 

/* Bit 5-4: Digital hysteresis amount - must not be 00; 01 - 1C; 10 - 2C;
 *   11 - 3C
 */
#define TSE_PPT_DHA   (3 << 4)  -- set bits 5 and 4 for  Digital hysteresis amount

/* Bit 3-2: Sequencer enable and rate - must not be 00 */
#define TSE_PPT_SER   (3 << 2) -- set bits 3 and 2 for Sequencer enable and rate

/* Bit 1: Thermal sensor output select - 0: catastrophic, 1: hot;
 * recommended is 1
 */
#define TSE_PPT_OUT   (1 << 1) -- set bit 1 to select the output mode.


So to enable thermal sensor, first read whatever the default value in the register
    •  value = read(0x01) and 
    •  write (value|TSE_PPT_EN, 0x01) -- we are ORing the ‘value’ with TSE_PPT_EN and writing to register 0x01.
A sensor can be anything that can sense the thermal or the temperature level.

A cooling device can be anything which can cool the device hardware for example it can be a  fan or heatsink. But a fan or heatsink can be applicable only for laptops or pc. So small devices like mobile phones and tablets don't have a fan or heatsink. In this case a cooling device can be a software like controlling a cpu frequency to reduce the power 2.

As for I understand the generic layer provide thermal zone for each sensor.  These thermal zone can attach any cooling devices depending on trip point. Trip point is basically a particular level of thermal management. Depending on the level a specific cooling device will be chosen. This process is called binding and  binding api is called whenever a cooling device is attached to thermal zone. When the device reaches a particular trip point the relevant cooling device is called  from the thermal Zone. A thermal  zone can attach any number of cooling devices for various trip point levels.

cpufreq: (drivers/cpufreq)

http://doc.opensuse.org/products/draft/SLES/SLES-tuning_sd_draft/cha.tuning.power.html
Generic  CPU cooling support: (drivers/thermal)

 `config CPU_THERMAL`
    	`bool "generic cpu cooling support"`
    	`depends on CPU_FREQ`
    	`select CPU_FREQ_TABLE`
    	`help`

This implements the generic cpu cooling mechanism through frequency
reduction. An ACPI version of this already exists (drivers/acpi/processor_thermal.c).
This will be useful for platforms using the generic thermal interface and not the ACPI interface.

thermal_sys-$(CONFIG_CPU_THERMAL)   	+= cpu_cooling.o


Power Clamp driver: (drivers/thermal)

linux/Documentation/thermal/intel_powerclamp.txt
   
   `config INTEL_POWERCLAMP`
        `tristate "Intel PowerClamp idle injection driver"`
	`depends on THERMAL`
 	`depends on X86`
  	`depends on CPU_SUP_INTEL`
 	`help`

Enable this to enable Intel PowerClamp idle injection driver. This
enforce idle time which results in more package C-state residency. The
user interface is exposed via generic thermal framework.

obj-$(CONFIG_INTEL_POWERCLAMP)  += intel_powerclamp.o

This is basic understanding regarding the intel cooling device, based on this learning commits are done 
like registering cooling device, use_define_PCI_table 

[commits](https://github.com/hemaprathaban/thermal/commits/hema_thermal)


