// Functions and pin definitions to communicate with the DRV8323RS motor driver on the TI BOOSTXL dev board.

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_uart.h>
#include <driverlib/adc.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/ssi.h>
#include <driverlib/pwm.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/pin_map.h>
#include <drv8323rs.h>
#include <utils/uartstdio.h>

#include <drv8323rs.h>

// Initializes DRV8323RS SPI communication.
void InitDRV8323RS(void)
{
    // Enable the GPIO port used for DRV8323RS chip enable
    SysCtlPeripheralEnable(DRV8323RS_ENABLE_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_ENABLE_PERIPH));

    // Enable the pin used for DRV8323RS chip enable, and set the pin high
    GPIOPinTypeGPIOOutput(DRV8323RS_ENABLE_PORT, DRV8323RS_ENABLE_PIN);
    GPIOPinWrite(DRV8323RS_ENABLE_PORT, DRV8323RS_ENABLE_PIN, DRV8323RS_ENABLE_PIN);

    // Enable the GPIO port used for DRV8323RS nSCS
    // Because of the pinout on the BOOSTXL board, instead of using SSI2Fss a "manual" nSCS is used
    SysCtlPeripheralEnable(DRV8323RS_NSCS_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_NSCS_PERIPH));

    // Enable the pin used for DRV8323RS nSCS, and set the pin high
    GPIOPinTypeGPIOOutput(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN);
    GPIOPinWrite(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN);

    // Enable the SSI2 peripheral
    SysCtlPeripheralEnable(DRV8323RS_SSI_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_SSI_PERIPH));

    // Enable the GPIO port used for SSI2
    SysCtlPeripheralEnable(DRV8323RS_SSI_GPIO_PERIPH);
    while(!SysCtlPeripheralReady(DRV8323RS_SSI_GPIO_PERIPH));

    // Configure the SSI2 pins and set their pin types
    GPIOPinConfigure(DRV8323RS_SSI_SCLK_PIN_CONFIG); // SCLK
    GPIOPinConfigure(DRV8323RS_SSI_RX_PIN_CONFIG);  // MISO/SDO
    GPIOPinConfigure(DRV8323RS_SSI_TX_PIN_CONFIG);  // MOSI/SDI
    GPIOPinTypeSSI(DRV8323RS_SSI_PORT, DRV8323RS_SSI_SCLK_PIN | DRV8323RS_SSI_RX_PIN | DRV8323RS_SSI_TX_PIN);

    // Configure SSI2 for "Freescale SPI" protocol
    SSIConfigSetExpClk(DRV8323RS_SSI_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_1,SSI_MODE_MASTER,DRV8323RS_SPI_CLK_FREQ,DRV8323RS_SPI_WORD_LEN);

    // Enable SSI2 module
    SSIEnable(DRV8323RS_SSI_BASE);
}

// Writes a word to an address on the DRV8323RS.
// @param address: The address to write to
// @param data: The data to write
void SPIWriteDRV8323(uint8_t address, uint16_t data)
{
    // Initializations
    uint32_t ui32RxData = 0;
    uint32_t *pui32RxData = &ui32RxData;

    // Form the transmission word
    uint32_t ui32TxData = (((0 << 15) | (address << 11)) | (data & 0x7FF));

    // Pull nSCS low and delay
    GPIOPinWrite(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN, 0);
    SysCtlDelay(50);

    // Send the data; wait until the transmission is complete
    SSIDataPut(DRV8323RS_SSI_BASE, ui32TxData);
    while(SSIBusy(DRV8323RS_SSI_BASE)){;}

    // Delay after sending data, then pull nSCS high
    SysCtlDelay(50);
    GPIOPinWrite(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN);

    // Empty RX buffer
    SSIDataGet(DRV8323RS_SSI_BASE, pui32RxData);

    // Insert a delay before the next SPI operation
    SysCtlDelay(50);
}

// Reads a word from an address on the DRV8323RS.
// @param address: The address to read from
// @return: The data at the specified address
uint16_t SPIReadDRV8323(uint8_t address)
{
    // Check that address is valid
    // Valid register addresses are [0x06:0x00]
    if(address > 0x06) return(0);

    // Initializations
    uint32_t ui32RxData = 0;
    uint32_t *pui32RxData = &ui32RxData;

    // Form the transmission word
    uint32_t ui32TxData = (((1 << 15) | (address << 11)));

    // Pull nSCS low and delay
    GPIOPinWrite(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN, 0);
    SysCtlDelay(50);

    // Send the data; wait until the transmission is complete
    SSIDataPut(DRV8323RS_SSI_BASE, ui32TxData);
    while(SSIBusy(DRV8323RS_SSI_BASE)){;}

    // Delay after sending data, then pull nSCS high
    SysCtlDelay(50);
    GPIOPinWrite(DRV8323RS_NSCS_PORT, DRV8323RS_NSCS_PIN, DRV8323RS_NSCS_PIN);

    // Empty RX buffer
    SSIDataGet(DRV8323RS_SSI_BASE, pui32RxData);

    // Insert a delay before the next SPI operation
    SysCtlDelay(50);

    return ((uint16_t) (ui32RxData & 0x7FF));
}
