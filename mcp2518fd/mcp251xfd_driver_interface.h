#ifndef DRIVER_INTERFACE_H_INC
#define DRIVER_INTERFACE_H_INC

/*! @brief MCP251XFD_X get millisecond
 *
 * This function will be called when the driver needs to get current millisecond
 */
uint32_t GetCurrentms(void);

/*! @brief MCP251XFD_X compute CRC16-CMS
 *
 * This function will be called when a CRC16-CMS computation is needed (ie. in CRC mode or Safe Write). In normal mode, this can point to NULL.
 * @param[in] *data Is the pointed byte stream
 * @param[in] size Is the size of the pointed byte stream
 * @return The CRC computed
 */
uint16_t ComputeCRC16(const uint8_t* data, size_t size);

//*******************************************************************************************************************
/*! @brief MCP251XFD_Ext1 SPI interface configuration
 *
 * This function will be called at driver initialization to configure the interface driver SPI
 * @param[in] *pIntDev Is the MCP251XFD_Desc.InterfaceDevice of the device that call the interface initialization
 * @param[in] chipSelect Is the Chip Select index to use for the SPI initialization
 * @param[in] sckFreq Is the SCK frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251XFD_InterfaceInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq);

/*! @brief MCP251XFD_Ext1 SPI transfer
 *
 * This function will be called at driver read/write data from/to the interface driver SPI
 * @param[in] *pIntDev Is the MCP251XFD_Desc.InterfaceDevice of the device that call this function
 * @param[in] chipSelect Is the Chip Select index to use for the SPI transfer
 * @param[in] *txData Is the buffer to be transmit to through the SPI interface
 * @param[out] *rxData Is the buffer to be received to through the SPI interface (can be NULL if it's just a send of data)
 * @param[in] size Is the size of data to be send and received trough SPI. txData and rxData shall be at least the same size
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251XFD_InterfaceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size);

#endif
