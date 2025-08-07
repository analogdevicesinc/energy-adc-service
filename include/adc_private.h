/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file adc_private.h
 * @brief    Definitions needed for ADC service internal functions.
 * @addtogroup ADI_ADC
 * @{
 */

#ifndef __ADC_PRIVATE_H__
#define __ADC_PRIVATE_H__

/*============= I N C L U D E S =============*/
#include "adi_adc.h"
#include "adi_adc_frame_format.h"
#include "adi_ade91xx_frame_format.h"
#include "adi_adema12x_frame_format.h"
#include "app_cfg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enum for status of ADC receive command.
 */
typedef enum
{
    /** Command not recieved. */
    ADI_ADC_READ_WRITE_CMD_STATUS_IDLE = 0u,
    /** API called. */
    ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED,
    /** SPI transfer to ADC started  */
    ADI_ADC_READ_WRITE_CMD_STATUS_TX_PROGRESS,
    /** SPI transfer to ADC done  */
    ADI_ADC_READ_WRITE_CMD_STATUS_TX_DONE,
    /** Response started  */
    ADI_ADC_READ_WRITE_CMD_STATUS_RX_PROGRESS,
    /** Response received  */
    ADI_ADC_READ_WRITE_CMD_STATUS_RX_DONE
} ADI_ADC_READ_WRITE_CMD_STATUS;

/**
 * Enum to specify the status of ADC synchronization.
 */
typedef enum
{
    /** Idle state. */
    ADI_ADC_SYNC_IDLE,
    /** Prepare snap for adc synchronization. */
    ADI_ADC_SYNC_PREPARE_SNAP,
    /** Toggle cs pin state. */
    ADI_ADC_SYNC_TOGGLE_CSB,
    /** Read snap shot count register. */
    ADI_ADC_SYNC_READ_SNAP_SHOT_CNT,
    /** Write low value of adc sync counter register. */
    ADI_ADC_SYNC_WRITE_CNT_LOW,
    /** Write high value of adc sync counter register. */
    ADI_ADC_SYNC_WRITE_CNT_HIGH,
    /** Wait state. */
    ADI_ADC_SYNC_WAIT,

} ADI_ADC_SYNC_STATE;

/**
 * Enum for ADC command type.
 */
typedef enum
{
    /** Command sent to ADC is NOP (read to STATUS2 register). */
    ADI_ADC_CMD_TYPE_NOP = 0u,
    /** Command sent to ADC is a custom command. */
    ADI_ADC_CMD_TYPE_CUSTOM_CMD,
} ADI_ADC_CMD_TYPE;

/* frame format structures */

/**
 * Structure for ADC sync.
 */
typedef struct
{
    /** state for adc sync process */
    ADI_ADC_SYNC_STATE syncSnapState;
    /** wait */
    uint32_t waitCount;
} ADI_ADC_SYNC_DATA;

/**
 *  Structure for circular buffer for ADC frames.
 *
 */
typedef struct
{
    /** Index of earliest frame. */
    int32_t readIdx;
    /** Index of next frame to write */
    int32_t writeIdx;
    /** Index of earliest frame. */
    int32_t frameReadIdx;
    /** Index of next frame to write */
    int32_t frameWriteIdx;
    /** Type of ADC command that is sent to the ADCs. */
    /** Frame buffer. Few blocks are kept extra. */
    uint8_t *pAdcRxFrames;
    /** Buffer to store error status of ADC frames */
    uint8_t *pError;
    /** Type of ADC command that is sent to the ADCs. */
    ADI_ADC_CMD_TYPE *pCmdType;
#if (APP_CFG_USE_TIMESTAMP == 1)
    /** Timestamp buffer */
    uint32_t *pTimestamp;
#endif
} ADI_ADC_RX_BUFFER;

/** Address and value for configuration register */
typedef struct
{
    /** Address. */
    uint8_t addr;
    /** Value. */
    uint8_t value;

} ADC_REGISTER;

/** Configuration registers */
typedef struct
{
    /** ADC Register. */
    ADC_REGISTER mask2;
    /** ADC Register. */
    ADC_REGISTER syncSnap;
    /** ADC Register. */
    ADC_REGISTER snapshotCountHi;
    /** ADC Register. */
    ADC_REGISTER configCrcMmrDone;
    /** ADC Register. */
    ADC_REGISTER configCrcMmrForce;
    /** ADC Register. */
    ADC_REGISTER datarate;
    /** ADC Register. */
    ADC_REGISTER datapathConfigLock;
    /** ADC Register. */
    ADC_REGISTER configCrcMmrRetainedDone;
    /** ADC Register. */
    ADC_REGISTER configCrcMmrRetainedForce;
    /** ADC Register. */
    ADC_REGISTER status0Crc;
    /** ADC Register. */
    ADC_REGISTER config0Dready;
    /** ADC Register. */
    ADC_REGISTER config0ClkOut;
    /** ADC Register. */
    ADC_REGISTER config0ClkOutStreamDbg;
    /** ADC Register. */
    ADC_REGISTER config0Rst;
    /** ADC Register. */
    ADC_REGISTER status0;
    /** ADC Register. */
    ADC_REGISTER status1;
    /** ADC Register. */
    ADC_REGISTER status2;
    /** ADC Register. */
    ADC_REGISTER tdmConfig;
    /** ADC Register. */
    ADC_REGISTER tdmThrshMsb;
    /** ADC Register. */
    ADC_REGISTER tdmThrshLsb;

} ADC_CONFIG_REGISTERS;

/**
 * @brief Function pointer type for populating user-defined registers.
 *
 * @param pConfigReg Pointer to ADC configuration registers.
 * @param pUserCfgRegs Pointer to user-defined ADC configuration registers.
 * @return ADI_ADC_STATUS indicating success or error.
 */
typedef ADI_ADC_STATUS (*PF_POPULATE_USER_CFG_REG)(ADC_CONFIG_REGISTERS *pConfigReg,
                                                   ADI_ADC_CONFIG_REGISTERS *pUserCfgRegs);

/**
 * @brief Function pointer type for setting the spi response frame format.
 *
 * @param hAdc      - ADC Service handle
 * @param format    - SPI response frame format type.
 * @return ADI_ADC_STATUS indicating success or error.
 */
typedef ADI_ADC_STATUS (*SET_FRAME_FORMAT)(ADI_ADC_HANDLE hAdc, uint8_t format);

/** Type config details. */
typedef struct
{
    /** ADC Configuration register details. */
    ADC_CONFIG_REGISTERS configReg;

    /** Number of samples per frame. */
    uint8_t samplesPerFrame;
    /** Frame size */
    uint8_t frameLength;
    /** Offset of command within the frame*/
    uint8_t cmdOffset;
    /** Offset of status0 within the frame*/
    uint8_t status0Offset;
    /** Offset of status1 within the frame*/
    uint8_t status1Offset;
    /** Offset of data field in frame */
    uint8_t dataOffset;
    /** Function pointer to check status*/
    ADI_ADC_STATUS (*pfCheckStatus)(uint8_t status0, uint8_t status1);
    /** Function pointer to fill user-defined registers */
    PF_POPULATE_USER_CFG_REG pfPopulateUserCfgReg;
    /** Function pointer to set frame format */
    SET_FRAME_FORMAT pfSetFrameFormat;
} ADC_TYPE_CONFIG;

/** Callback type. */
typedef ADI_ADC_STATUS (*ADC_POPULATE_INIT_FUNC)(ADC_CONFIG_REGISTERS *);

/** Type for sample delay buffer. */
typedef struct
{
    /** Write index. */
    uint32_t writeIdx;
    /** Read index. */
    uint32_t readIdx;
    /** Buffer to store delayed samples. */
    int32_t *pBuffer;
} ADI_ADC_DELAY_BUFFER;

/**
 * Structure for holding the adc datapath read write values.
 */
typedef struct
{
    /** Stores value read from adc. */
    uint8_t readVal[2];
    /** Stores value read from adc LO register. */
    uint8_t valLo[2];
    /** Stores value read from adc MD register. */
    uint8_t valMd[2];
    /** Stores value read from adc HI register. */
    uint8_t valHi[2];
} ADI_ADC_DATAPATH_READ_VALS;

/**
 * Structure for ADC info.
 */
typedef struct
{
    /** Stores ADC configuration parameters. */
    ADI_ADC_CONFIG adcCfg;
    /** Stores ADC configuration registers parameters. */
    ADI_ADC_CONFIG_REGISTERS configReg;
    /** Stores status of ADC request. */
    ADI_ADC_READ_WRITE_CMD_STATUS cmdStatus;
    /** Pointer to callback function that indicates
     *  if ADC response if ready or not.
     */
    ADI_ADC_CALLBACK_FUNC pCallbackFunc;
    /** Flag for blockready. */
    volatile uint8_t blockReady;
    /** Maximum frames in buffer. */
    uint32_t maxFramesInBuffer;

    /** Flag indicating if synchronization is in progress or not.
     */
    bool syncInProgress;
    /** ADC index for CRC computation of response frame. */
    volatile uint8_t adcCrcIndex;
    /** SPI Recive buffer */
    ADI_ADC_RX_BUFFER rxBuffer;
    /** ADC Sync data */
    ADI_ADC_SYNC_DATA syncData;

    /** Frame size for all ADCs in daisychain*/
    uint32_t allAdcFrameLength;
    /** Indicates whether previous SPI Rx is complete*/
    volatile bool isAdcSpiRxComplete;
    /** Expected CRC for curent frame*/
    uint32_t expectedCrc;
    /** indicates RX buffer is overflowed*/
    uint8_t rxBufferOverflow;
    /** Some runtime statistics & status*/
    ADI_ADC_RUN_DATA runData;
    /** Stores values read from datapath register for processing */
    ADI_ADC_DATAPATH_READ_VALS datapathReadVal;
    /** Buffers to store delayed samples */
    ADI_ADC_DELAY_BUFFER *pChannelDelayBuffers;
    /** Pointer to state memory */
    uint32_t *pStateMemory;
    /** state memory size */
    uint32_t stateMemorySize;
    /** Maximum number of channels */
    uint32_t maxNumChannel;
    /** Maximum number of channels per Adc*/
    uint32_t maxNumChannelPerAdc;
    /** Delay buffer size*/
    uint32_t delayBuffSize;

    /** ADC type config */
    ADC_TYPE_CONFIG *pTypeConfig;
    /** Datapath shift value */
    uint8_t *pDatapathShift;
    /** Datapath SCF enable value */
    uint8_t *pDatapathScfEn;
    /** Buffer to extract the samples from the ADC frame */
    int32_t *pSampleLinearBuf;
    /** SPI buffer to store previous Rx frames
     * on issuing a read command */
    uint8_t *pLastCmdRxFrames;
    /** SPI Transmit buffer for commands in runtime */
    uint8_t *pTxBufferCmd;
    /** SPI Transmit buffer */
    uint8_t *pTxBuffer;
    /** Buffer to store one proper previous sample of each ADC.
     * These values are used when there is any CRC error on the
     * current sample.
     */
    int32_t *pPrevSamples;
    /** Buffer to store the pointers of Rx buffers */
    uint8_t **pRxFramePtr;
    /** Buffer to store the pointers of Tx Cmd buffers */
    uint8_t **pTxCmdFramePtr;
    /** Buffer to store the pointers of Tx buffers */
    uint8_t **pTxFramePtr;

} ADI_ADC_INFO;

/**
 * Function to read a block of ADC data.
 * @param[in]  pInfo	        - Pointer to the ADC info structure.
 * @param[out] pBuffer	        - Pointer to the output buffer.
 * @param[out] pAdcStatusOutput	- Pointer to the status output buffer.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcReadBlock(ADI_ADC_INFO *pInfo, int32_t *pBuffer,
                            ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput);

/**
 * Function to read a block of ADC data with an integer sample delay.
 * @param[in]  pInfo	        - Pointer to the ADC info structure.
 * @param[out] pBuffer	        - Pointer to the output buffer.
 * @param[out] pAdcStatusOutput	- Pointer to the status output buffer.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcReadBlockWithDelay(ADI_ADC_INFO *pInfo, int32_t *pBuffer,
                                     ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput);

/**
 * Function to get ADC run status.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[out] pRunStatus - Pointer to buffer to store output.
 * @return  One of the return codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetRunStatus(ADI_ADC_INFO *pInfo, bool *pRunStatus);

/**
 * @brief Checks if a block of data is available or not.
 *
 */
int32_t AdcIsBlockReady(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to Assemble read/write command.
 *
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcAssembleReadWrite(ADI_ADC_INFO *pInfo, uint8_t operation, uint16_t address,
                                    uint8_t value, int8_t adcIdx);

/**
 * @brief Function to Assemble write command.
 *
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcAssembleWriteRegister(ADI_ADC_INFO *pInfo, uint16_t address, uint8_t value,
                                        int8_t adcIdx);

/**
 * @brief Function to write command to adc.
 *
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcWriteRegister(ADI_ADC_INFO *pInfo, uint16_t address, uint8_t value,
                                int8_t adcIdx);

/**
 * @brief Function will assemble a NOP command for all ADC.
 *        NOP command is a READ to STATUS2 register.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  txFramePtr	- Double Pointer to the tx frame buffer.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcAssembleNopAllAdc(ADI_ADC_INFO *pInfo, uint8_t **txFramePtr);

/**
 * @brief Function to perform the clock out initialization sequence on the ADCs.
 *
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  adcIdx	- Adc index which can take values from 0 to 'numAdc' in #ADI_ADC_CONFIG.
 *                      It cannot be -1;
 *
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcEnableClockOut(ADI_ADC_INFO *pInfo, uint8_t adcIdx);

/**
 * @brief Function to perform the initialization sequence on the ADCs.
 *
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  pConfigReg	- Pointer to the user requested config registers.
 *
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 */
ADI_ADC_STATUS AdcConfigure(ADI_ADC_INFO *pInfo, ADI_ADC_CONFIG_REGISTERS *pConfigReg);

/**
 * @brief Perform SPI transfer.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS TransferAdcData(ADI_ADC_INFO *pInfo);

/**
 * @brief Set Align bit in SYNC_SNAP register to 1
 *        so that ADC outputs are generated at exact
 *        moment.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AlignAdc(ADI_ADC_INFO *pInfo);

/**
 * @brief Configure TDM mode
 *
 */
ADI_ADC_STATUS UpdateTdmConfig(ADI_ADC_INFO *pInfo);

/**
 * @brief Configure TDM mode
 *
 */
ADI_ADC_STATUS SetTdmThreshold(ADI_ADC_INFO *pInfo);

/**
 * @brief function to read adc registers.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  address	- Address.
 * @param[in]  adcIdx	- ADC index.
 * @param[out]  pBuffer	- Pointer to output buffer.
 * @param[out]  pNumBytes	- Number of bytes.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcReadRegister(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                               uint8_t *pBuffer, uint32_t *pNumBytes);

/**
 * @brief Reset the variables to their reset state.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS SetConfigurations(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to copy the data from Rx frame.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  adcIdx	- Adc index.
 * @param[in]  pRxFrame	- Pointer to Rx frame.
 * @param[out]  pDst	- Pointer to destination buffer.
 * @param[out]  pNumBytes	- Pointer to number of bytes.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS CopyDataFromRxFrame(ADI_ADC_INFO *pInfo, int8_t adcIdx, uint8_t *pRxFrame,
                                   volatile uint8_t *pDst, uint32_t *pNumBytes);

/**
 * @brief Function to assemble the NOP command.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  pTypeConfig	- Pointer to type config.
 * @param[in]  pAdcFrame	- Pointer to adc frame.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AssembleNop(ADI_ADC_INFO *pInfo, ADC_TYPE_CONFIG *pTypeConfig, uint8_t *pAdcFrame);

/**
 * @brief Function to collect adc samples.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  timestamp	- Timestamp of current sample.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcCollectSamples(ADI_ADC_INFO *pInfo, uint32_t timestamp);

/**
 * @brief Function to copy status.
 * @param[in]  pTypeConfig	- Pointer to type config.
 * @param[in]  pRxFrame	- Pointer to Rx frame.
 * @param[in]  pCmdType	- Pointer to command type.
 * @param[out]  pAdcStatusOutput	- Pointer to status output.
 *
 */
void CopyStatusOutput(ADC_TYPE_CONFIG *pTypeConfig, volatile uint8_t *pRxFrame,
                      ADI_ADC_CMD_TYPE *pCmdType, ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput);

/**
 * @brief Function to get pointer to buffer where
 *        data has to be read.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 *
 */
volatile uint8_t *GetAdcRxWriteFrame(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to update the ADCs with new sampling rate.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcSetSamplingRate(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to reset the internal states of the adc service.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcResetStates(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to process the Rx frames received from ADCs.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcProcessRxFrames(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to clear quantisation noise.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  pBuffer	- Pointer to the buffer with samples.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS AdcClearQuantizationNoise(ADI_ADC_INFO *pInfo, int32_t *pBuffer);

/**
 * @brief Function to reset the configuration when frame format is switched.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS documentation for details.
 *
 */
ADI_ADC_STATUS ResetConfigurations(ADI_ADC_INFO *pInfo);

/**
 * @brief Set integer delay for ADC channels.
 *
 * This function sets the integer delay values for the specified ADC channels.
 * The delays are applied on a per-channel basis, and the user provides the
 * desired delays along with the corresponding channel indices.
 *
 * @param[in] pInfo         Pointer to the ADC service info structure.
 * @param[in] pIntegerDelay Pointer to the array containing integer delay values per channel.
 * @param[in] pChanIdx      Pointer to the array of channel indices corresponding to the delays.
 * @param[in] numChan       Number of channels for which the delay is to be set.
 * @param[in] adcNum        ADC index for which the delays are applied.
 *
 * @return ADI_ADC_STATUS_SUCCESS if delays are set successfully, otherwise appropriate error code.
 */
ADI_ADC_STATUS AdcSetIntegerSampleDelay(ADI_ADC_INFO *pInfo, uint8_t *pIntegerDelay,
                                        uint8_t *pChanIdx, int8_t numChan, int8_t adcNum);

/**
 * @brief Validate ADC channel indices for a given ADC.
 *
 * This function checks whether the specified channel indices are valid for the given ADC.
 * It ensures that the channels exist within the configuration and that the channel indices
 * are within the allowed range for the ADC.
 *
 * @param[in] pInfo     Pointer to the ADC service info structure.
 * @param[in] adcIdx    Index of the ADC to validate.
 * @param[in] pChanIdx  Pointer to the array of channel indices to validate.
 * @param[in] numChan   Number of channels to validate.
 *
 * @return ADI_ADC_STATUS_SUCCESS if all channels are valid, otherwise appropriate error code.
 */
ADI_ADC_STATUS CheckChannelValid(ADI_ADC_INFO *pInfo, int8_t adcIdx, uint8_t *pChanIdx,
                                 int8_t numChan);

/**
 * @brief Allocate memory for pointers.
 *
 * @param[in] pInfo     Pointer to the ADC service info structure.
 * @param[in] pStateMemory    State memory.
 * @param[in] stateMemorySize    State memory size.
 * @param[in] pConfig     ADC Configs.
 *
 * @return ADI_ADC_STATUS_SUCCESS if all channels are valid, otherwise appropriate error code.
 */
ADI_ADC_STATUS AllocateMemory(ADI_ADC_INFO *pInfo, uint32_t *pStateMemory, uint32_t stateMemorySize,
                              ADI_ADC_CONFIG *pConfig);

/**
 * @brief Function to set max number of channels.
 *
 * @param[in] numAdc    Number of ADC
 * @param[in] pAdcType  Type of ADCs.
 * @param[in] pInfo     Pointer to the ADC service info structure..
 *
 * @return ADI_ADC_STATUS_SUCCESS if all channels are valid, otherwise appropriate error code.
 */
ADI_ADC_STATUS SetMaxChannels(uint8_t numAdc, ADI_ADC_TYPE *pAdcType, ADI_ADC_INFO *pInfo);

#ifdef __cplusplus
}
#endif

#endif /*__ADC_PRIVATE_H__ */
/** @} */
