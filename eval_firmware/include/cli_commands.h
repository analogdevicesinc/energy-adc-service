/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 *  @file        cli_commands.h
 *  @defgroup    CLI_COMMANDS Cli command routines
 *  @brief       Defines for cli_commands
 * @{
 */

#ifndef __CLI_COMMANDS_H__
#define __CLI_COMMANDS_H__

/*============= I N C L U D E S =============*/
#include "cli_interface.h"
#include "cli_utils.h"
#include <complex.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============= F U N C T I O N  P R O T O T Y P E S =============*/

/**
 * @brief Command to display help.
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdHelp(Args *pArgs);

/**
 * @brief Command to exit the program.
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdExit(Args *pArgs);

/**
 * @brief Command to print the XML file.
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdPrint(Args *pArgs);

/**
 * @brief Command to print the Version
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdVersion(Args *pArgs);

/**
 * @brief Command to set the timeout
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdTimeout(Args *pArgs);

/**
 * @brief Command to Open the device
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdOpen(Args *pArgs);

/**
 * @brief Command to Close the device
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdClose(Args *pArgs);

/**
 * @brief Command to Read from a register
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdRead(Args *pArgs);

/**
 * @brief Command to Write to a register
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdWrite(Args *pArgs);

/**
 * @brief Command to Read from a buffer
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdReadBuf(Args *pArgs);

/**
 * @brief Command to Write to a buffer
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdWriteBuf(Args *pArgs);

/**
 * @brief Command to get the name of the trigger
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdGetRig(Args *pArgs);

/**
 * @brief Command to set the trigger
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdSetRig(Args *pArgs);

/**
 * @brief Command to set the number of kernel buffers
 * @param pArgs - pointer to command arguments storage.
 * @return 0 in case of success or negative value otherwise.
 */
int32_t CmdSet(Args *pArgs);

#ifdef __cplusplus
}
#endif

#endif /* __CLI_COMMANDS_H__ */
/**
 * @}
 */
