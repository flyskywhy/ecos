/* $Id: xspi_g.c,v 1.1.2.2 2011/05/07 03:21:14 sadanan Exp $ */
/******************************************************************************
*
* (c) Copyright 2001-2011 Xilinx, Inc. All rights reserved.
*
* This file contains confidential and proprietary information of Xilinx, Inc.
* and is protected under U.S. and international copyright and other
* intellectual property laws.
*
* DISCLAIMER
* This disclaimer is not a license and does not grant any rights to the
* materials distributed herewith. Except as otherwise provided in a valid
* license issued to you by Xilinx, and to the maximum extent permitted by
* applicable law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL
* FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS,
* IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF
* MERCHANTABILITY, NON-INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE;
* and (2) Xilinx shall not be liable (whether in contract or tort, including
* negligence, or under any other theory of liability) for any loss or damage
* of any kind or nature related to, arising under or in connection with these
* materials, including for any direct, or any indirect, special, incidental,
* or consequential loss or damage (including loss of data, profits, goodwill,
* or any type of loss or damage suffered as a result of any action brought by
* a third party) even if such damage or loss was reasonably foreseeable or
* Xilinx had been advised of the possibility of the same.
*
* CRITICAL APPLICATIONS
* Xilinx products are not designed or intended to be fail-safe, or for use in
* any application requiring fail-safe performance, such as life-support or
* safety devices or systems, Class III medical devices, nuclear facilities,
* applications related to the deployment of airbags, or any other applications
* that could lead to death, personal injury, or severe property or
* environmental damage (individually and collectively, "Critical
* Applications"). Customer assumes the sole risk and liability of any use of
* Xilinx products in Critical Applications, subject only to applicable laws
* and regulations governing limitations on product liability.
*
* THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE
* AT ALL TIMES.
*
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xspi_g.c
*
* This file contains a configuration table that specifies the configuration of
* SPI devices in the system.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a rpm  10/11/01 First release
* 1.00b jhl  03/14/02 Repartitioned driver for smaller files.
* 1.00b rpm  04/24/02 Condensed config typedef - got rid of versions and
*                     multiple base addresses.
* 1.11a wgr  03/22/07 Converted to new coding style.
* 1.12a sv   03/17/08 Updated the code to support 16/32 bit transfer width.
* 2.00a sv   07/30/08 Updated the code to support 16/32 bit transfer width.
* 3.02a sdm  05/04/11 Added a new parameter for the mode in which SPI device
*		      operates.
*
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/
#include "xspi.h"
#include "xparameters.h"

/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/


/************************** Variable Prototypes ******************************/

/**
 * This table contains configuration information for each SPI device
 * in the system.
 */
XSpi_Config XSpi_ConfigTable[XPAR_XSPI_NUM_INSTANCES] = {
	{
	 XPAR_SPI_0_DEVICE_ID,		/* Device ID for instance */
	 XPAR_SPI_0_BASEADDR,		/* Device base address */
	 XPAR_SPI_0_FIFO_EXIST,		/* Does device have FIFOs? */
	 XPAR_SPI_0_SLAVE_ONLY,		/* Is the device slave only? */
	 XPAR_SPI_0_NUM_SS_BITS,	/* Number of slave select bits */
	 XPAR_SPI_0_NUM_TRANSFER_BITS	/* Transfer Data width */
	 XPAR_SPI_0_SPI_MODE		/* standard/dual/quad mode */
	}
	,
	{
	 XPAR_SPI_1_DEVICE_ID,		/* Device ID for instance */
	 XPAR_SPI_1_BASEADDR,		/* Device base address */
	 XPAR_SPI_1_FIFO_EXIST,		/* Does device have FIFOs? */
	 XPAR_SPI_1_SLAVE_ONLY,		/* Is the device slave only? */
	 XPAR_SPI_1_NUM_SS_BITS,	/* Number of slave select bits */
	 XPAR_SPI_1_NUM_TRANSFER_BITS	/* Transfer Data width */
	 XPAR_SPI_1_SPI_MODE		/* standard/dual/quad mode */
	}
};
