/* Copyright 2025, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2025, Guido Cicconi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "app.h"
#include "fxls8964.h"

/*==================[macros and definitions]=================================*/

#define I2C_BAUDRATE     100000U

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static uint32_t trash = 0;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

int main(void)
{
    i2c_master_config_t masterConfig;
    int16_t accX;

    BOARD_InitHardware();

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, ACCEL_I2C_CLK_FREQ);

    fxls8964_init_freefall();
    fxls8964_setFreefallMode();

    while (1)
    {
    	// accX = fxls8964_getAcX();
    	// PRINTF("%d", accX);
    	if(fxls8964_getFreeFall())
    	{
    		trash = 1;
    	}
    }
}

/*==================[end of file]============================================*/
