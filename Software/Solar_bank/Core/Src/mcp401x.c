/*
 *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2023 Jerryl James
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************
 */

#include "main.h"
#include "mcp401x.h"

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief Set step value. Range 0 - 127.
  */
void SetStep(uint8_t *step)
{
    HAL_I2C_Master_Transmit(&hi2c1, MCP401XADDR, step, 1, 100U);
}

/**
  * @brief Read step value. Range 0 - 127.
  */
int ReadStep(void)
{
    uint8_t tempBuf[1] = {0};
    HAL_I2C_Master_Receive(&hi2c1, MCP401XADDR, tempBuf, 1, 100U);
    return *tempBuf;
}

void SetResistance(uint32_t Rwb)
{
    uint8_t step;

    if (Rwb <= Rw)
    {
        step = 0;
    }
    else if (Rwb >= (MAXRESISTANCE + Rw))
    {
        step = MAXSTEP;
    }
    else
    {
        step = (uint8_t)(((Rwb - Rw) * MAXSTEP + (MAXRESISTANCE / 2)) / MAXRESISTANCE);
    }

    SetStep(&step);
}

int ReadResistance(void)
{
    uint8_t step = ReadStep();
    return Rw + (int)((MAXRESISTANCE * step + (MAXSTEP / 2)) / MAXSTEP);
}

HAL_StatusTypeDef Set_Resistance_I2C(uint32_t Rwb)
{
    uint8_t step;

    if (Rwb <= Rw)
    {
        step = 0;
    }
    else if (Rwb >= (MAXRESISTANCE + Rw))
    {
        step = MAXSTEP;
    }
    else
    {
        step = (uint8_t)(((Rwb - Rw) * MAXSTEP + (MAXRESISTANCE / 2)) / MAXRESISTANCE);
    }

    return HAL_I2C_Master_Transmit(&hi2c1, MCP401XADDR, &step, 1, 100U);
}

HAL_StatusTypeDef Read_step_I2C(uint8_t *step)
{
    return HAL_I2C_Master_Receive(&hi2c1, MCP401XADDR, step, 1, 100U);
}

// In main usage
void POTENTIOMETER_SET_RESISTANCE(uint32_t Rwb)
{
    Set_Resistance_I2C(Rwb);
}

void POTENTIOMETER_GET_STEP(uint8_t *step)
{
    Read_step_I2C(step);
}
