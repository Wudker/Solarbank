/*
 * INA219.c
 * Original library: Piotr Smolen, Brandon Thibeaux.
 * Solarbank: calibration for R31 = 0.05 ohm.
 */

#include "main.h"
#include "INA219.h"

enum BatteryState batteryState;
bool isFirst;
uint16_t ina219_calibrationValue;
int16_t ina219_currentDivider_mA;
int16_t ina219_powerMultiplier_mW;

uint16_t Read16(INA219_t *ina219, uint8_t Register)
{
    uint8_t Value[2] = {0U, 0U};

    if (ina219 == 0 || ina219->ina219_i2c == 0)
    {
        Error_Handler();
        return 0;
    }

    if (HAL_I2C_Mem_Read(ina219->ina219_i2c,
                         ((uint16_t)ina219->Address << 1),
                         Register, 1, Value, 2, 1000) != HAL_OK)
    {
        Error_Handler();
        return 0;
    }

    return ((uint16_t)Value[0] << 8) | Value[1];
}

HAL_StatusTypeDef Write16(INA219_t *ina219, uint8_t Register, uint16_t Value)
{
    uint8_t addr[2];
    HAL_StatusTypeDef status;

    if (ina219 == 0 || ina219->ina219_i2c == 0)
    {
        Error_Handler();
        return HAL_ERROR;
    }

    addr[0] = (Value >> 8) & 0xff;
    addr[1] = Value & 0xff;

    status = HAL_I2C_Mem_Write(ina219->ina219_i2c,
                               ((uint16_t)ina219->Address << 1),
                               Register, 1, addr, 2, 1000);
    if (status != HAL_OK)
    {
        Error_Handler();
    }
    return status;
}

uint16_t INA219_ReadBusVoltage(INA219_t *ina219)
{
    uint16_t result = Read16(ina219, INA219_REG_BUSVOLTAGE);
    return ((result >> 3) * 4);
}

int16_t INA219_ReadCurrent_raw(INA219_t *ina219)
{
    int16_t result = (int16_t)Read16(ina219, INA219_REG_CURRENT);
    return result;
}

int16_t INA219_ReadCurrent(INA219_t *ina219)
{
    int16_t result = INA219_ReadCurrent_raw(ina219);
    return (result / ina219_currentDivider_mA);
}

uint16_t INA219_ReadShuntVolage(INA219_t *ina219)
{
    uint16_t result = Read16(ina219, INA219_REG_SHUNTVOLTAGE);
    return (uint16_t)(result * 0.01f);
}

uint16_t INA219_ReadPower(INA219_t *ina219)
{
    uint16_t result = Read16(ina219, INA219_REG_POWER);
    result = result * ina219_powerMultiplier_mW;
    return result;
}

float INA219_GetBatteryLife(INA219_t *ina219, float batteryMax, float batteryMin)
{
    float percentageLife = 0.0f;
    uint16_t vbus = INA219_ReadBusVoltage(ina219);
    percentageLife = (vbus - batteryMin) / (batteryMax - batteryMin);
    if (percentageLife >= 0.0f)
    {
        return percentageLife * 100.0f;
    }
    return 0.0f;
}

int lastTime, deltaTime, now;
int INA219_GetDeltaTime_ms(void)
{
    int now = HAL_GetTick();
    deltaTime = now - lastTime;
    lastTime = now;
    return deltaTime;
}

float totalEnergy;
float INA219_GetTotalPowerUsed(INA219_t *ina219)
{
    float sample = INA219_GetMiliWattsDeltaTime(ina219);
    totalEnergy += (sample * 1000);
    return totalEnergy;
}

float INA219_GetMiliWattsDeltaTime(INA219_t *ina219)
{
    float energy = INA219_GetDeltaTime_ms() * INA219_ReadPower(ina219);
    return energy / 1000.0f;
}

float sample[10];
int ina219_sample_index = 0;
float INA219_GetAVGMiliWatt(INA219_t *ina219)
{
    float energy = 0;
    if (!isFirst)
    {
        isFirst = true;
        sample[0] = INA219_ReadPower(ina219);
        ina219_sample_index = 1;
    }
    else
    {
        if (ina219_sample_index != 10)
        {
            sample[ina219_sample_index] = INA219_ReadPower(ina219);
            ina219_sample_index++;
        }
        else
        {
            ina219_sample_index = 0;
            sample[ina219_sample_index] = INA219_ReadPower(ina219);
        }
    }
    for (int i = 0; i < (int)(sizeof(sample) / sizeof(sample[0])); i++)
    {
        energy += sample[i];
    }
    return energy / (sizeof(sample) / sizeof(sample[0]));
}

enum BatteryState INA219_HealthCheck(INA219_t *ina219,
                                     float batteryPercentageThreshold,
                                     float batteryPercentage)
{
    (void)ina219;
    switch (batteryState)
    {
        case Battery_START:
            batteryState = Battery_OK;
            break;
        case Battery_OK:
        case Battery_LOW:
            batteryState = (batteryPercentage > batteryPercentageThreshold)
                               ? Battery_OK : Battery_LOW;
            break;
        default:
            batteryState = Battery_START;
            break;
    }
    return batteryState;
}

void INA219_Reset(INA219_t *ina219)
{
    Write16(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
    HAL_Delay(1);
}

void INA219_setCalibration(INA219_t *ina219, uint16_t CalibrationData)
{
    Write16(ina219, INA219_REG_CALIBRATION, CalibrationData);
}

uint16_t INA219_getConfig(INA219_t *ina219)
{
    return Read16(ina219, INA219_REG_CONFIG);
}

void INA219_setConfig(INA219_t *ina219, uint16_t Config)
{
    Write16(ina219, INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_2A(INA219_t *ina219)
{
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ina219_calibrationValue = 4096;
    ina219_currentDivider_mA = 10;
    ina219_powerMultiplier_mW = 2;

    INA219_setCalibration(ina219, ina219_calibrationValue);
    INA219_setConfig(ina219, config);
}

void INA219_setCalibration_32V_1A(INA219_t *ina219)
{
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ina219_calibrationValue = 10240;
    ina219_currentDivider_mA = 25;
    ina219_powerMultiplier_mW = 0.8f;

    INA219_setCalibration(ina219, ina219_calibrationValue);
    INA219_setConfig(ina219, config);
}

void INA219_setCalibration_16V_400mA(INA219_t *ina219)
{
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    // R31 = 0.05 ohm. Current LSB = 50 uA, Power LSB = 1 mW.
    ina219_calibrationValue = 16384;
    ina219_currentDivider_mA = 20;
    ina219_powerMultiplier_mW = 1;

    INA219_setCalibration(ina219, ina219_calibrationValue);
    INA219_setConfig(ina219, config);
}

void INA219_setPowerMode(INA219_t *ina219, uint8_t Mode)
{
    uint16_t config = INA219_getConfig(ina219);

    switch (Mode)
    {
        case INA219_CONFIG_MODE_POWERDOWN:
        case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
        case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
        case INA219_CONFIG_MODE_ADCOFF:
            config = (config & ~INA219_CONFIG_MODE_MASK) |
                     (Mode & INA219_CONFIG_MODE_MASK);
            INA219_setConfig(ina219, config);
            break;
        default:
            break;
    }
}

uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address)
{
    if (ina219 == 0 || i2c == 0 || Address > 0x7f)
    {
        return 0;
    }

    isFirst = false;
    ina219->ina219_i2c = i2c;
    ina219->Address = Address;
    ina219_currentDivider_mA = 0;
    ina219_powerMultiplier_mW = 0;

    HAL_StatusTypeDef ready = HAL_I2C_IsDeviceReady(i2c, ((uint16_t)Address << 1), 3, 2);
    if (ready == HAL_OK)
    {
        INA219_Reset(ina219);
        INA219_setCalibration_16V_400mA(ina219);
        return 1;
    }
    return 0;
}
