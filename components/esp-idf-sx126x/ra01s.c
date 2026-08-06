#include "ra01s.h"

#define TAG "RA01S"

// Arduino compatible macros
#define delayMicroseconds(us) esp_rom_delay_us(us)
#define delay(ms)             esp_rom_delay_us(ms * 1000)

void LoRaErrorDefault(int error) {
    ESP_LOGE(TAG, "LoRaErrorDefault=%d", error);
    while (true) {
        vTaskDelay(1);
    }
}

__attribute__((weak, alias("LoRaErrorDefault"))) void LoRaError(int error);

esp_err_t LoRaInit(sx126x_config_t *sx126x_config, sx126x_handle_t *sx126x_handle) {
    esp_err_t ret = ESP_FAIL;

    /* validate memory allocation */
    sx126x_handle_t out_handle = (sx126x_handle_t)calloc(1, sizeof(*out_handle));
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_NO_MEM, TAG, "No memory for SX126x device");

    /* copy device config to out_handle */
    out_handle->dev_config = *sx126x_config;
    out_handle->txActive   = false;
    out_handle->txLost     = 0;
    out_handle->debugPrint = false;

    /* GPIO config */
    gpio_reset_pin(sx126x_config->ss);
    gpio_set_direction(sx126x_config->ss, GPIO_MODE_OUTPUT);
    gpio_set_level(sx126x_config->ss, HIGH);

    gpio_reset_pin(sx126x_config->reset);
    gpio_set_direction(sx126x_config->reset, GPIO_MODE_OUTPUT);

    gpio_reset_pin(sx126x_config->busy);
    gpio_set_direction(sx126x_config->busy, GPIO_MODE_INPUT);

    if (sx126x_config->txen != -1) {
        gpio_reset_pin(sx126x_config->txen);
        gpio_set_direction(sx126x_config->txen, GPIO_MODE_OUTPUT);
    }

    if (sx126x_config->rxen != -1) {
        gpio_reset_pin(sx126x_config->rxen);
        gpio_set_direction(sx126x_config->rxen, GPIO_MODE_OUTPUT);
    }

    /* SPI device configuration */
    const spi_device_interface_config_t sx_dev_config = {
        .clock_speed_hz = 9000000,
        .mode           = 0,
        .spics_io_num   = sx126x_config->ss,
        .queue_size     = 7,
        .flags          = 0,
        .pre_cb         = NULL,
    };

    /* add SX126x to SPI bus */
    ESP_GOTO_ON_ERROR(spi_bus_add_device(sx126x_config->spi_host, &sx_dev_config, &out_handle->spi_handle), err_handle,
                      TAG, "Failed to add SX126x device to SPI bus");

    /* log SX126x config */
    ESP_LOGI(TAG, "SS_GPIO=%d", sx126x_config->ss);
    ESP_LOGI(TAG, "RESET_GPIO=%d", sx126x_config->reset);
    ESP_LOGI(TAG, "BUSY_GPIO=%d", sx126x_config->busy);
    ESP_LOGI(TAG, "TXEN_GPIO=%d", sx126x_config->txen);
    ESP_LOGI(TAG, "RXEN_GPIO=%d", sx126x_config->rxen);

    *sx126x_handle = out_handle;
    return ESP_OK;

err_handle:
    if (out_handle->spi_handle)
        spi_bus_remove_device(out_handle->spi_handle);
    free(out_handle);
    return ret;
}

void spi_write_byte(sx126x_handle_t handle, uint8_t *Dataout, size_t DataLength) {
    spi_transaction_t SPITransaction;

    if (DataLength > 0) {
        memset(&SPITransaction, 0, sizeof(spi_transaction_t));
        SPITransaction.length    = DataLength * 8;
        SPITransaction.tx_buffer = Dataout;
        SPITransaction.rx_buffer = NULL;
        spi_device_transmit(handle->spi_handle, &SPITransaction);
    }

    return;
}

void spi_read_byte(sx126x_handle_t handle, uint8_t *Datain, uint8_t *Dataout, size_t DataLength) {
    spi_transaction_t SPITransaction;

    if (DataLength > 0) {
        memset(&SPITransaction, 0, sizeof(spi_transaction_t));
        SPITransaction.length    = DataLength * 8;
        SPITransaction.tx_buffer = Dataout;
        SPITransaction.rx_buffer = Datain;
        spi_device_transmit(handle->spi_handle, &SPITransaction);
    }

    return;
}

uint8_t spi_transfer(sx126x_handle_t handle, uint8_t address) {
    uint8_t datain[1];
    uint8_t dataout[1];
    dataout[0] = address;
    // spi_write_byte(dataout, 1 );
    spi_read_byte(handle, datain, dataout, 1);
    return datain[0];
}

int16_t LoRaBegin(sx126x_handle_t handle) {

    uint32_t frequencyInHz   = handle->dev_config.frequency;
    int8_t   txPowerInDbm    = handle->dev_config.tx_power;
    float    tcxoVoltage     = handle->dev_config.tcxo_voltage;
    bool     useRegulatorLDO = handle->dev_config.use_regulator_ldo;

    if (txPowerInDbm > 22)
        txPowerInDbm = 22;
    if (txPowerInDbm < -3)
        txPowerInDbm = -3;

    Reset(handle);

    uint8_t wk[2];
    ReadRegister(handle, SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2); // 0x0740
    uint16_t syncWord = (wk[0] << 8) + wk[1];
    ESP_LOGI(TAG, "syncWord=0x%x", syncWord);
    if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE) {
        ESP_LOGE(TAG, "SX126x error, maybe no SPI connection");
        return ERR_INVALID_MODE;
    }

    ESP_LOGI(TAG, "SX126x installed");
    SetStandby(handle, SX126X_STANDBY_RC);

    SetDio2AsRfSwitchCtrl(handle, true);
    ESP_LOGI(TAG, "tcxoVoltage=%f", tcxoVoltage);
    // set TCXO control, if requested
    if (tcxoVoltage > 0.0) {
        SetDio3AsTcxoCtrl(handle, tcxoVoltage,
                          RADIO_TCXO_SETUP_TIME); // Configure the radio to use a TCXO controlled by DIO3
    }

    Calibrate(handle, SX126X_CALIBRATE_IMAGE_ON | SX126X_CALIBRATE_ADC_BULK_P_ON | SX126X_CALIBRATE_ADC_BULK_N_ON |
                          SX126X_CALIBRATE_ADC_PULSE_ON | SX126X_CALIBRATE_PLL_ON | SX126X_CALIBRATE_RC13M_ON |
                          SX126X_CALIBRATE_RC64K_ON);

    ESP_LOGI(TAG, "useRegulatorLDO=%d", useRegulatorLDO);
    if (useRegulatorLDO) {
        SetRegulatorMode(handle, SX126X_REGULATOR_LDO); // set regulator mode: LDO
    } else {
        SetRegulatorMode(handle, SX126X_REGULATOR_DC_DC); // set regulator mode: DC-DC
    }

    SetBufferBaseAddress(handle, 0, 0);
#if 0
	// SX1261_TRANCEIVER
	SetPaConfig(0x06, 0x00, 0x01, 0x01); // PA Optimal Settings +15 dBm
	// SX1262_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	// SX1268_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
#endif
    SetPaConfig(handle, 0x04, 0x07, 0x00, 0x01);               // PA Optimal Settings +22 dBm
    SetOvercurrentProtection(handle, 60.0);                    // current max 60mA for the whole device
    SetPowerConfig(handle, txPowerInDbm, SX126X_PA_RAMP_200U); // 0 fuer Empfaenger
    SetRfFrequency(handle, frequencyInHz);
    return ERR_NONE;
}

void FixInvertedIQ(sx126x_handle_t handle, uint8_t iqConfig) {
    // fixes IQ configuration for inverted IQ
    // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
    // When exchanging LoRa packets with inverted IQ polarity, some packet losses may be observed for longer packets.
    // Workaround: Bit 2 at address 0x0736 must be set to:
    // “0” when using inverted IQ polarity (see the SetPacketParam(...) command)
    // “1” when using standard IQ polarity

    // read current IQ configuration
    uint8_t iqConfigCurrent = 0;
    ReadRegister(handle, SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736

    // set correct IQ configuration
    // if(iqConfig == SX126X_LORA_IQ_STANDARD) {
    if (iqConfig == SX126X_LORA_IQ_INVERTED) {
        iqConfigCurrent &= 0xFB; // using inverted IQ polarity
    } else {
        iqConfigCurrent |= 0x04; // using standard IQ polarity
    }

    // update with the new value
    WriteRegister(handle, SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736
}

void LoRaConfig(sx126x_handle_t handle) {
    uint8_t  spreadingFactor = handle->dev_config.spreading_factor;
    uint8_t  bandwidth       = handle->dev_config.bandwidth;
    uint8_t  codingRate      = handle->dev_config.coding_rate;
    uint16_t preambleLength  = handle->dev_config.preamble_length;
    uint8_t  payloadLen      = handle->dev_config.payload_len;
    bool     crcOn           = handle->dev_config.crc_on;
    bool     invertIrq       = handle->dev_config.invert_iq;

    SetStopRxTimerOnPreambleDetect(handle, false);
    SetLoRaSymbNumTimeout(handle, 0);
    SetPacketType(handle, SX126X_PACKET_TYPE_LORA); // SX126x.ModulationParams.PacketType : MODEM_LORA
    uint8_t ldro = 0;                               // LowDataRateOptimize OFF
    SetModulationParams(handle, spreadingFactor, bandwidth, codingRate, ldro);

    handle->PacketParams[0] = (preambleLength >> 8) & 0xFF;
    handle->PacketParams[1] = preambleLength;
    if (payloadLen) {
        handle->PacketParams[2] = 0x01; // Fixed length packet (implicit header)
        handle->PacketParams[3] = payloadLen;
    } else {
        handle->PacketParams[2] = 0x00; // Variable length packet (explicit header)
        handle->PacketParams[3] = 0xFF;
    }

    if (crcOn)
        handle->PacketParams[4] = SX126X_LORA_CRC_ON;
    else
        handle->PacketParams[4] = SX126X_LORA_CRC_OFF;

    if (invertIrq)
        handle->PacketParams[5] = 0x01; // Inverted LoRa I and Q signals setup
    else
        handle->PacketParams[5] = 0x00; // Standard LoRa I and Q signals setup

    // fixes IQ configuration for inverted IQ
    FixInvertedIQ(handle, handle->PacketParams[5]);

    WriteCommand(handle, SX126X_CMD_SET_PACKET_PARAMS, handle->PacketParams, 6); // 0x8C

    // Do not use DIO interruptst
    SetDioIrqParams(handle, SX126X_IRQ_ALL, // all interrupts enabled
                    SX126X_IRQ_NONE,        // interrupts on DIO1
                    SX126X_IRQ_NONE,        // interrupts on DIO2
                    SX126X_IRQ_NONE         // interrupts on DIO3
    );

    // Receive state no receive timeoout
    SetRx(handle, 0xFFFFFF);
}

void LoRaDebugPrint(sx126x_handle_t handle, bool enable) { handle->debugPrint = enable; }

uint8_t LoRaReceive(sx126x_handle_t handle, uint8_t *pData, int16_t len) {
    uint8_t  rxLen   = 0;
    uint16_t irqRegs = GetIrqStatus(handle);
    // uint8_t status = GetStatus();

    if (irqRegs & SX126X_IRQ_RX_DONE) {
        // ClearIrqStatus(SX126X_IRQ_RX_DONE);
        ClearIrqStatus(handle, SX126X_IRQ_ALL);
        rxLen = ReadBuffer(handle, pData, len);
    }

    return rxLen;
}

bool LoRaSend(sx126x_handle_t handle, uint8_t *pData, int16_t len, uint8_t mode) {
    uint16_t irqStatus;
    bool     rv = false;

    if (handle->txActive == false) {
        handle->txActive = true;
        if (handle->PacketParams[2] == 0x00) { // Variable length packet (explicit header)
            handle->PacketParams[3] = len;
        }
        WriteCommand(handle, SX126X_CMD_SET_PACKET_PARAMS, handle->PacketParams, 6); // 0x8C

        // ClearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
        ClearIrqStatus(handle, SX126X_IRQ_ALL);

        WriteBuffer(handle, pData, len);
        SetTx(handle, 500);

        if (mode & SX126x_TXMODE_SYNC) {
            irqStatus = GetIrqStatus(handle);
            while ((!(irqStatus & SX126X_IRQ_TX_DONE)) && (!(irqStatus & SX126X_IRQ_TIMEOUT))) {
                delay(1);
                irqStatus = GetIrqStatus(handle);
            }
            if (handle->debugPrint) {
                ESP_LOGI(TAG, "irqStatus=0x%x", irqStatus);
                if (irqStatus & SX126X_IRQ_TX_DONE) {
                    ESP_LOGI(TAG, "SX126X_IRQ_TX_DONE");
                }
                if (irqStatus & SX126X_IRQ_TIMEOUT) {
                    ESP_LOGI(TAG, "SX126X_IRQ_TIMEOUT");
                }
            }
            handle->txActive = false;

            SetRx(handle, 0xFFFFFF);

            if (irqStatus & SX126X_IRQ_TX_DONE) {
                rv = true;
            }
        } else {
            rv = true;
        }
    }
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "Send rv=0x%x", rv);
    }
    if (rv == false)
        handle->txLost++;
    return rv;
}

bool LoRaSendAsync(sx126x_handle_t handle, uint8_t *pData, int16_t len) {
    if (handle->txActive)
        return false;

    handle->txActive = true;

    if (handle->PacketParams[2] == 0x00)
        handle->PacketParams[3] = len;
    WriteCommand(handle, SX126X_CMD_SET_PACKET_PARAMS, handle->PacketParams, 6);

    ClearIrqStatus(handle, SX126X_IRQ_ALL);
    WriteBuffer(handle, pData, len);
    SetTx(handle, 500);

    return true;
}

bool LoRaTxWaitDone(sx126x_handle_t handle, uint16_t *irq_status_out) {
    uint16_t irq_status = GetIrqStatus(handle);
    ClearIrqStatus(handle, SX126X_IRQ_ALL);
    handle->txActive = false;

    if (irq_status_out)
        *irq_status_out = irq_status;

    return (irq_status & SX126X_IRQ_TX_DONE) != 0;
}

bool ReceiveMode(sx126x_handle_t handle) {
    uint16_t irq;
    bool     rv = false;

    if (handle->txActive == false) {
        rv = true;
    } else {
        irq = GetIrqStatus(handle);
        if (irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT)) {
            SetRx(handle, 0xFFFFFF);
            handle->txActive = false;
            rv               = true;
        }
    }

    return rv;
}

void GetPacketStatus(sx126x_handle_t handle, int8_t *rssiPacket, int8_t *snrPacket) {
    uint8_t buf[4];
    ReadCommand(handle, SX126X_CMD_GET_PACKET_STATUS, buf, 4); // 0x14
    *rssiPacket = (buf[3] >> 1) * -1;
    (buf[2] < 128) ? (*snrPacket = buf[2] >> 2) : (*snrPacket = ((buf[2] - 256) >> 2));
}

void SetTxPower(sx126x_handle_t handle, int8_t txPowerInDbm) {
    SetPowerConfig(handle, txPowerInDbm, SX126X_PA_RAMP_200U);
}

void Reset(sx126x_handle_t handle) {
    delay(10);
    gpio_set_level(handle->dev_config.reset, LOW);
    delay(20);
    gpio_set_level(handle->dev_config.reset, HIGH);
    delay(10);
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(handle, BUSY_WAIT, "Reset", true);
}

void Wakeup(sx126x_handle_t handle) { GetStatus(handle); }

void SetStandby(sx126x_handle_t handle, uint8_t mode) {
    uint8_t data = mode;
    WriteCommand(handle, SX126X_CMD_SET_STANDBY, &data, 1); // 0x80
}

uint8_t GetStatus(sx126x_handle_t handle) {
    uint8_t rv;
    ReadCommand(handle, SX126X_CMD_GET_STATUS, &rv, 1); // 0xC0
    return rv;
}

void SetDio3AsTcxoCtrl(sx126x_handle_t handle, float voltage, uint32_t delay) {
    uint8_t buf[4];

    // buf[0] = tcxoVoltage & 0x07;
    if (fabs(voltage - 1.6) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_1_6;
    } else if (fabs(voltage - 1.7) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_1_7;
    } else if (fabs(voltage - 1.8) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_1_8;
    } else if (fabs(voltage - 2.2) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_2_2;
    } else if (fabs(voltage - 2.4) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_2_4;
    } else if (fabs(voltage - 2.7) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_2_7;
    } else if (fabs(voltage - 3.0) <= 0.001) {
        buf[0] = SX126X_DIO3_OUTPUT_3_0;
    } else {
        buf[0] = SX126X_DIO3_OUTPUT_3_3;
    }

    uint32_t delayValue = (float)delay / 15.625;
    buf[1]              = (uint8_t)((delayValue >> 16) & 0xFF);
    buf[2]              = (uint8_t)((delayValue >> 8) & 0xFF);
    buf[3]              = (uint8_t)(delayValue & 0xFF);

    WriteCommand(handle, SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4); // 0x97
}

void Calibrate(sx126x_handle_t handle, uint8_t calibParam) {
    uint8_t data = calibParam;
    WriteCommand(handle, SX126X_CMD_CALIBRATE, &data, 1); // 0x89
}

void SetDio2AsRfSwitchCtrl(sx126x_handle_t handle, uint8_t enable) {
    uint8_t data = enable;
    WriteCommand(handle, SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1); // 0x9D
}

void SetRfFrequency(sx126x_handle_t handle, uint32_t frequency) {
    uint8_t  buf[4];
    uint32_t freq = 0;

    CalibrateImage(handle, frequency);

    freq   = (uint32_t)((double)frequency / (double)FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    WriteCommand(handle, SX126X_CMD_SET_RF_FREQUENCY, buf, 4); // 0x86
}

void CalibrateImage(sx126x_handle_t handle, uint32_t frequency) {
    uint8_t calFreq[2];

    if (frequency > 900000000) {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    } else if (frequency > 850000000) {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    } else if (frequency > 770000000) {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    } else if (frequency > 460000000) {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    } else if (frequency > 425000000) {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    WriteCommand(handle, SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2); // 0x98
}

void SetRegulatorMode(sx126x_handle_t handle, uint8_t mode) {
    uint8_t data = mode;
    WriteCommand(handle, SX126X_CMD_SET_REGULATOR_MODE, &data, 1); // 0x96
}

void SetBufferBaseAddress(sx126x_handle_t handle, uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    WriteCommand(handle, SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2); // 0x8F
}

void SetPowerConfig(sx126x_handle_t handle, int8_t power, uint8_t rampTime) {
    uint8_t buf[2];

    if (power > 22) {
        power = 22;
    } else if (power < -3) {
        power = -3;
    }

    buf[0] = power;
    buf[1] = (uint8_t)rampTime;
    WriteCommand(handle, SX126X_CMD_SET_TX_PARAMS, buf, 2); // 0x8E
}

void SetPaConfig(sx126x_handle_t handle, uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    WriteCommand(handle, SX126X_CMD_SET_PA_CONFIG, buf, 4); // 0x95
}

void SetOvercurrentProtection(sx126x_handle_t handle, float currentLimit) {
    if ((currentLimit >= 0.0) && (currentLimit <= 140.0)) {
        uint8_t buf[1];
        buf[0] = (uint8_t)(currentLimit / 2.5);
        WriteRegister(handle, SX126X_REG_OCP_CONFIGURATION, buf, 1); // 0x08E7
    }
}

void SetSyncWord(sx126x_handle_t handle, int16_t sync) {
    uint8_t buf[2];

    buf[0] = (uint8_t)((sync >> 8) & 0x00FF);
    buf[1] = (uint8_t)(sync & 0x00FF);
    WriteRegister(handle, SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2); // 0x0740
}

void SetDioIrqParams(sx126x_handle_t handle, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask,
                     uint16_t dio3Mask) {
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);
    WriteCommand(handle, SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8); // 0x08
}

void SetStopRxTimerOnPreambleDetect(sx126x_handle_t handle, bool enable) {
    ESP_LOGI(TAG, "SetStopRxTimerOnPreambleDetect enable=%d", enable);
    // uint8_t data = (uint8_t)enable;
    uint8_t data = 0;
    if (enable)
        data = 1;
    WriteCommand(handle, SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1); // 0x9F
}

void SetLoRaSymbNumTimeout(sx126x_handle_t handle, uint8_t SymbNum) {
    uint8_t data = SymbNum;
    WriteCommand(handle, SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1); // 0xA0
}

void SetPacketType(sx126x_handle_t handle, uint8_t packetType) {
    uint8_t data = packetType;
    WriteCommand(handle, SX126X_CMD_SET_PACKET_TYPE, &data, 1); // 0x01
}

void SetModulationParams(sx126x_handle_t handle, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
                         uint8_t lowDataRateOptimize) {
    uint8_t data[4];
    // currently only LoRa supported
    data[0] = spreadingFactor;
    data[1] = bandwidth;
    data[2] = codingRate;
    data[3] = lowDataRateOptimize;
    WriteCommand(handle, SX126X_CMD_SET_MODULATION_PARAMS, data, 4); // 0x8B
}

void SetCadParams(sx126x_handle_t handle, uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin,
                  uint8_t cadExitMode, uint32_t cadTimeout) {
    uint8_t data[7];
    data[0] = cadSymbolNum;
    data[1] = cadDetPeak;
    data[2] = cadDetMin;
    data[3] = cadExitMode;
    data[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
    data[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
    data[6] = (uint8_t)(cadTimeout & 0xFF);
    WriteCommand(handle, SX126X_CMD_SET_CAD_PARAMS, data, 7); // 0x88
}

void SetCad(sx126x_handle_t handle) {
    uint8_t data = 0;
    WriteCommand(handle, SX126X_CMD_SET_CAD, &data, 0); // 0xC5
}

uint16_t GetIrqStatus(sx126x_handle_t handle) {
    uint8_t data[3];
    ReadCommand(handle, SX126X_CMD_GET_IRQ_STATUS, data, 3); // 0x12
    return (data[1] << 8) | data[2];
}

void ClearIrqStatus(sx126x_handle_t handle, uint16_t irq) {
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    WriteCommand(handle, SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2); // 0x02
}

void SetRx(sx126x_handle_t handle, uint32_t timeout) {
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "----- SetRx timeout=%" PRIu32, timeout);
    }
    SetStandby(handle, SX126X_STANDBY_RC);
    SetRxEnable(handle);
    uint8_t buf[3];
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);
    WriteCommand(handle, SX126X_CMD_SET_RX, buf, 3); // 0x82

    for (int retry = 0; retry < 10; retry++) {
        if ((GetStatus(handle) & 0x70) == 0x50)
            break;
        delay(1);
    }
    if ((GetStatus(handle) & 0x70) != 0x50) {
        ESP_LOGE(TAG, "SetRx Illegal Status");
        LoRaError(ERR_INVALID_SETRX_STATE);
    }
}

void SetRxEnable(sx126x_handle_t handle) {
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "SetRxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", handle->dev_config.txen, handle->dev_config.rxen);
    }
    if ((handle->dev_config.txen != -1) && (handle->dev_config.rxen != -1)) {
        gpio_set_level(handle->dev_config.rxen, HIGH);
        gpio_set_level(handle->dev_config.txen, LOW);
    }
}

void SetTx(sx126x_handle_t handle, uint32_t timeoutInMs) {
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "----- SetTx timeoutInMs=%" PRIu32, timeoutInMs);
    }
    SetStandby(handle, SX126X_STANDBY_RC);
    SetTxEnable(handle);
    uint8_t  buf[3];
    uint32_t tout = timeoutInMs;
    if (timeoutInMs != 0) {
        uint32_t timeoutInUs = timeoutInMs * 1000;
        tout                 = (uint32_t)(timeoutInUs / 0.015625);
    }
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "SetTx timeoutInMs=%" PRIu32 " tout=%" PRIu32, timeoutInMs, tout);
    }
    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t)(tout & 0xFF);
    WriteCommand(handle, SX126X_CMD_SET_TX, buf, 3); // 0x83

    for (int retry = 0; retry < 10; retry++) {
        if ((GetStatus(handle) & 0x70) == 0x60)
            break;
        vTaskDelay(1);
    }
    if ((GetStatus(handle) & 0x70) != 0x60) {
        ESP_LOGE(TAG, "SetTx Illegal Status");
        LoRaError(ERR_INVALID_SETTX_STATE);
    }
}

void SetTxEnable(sx126x_handle_t handle) {
    if (handle->debugPrint) {
        ESP_LOGI(TAG, "SetTxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", handle->dev_config.txen, handle->dev_config.rxen);
    }
    if ((handle->dev_config.txen != -1) && (handle->dev_config.rxen != -1)) {
        gpio_set_level(handle->dev_config.rxen, LOW);
        gpio_set_level(handle->dev_config.txen, HIGH);
    }
}

int GetPacketLost(sx126x_handle_t handle) { return handle->txLost; }

uint8_t GetRssiInst(sx126x_handle_t handle) {
    uint8_t buf[2];
    ReadCommand(handle, SX126X_CMD_GET_RSSI_INST, buf, 2); // 0x15
    return buf[1];
}

void GetRxBufferStatus(sx126x_handle_t handle, uint8_t *payloadLength, uint8_t *rxStartBufferPointer) {
    uint8_t buf[3];
    ReadCommand(handle, SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3); // 0x13
    *payloadLength        = buf[1];
    *rxStartBufferPointer = buf[2];
}

void WaitForIdleBegin(sx126x_handle_t handle, unsigned long timeout, char *text) {
    // ensure BUSY is low (state meachine ready)
    bool stop = false;
    for (int retry = 0; retry < 10; retry++) {
        if (retry == 9)
            stop = true;
        bool ret = WaitForIdle(handle, BUSY_WAIT, text, stop);
        if (ret == true)
            break;
        ESP_LOGW(TAG, "WaitForIdle fail retry=%d", retry);
        vTaskDelay(1);
    }
}

bool WaitForIdle(sx126x_handle_t handle, unsigned long timeout, char *text, bool stop) {
    bool       ret   = true;
    TickType_t start = xTaskGetTickCount();
    // delayMicroseconds(1);
    while (xTaskGetTickCount() - start < (timeout / portTICK_PERIOD_MS)) {
        if (gpio_get_level(handle->dev_config.busy) == 0)
            break;
        // delayMicroseconds(1);
        //  Give up CPU execution rights
        vTaskDelay(1);
    }
    if (gpio_get_level(handle->dev_config.busy)) {
        if (stop) {
            ESP_LOGE(TAG, "WaitForIdle Timeout text=%s timeout=%lu start=%" PRIu32, text, timeout, start);
            LoRaError(ERR_IDLE_TIMEOUT);
        } else {
            ESP_LOGW(TAG, "WaitForIdle Timeout text=%s timeout=%lu start=%" PRIu32, text, timeout, start);
            ret = false;
        }
    }
    return ret;
}

uint8_t ReadBuffer(sx126x_handle_t handle, uint8_t *rxData, int16_t rxDataLen) {
    uint8_t offset        = 0;
    uint8_t payloadLength = 0;
    GetRxBufferStatus(handle, &payloadLength, &offset);
    if (payloadLength > rxDataLen) {
        ESP_LOGW(TAG, "ReadBuffer rxDataLen too small. payloadLength=%d rxDataLen=%d", payloadLength, rxDataLen);
        return 0;
    }

    // ensure BUSY is low (state meachine ready)
    WaitForIdle(handle, BUSY_WAIT, "start ReadBuffer", true);

    // start transfer
    uint8_t *buf;
    buf = malloc(payloadLength + 3);
    if (buf != NULL) {
        buf[0] = SX126X_CMD_READ_BUFFER; // 0x1E
        buf[1] = offset;                 // offset in rx fifo
        buf[2] = SX126X_CMD_NOP;
        memset(&buf[3], SX126X_CMD_NOP, payloadLength);
        spi_read_byte(handle, buf, buf, payloadLength + 3);
        memcpy(rxData, &buf[3], payloadLength);
        free(buf);
    } else {
        ESP_LOGE(TAG, "ReadBuffer malloc fail");
        payloadLength = 0;
    }

    // wait for BUSY to go low
    WaitForIdle(handle, BUSY_WAIT, "end ReadBuffer", false);

    return payloadLength;
}

void WriteBuffer(sx126x_handle_t handle, uint8_t *txData, int16_t txDataLen) {
    // ensure BUSY is low (state meachine ready)
    WaitForIdle(handle, BUSY_WAIT, "start WriteBuffer", true);

    // start transfer
    uint8_t *buf;
    buf = malloc(txDataLen + 2);
    if (buf != NULL) {
        buf[0] = SX126X_CMD_WRITE_BUFFER; // 0x0E
        buf[1] = 0;                       // offset in tx fifo
        memcpy(&buf[2], txData, txDataLen);
        spi_write_byte(handle, buf, txDataLen + 2);
        free(buf);
    } else {
        ESP_LOGE(TAG, "WriteBuffer malloc fail");
    }

    // wait for BUSY to go low
    WaitForIdle(handle, BUSY_WAIT, "end WriteBuffer", false);
}

void WriteRegister(sx126x_handle_t handle, uint16_t reg, uint8_t *data, uint8_t numBytes) {
    WaitForIdle(handle, BUSY_WAIT, "start WriteRegister", true);

    if (handle->debugPrint) {
        ESP_LOGI(TAG, "WriteRegister: REG=0x%02x", reg);
        for (uint8_t n = 0; n < numBytes; n++) {
            ESP_LOGI(TAG, "DataOut:%02x ", data[n]);
        }
    }

    uint8_t *buf = malloc(numBytes + 3);
    if (buf != NULL) {
        buf[0] = SX126X_CMD_WRITE_REGISTER;
        buf[1] = (reg & 0xFF00) >> 8;
        buf[2] = reg & 0xFF;
        memcpy(&buf[3], data, numBytes);
        spi_write_byte(handle, buf, numBytes + 3);
        free(buf);
    } else {
        ESP_LOGE(TAG, "WriteRegister malloc fail");
    }

    WaitForIdle(handle, BUSY_WAIT, "end WriteRegister", false);
}

void ReadRegister(sx126x_handle_t handle, uint16_t reg, uint8_t *data, uint8_t numBytes) {
    WaitForIdle(handle, BUSY_WAIT, "start ReadRegister", true);

    if (handle->debugPrint) {
        ESP_LOGI(TAG, "ReadRegister: REG=0x%02x", reg);
    }

    uint8_t *buf = malloc(numBytes + 4);
    if (buf != NULL) {
        memset(buf, SX126X_CMD_NOP, numBytes + 4);
        buf[0] = SX126X_CMD_READ_REGISTER;
        buf[1] = (reg & 0xFF00) >> 8;
        buf[2] = reg & 0xFF;
        spi_read_byte(handle, buf, buf, numBytes + 4);
        memcpy(data, &buf[4], numBytes);
        if (handle->debugPrint) {
            for (uint8_t n = 0; n < numBytes; n++) {
                ESP_LOGI(TAG, "DataIn:%02x ", data[n]);
            }
        }
        free(buf);
    } else {
        ESP_LOGE(TAG, "ReadRegister malloc fail");
    }

    WaitForIdle(handle, BUSY_WAIT, "end ReadRegister", false);
}

// WriteCommand with retry
void WriteCommand(sx126x_handle_t handle, uint8_t cmd, uint8_t *data, uint8_t numBytes) {
    uint8_t status;
    for (int retry = 1; retry < 10; retry++) {
        status = WriteCommand2(handle, cmd, data, numBytes);
        ESP_LOGD(TAG, "status=%02x", status);
        if (status == 0)
            break;
        ESP_LOGW(TAG, "WriteCommand2 status=%02x retry=%d", status, retry);
    }
    if (status != 0) {
        ESP_LOGE(TAG, "SPI Transaction error:0x%02x", status);
        LoRaError(ERR_SPI_TRANSACTION);
    }
}

uint8_t WriteCommand2(sx126x_handle_t handle, uint8_t cmd, uint8_t *data, uint8_t numBytes) {
    WaitForIdle(handle, BUSY_WAIT, "start WriteCommand2", true);

    if (handle->debugPrint) {
        ESP_LOGI(TAG, "WriteCommand: CMD=0x%02x", cmd);
    }

    uint8_t status = 0;

    uint8_t *buf = malloc(numBytes + 1);
    if (buf != NULL) {
        buf[0] = cmd;
        memcpy(&buf[1], data, numBytes);
        spi_read_byte(handle, buf, buf, numBytes + 1);

        uint8_t cmd_status = buf[1] & 0x0E;
        switch (cmd_status) {
        case SX126X_STATUS_CMD_TIMEOUT:
        case SX126X_STATUS_CMD_INVALID:
        case SX126X_STATUS_CMD_FAILED:
            status = cmd_status;
            break;
        case 0:
        case 7:
            status = SX126X_STATUS_SPI_FAILED;
            break;
        default:
            break; // success
        }

        free(buf);
    } else {
        ESP_LOGE(TAG, "WriteCommand2 malloc fail");
        status = SX126X_STATUS_SPI_FAILED;
    }

    WaitForIdle(handle, BUSY_WAIT, "end WriteCommand2", false);
    return status;
}

void ReadCommand(sx126x_handle_t handle, uint8_t cmd, uint8_t *data, uint8_t numBytes) {
    WaitForIdleBegin(handle, BUSY_WAIT, "start ReadCommand");

    if (handle->debugPrint) {
        ESP_LOGI(TAG, "ReadCommand: CMD=0x%02x", cmd);
    }

    uint8_t *buf = malloc(numBytes + 1);
    if (buf != NULL) {
        memset(buf, SX126X_CMD_NOP, numBytes + 1);
        buf[0] = cmd;
        spi_read_byte(handle, buf, buf, numBytes + 1);
        if (data != NULL && numBytes)
            memcpy(data, &buf[1], numBytes);
        free(buf);
    } else {
        ESP_LOGE(TAG, "ReadCommand malloc fail");
    }

    esp_rom_delay_us(10);
    WaitForIdle(handle, BUSY_WAIT, "end ReadCommand", false);
}
