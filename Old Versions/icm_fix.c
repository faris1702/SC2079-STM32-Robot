void I2C_Bus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. De-init I2C peripheral
    HAL_I2C_DeInit(&hi2c2);

    // 2. Configure SCL and SDA as GPIO outputs open-drain
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Change port if your I2C2 pins are different

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_10; // SCL (check your board)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11; // SDA (check your board)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Toggle SCL about 10 times while SDA is high
    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    // 4. Generate a STOP condition manually
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // SDA low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // SCL high
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // SDA high

    // 5. Re-init I2C peripheral
    MX_I2C2_Init();
}

void icm20948_init(void)
{
    uint8_t data;
    uint8_t who_am_i;

    I2C_Bus_Recovery();

    // Select bank 0 (just to be safe)
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Read WHO_AM_I (0x00) - should be 0xEA
	HAL_I2C_Mem_Read(&hi2c2, 0x68 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);
	if (HAL_I2C_Mem_Read(&hi2c2, 0x68<<1, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000) == HAL_OK) {
	    if (who_am_i == 0xEA) {
	        OLED_ShowString(5,45,"ICM OK");
	    } else {
	        OLED_ShowString(5,45,"BAD ID");
	    }
	} else {
	    OLED_ShowString(5,45,"I2C ERR");
	}

    // Wake up
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Enable accel & gyro
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, 0x68 << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Disable ICM internal I2C master (required for BYPASS)
	data = 0x00; // USER_CTRL (0x03)
	HAL_I2C_Mem_Write(&hi2c2, 0x68<<1, 0x03, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Enable BYPASS so MCU can talk to AK09916 at 0x0C
	data = 0x02; // INT_PIN_CFG (0x0F): BYPASS_EN=1
	HAL_I2C_Mem_Write(&hi2c2, 0x68<<1, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Put AK09916 into continuous mode (e.g., 100 Hz)
	data = 0x08; // CNTL2 (0x31): 100 Hz
	HAL_I2C_Mem_Write(&hi2c2, 0x0C<<1, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}
