
uint8_t uartTimeout = 100;      // Default timeout value for UART
uint16_t ledDelay = 1000;       // Default delay for LED toggle

// logMsg function prints a string, _out, to the console over the specified UART
void logMsg(UART_HandleTypeDef *huart, char _out[])
{
    HAL_UART_Transmit(huart, (uint8_t*) _out, strlen(_out), uartTimeout);
}

// logMsg function returns a char, c, over the specified UART
char logGetMsg(UART_HandleTypeDef *huart)
{
    char c;
    return HAL_UART_Receive(huart, (uint8_t*) &c, 1, uartTimeout);
}

/* USER CODE BEGIN 2 */

logMsg(&huart1, "welcome to Embedded controller programming \r\n");
logMsg(&huart1, "Enter g for toggling Green LED \r\n");
logMsg(&huart1, "Enter b for toggling Blue LED \r\n");
logMsg(&huart1, "Enter t to change timeout value to 1ms \r\n");
logMsg(&huart1, "Enter d to alternate LED delay value between 10ms and 1000ms\r\n");

/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    char input = logGetMsg(&huart1);
    
    switch(input)
    {
        case 'g':
            logMsg(&huart1, *input);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);   // Verify correct LEDs
            HAL_Delay(ledDelay);
            break;

        case 'b':
            logMsg(&huart1, *input);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);   // Verify correct LEDs
            HAL_Delay(ledDelay);
            break;

        case 't':
            logMsg(&huart1, *input);
            uartTimeout = 1;
            break;

        case 'd':
            logMsg(&huart1, *input);
            if (ledDelay == 1000)
            {
                ledDelay = 10;
                logMsg(&huart1, "LED delay set to 10ms\r\n");
            } 
            else if (ledDelay == 10)
            {
                ledDelay = 1000;
                logMsg(&huart1, "LED delay set to 1000ms\r\n");
            }
            break;

        default:
            logMsg(&huart1, "Unknown character recieved!\r\n");
            break;
    }

}