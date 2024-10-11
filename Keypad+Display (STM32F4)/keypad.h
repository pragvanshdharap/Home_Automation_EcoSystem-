 /*header file for Keypad  */
#define ROW_1_Pin GPIO_PIN_0
#define ROW_2_Pin GPIO_PIN_2
#define ROW_3_Pin GPIO_PIN_3
#define ROW_4_Pin GPIO_PIN_4
#define COL_1_Pin GPIO_PIN_5
#define COL_2_Pin GPIO_PIN_6
#define COL_3_Pin GPIO_PIN_7


GPIO_TypeDef* ROW_1_Port = GPIOA;
GPIO_TypeDef* ROW_2_Port = GPIOA;
GPIO_TypeDef* ROW_3_Port = GPIOA;
GPIO_TypeDef* ROW_4_Port = GPIOA;
GPIO_TypeDef* COL_1_Port = GPIOA;
GPIO_TypeDef* COL_2_Port = GPIOA;
GPIO_TypeDef* COL_3_Port = GPIOA;


void keypad_init(void)
{
  // Configure GPIO pins for keypad matrix
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = ROW_1_Pin | ROW_2_Pin | ROW_3_Pin | ROW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROW_1_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(ROW_2_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(ROW_3_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(ROW_4_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL_1_Pin | COL_2_Pin | COL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COL_1_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(COL_2_Port, &GPIO_InitStruct);
  HAL_GPIO_Init(COL_3_Port, &GPIO_InitStruct);

}


char key_pressed = '0';
char entered_password[5]; // Buffer for 4 digits + null terminator
char password[5] = "5105";  // Correct password
uint8_t ind = 0;
uint8_t password_correct = 0;

char keypad_scan(void)
{
	HAL_Delay(500);
  char keys[4][3] = {{'1', '2', '3'},
                     {'4', '5', '6'},
                     {'7', '8', '9'},
                     {'*', '0', '#'}};

  for(int i = 0; i < 3; i++)
  {
    // Set current column as output and low
    switch(i)
    {
      case 0:
        HAL_GPIO_WritePin(COL_1_Port, COL_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(COL_2_Port, COL_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(COL_3_Port, COL_3_Pin, GPIO_PIN_SET);
       break;

      case 1:
        HAL_GPIO_WritePin(COL_1_Port, COL_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(COL_2_Port, COL_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COL_3_Port, COL_3_Pin, GPIO_PIN_SET);
	    break;

	  case 2:
		HAL_GPIO_WritePin(COL_1_Port, COL_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL_2_Port, COL_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL_3_Port, COL_3_Pin, GPIO_PIN_RESET);
		break;

}
	// Read current rows
	if(HAL_GPIO_ReadPin(ROW_1_Port, ROW_1_Pin) == GPIO_PIN_RESET)
	  return keys[0][i];
	if(HAL_GPIO_ReadPin(ROW_2_Port, ROW_2_Pin) == GPIO_PIN_RESET)
	  return keys[1][i];
	if(HAL_GPIO_ReadPin(ROW_3_Port, ROW_3_Pin) == GPIO_PIN_RESET)
	  return keys[2][i];
	if(HAL_GPIO_ReadPin(ROW_4_Port, ROW_4_Pin) == GPIO_PIN_RESET)
	  return keys[3][i];
	}

return 0; // No key pressed
}

