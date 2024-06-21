/*
 * keypad.c
 *
 *  Created on: Jun 21, 2024
 *      Author: vsaona
 */

#include "keypad.h"

uint32_t previousMillis = 0;

void keypad_Init(){
  HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
}

int getKeyAsInt(uint16_t GPIO_Pin) {
	  int row, column;
	  int button;
	  uint32_t currentMillis;
	  GPIO_TypeDef* port;
	  switch(GPIO_Pin) {
	  case keypadRow1_Pin:
		row = 0;
		port = keypadRow1_GPIO_Port;
		break;
	  case keypadRow2_Pin:
		row = 1;
		port = keypadRow2_GPIO_Port;
	  	break;
	  case keypadRow3_Pin:
		row = 2;
		port = keypadRow3_GPIO_Port;
	  	break;
	  case keypadRow4_Pin:
		row = 3;
		port = keypadRow4_GPIO_Port;
	  	break;
	  }
	  currentMillis = HAL_GetTick();
	  if (currentMillis - previousMillis > 10) {
	    if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	      return(10);
	    }
	    HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_RESET);
	    if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	      column = 1;
	    }
	    HAL_GPIO_WritePin(keypadColumn1_GPIO_Port, keypadColumn1_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_RESET);
	    if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	      column = 2;
	    }
	    HAL_GPIO_WritePin(keypadColumn2_GPIO_Port, keypadColumn2_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_RESET);
	    if(!HAL_GPIO_ReadPin(port, GPIO_Pin)) {
	      column = 3;
	    }
	    previousMillis = currentMillis;
	    button = column + row * 3;
	    if(button == 10) { // asterisk
	    	button = 11;
	    } else if (button == 11) { // zero
	    	button = 0;
	    } else if (button == 12) { // hash
	    	button = 12;
	    }

	  }
	  HAL_GPIO_WritePin(keypadColumn3_GPIO_Port, keypadColumn3_Pin, GPIO_PIN_SET);
	  return(button);
}
char getKeyAsChar(uint16_t GPIO_Pin)
{
	int key = getKeyAsInt(GPIO_Pin);
	switch(key) {
	case 10:
	  return(10);
	case 11:
      return('*');
	case 12:
	  return('#');
	default:
	  return('\0' + key);
	}
}
