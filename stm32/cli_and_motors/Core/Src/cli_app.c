#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stepper.h"
// #include "stm32f4xx_hal_gpio.h"
#define MAX_INPUT_LENGTH 50
#define USING_VS_CODE_TERMINAL 0
#define USING_OTHER_TERMINAL 1 // e.g. Putty, TerraTerm
char cOutputBuffer[configCOMMAND_INT_MAX_OUTPUT_SIZE], pcInputString[MAX_INPUT_LENGTH];
extern const CLI_Command_Definition_t xCommandList[];
int8_t cRxedChar;
const char * cli_prompt = "\r\ncli> ";
/* CLI escape sequences*/
uint8_t backspace[] = "\b \b";
uint8_t backspace_tt[] = " \b";
uint8_t flag = 0;
extern stepperXTaskHandle;
extern stepperYTaskHandle;
extern rotServoTaskHandle;
extern linServoTaskHandle;
extern linActTaskHandle;

int _write(int file, char *data, int len)
{
    UNUSED(file);
    // Transmit data using UART2
    for (int i = 0; i < len; i++)
    {
        // Send the character
        USART2->DR = (uint16_t)data[i];
        // Wait for the transmit buffer to be empty
        while (!(USART2->SR & USART_SR_TXE));
    }
    return len;
}
//*****************************************************************************
BaseType_t cmd_clearScreen(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    printf("\033[2J\033[1;1H");
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_toggle_led(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString)
{
    (void)pcCommandString; // contains the command string
    (void)xWriteBufferLen; // contains the length of the write buffer
    
    /* Toggle the LED */
    if (flag)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    flag = !flag;

    
    /* Write the response to the buffer */
    uint8_t string[] = "LED toggled\r\n";
    strcpy(pcWriteBuffer, (char *)string);
    
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_hello(char *pcWriteBuffer, size_t xWriteBufferLen,
    const char *pcCommandString)
{
    (void)pcCommandString; // contains the command string
    (void)xWriteBufferLen; // contains the length of the write buffer

    /* Write the response to the buffer */
    uint8_t string[] = "Hello World!\r\n";
    strcpy(pcWriteBuffer, (char *)string);

    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_add(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString)
{
    char *pcParameter1, *pcParameter2;
    BaseType_t xParameter1StringLength, xParameter2StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength
                        );
    pcParameter2 = FreeRTOS_CLIGetParameter
                        (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          2,
                          /* Store the parameter string length. */
                          &xParameter2StringLength
                        );
    // convert the string to a number
    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    int32_t xValue2 = strtol(pcParameter2, NULL, 10);
    // add the two numbers
    int32_t xResultValue = xValue1 + xValue2;
    // convert the result to a string
    char cResultString[10];
    itoa(xResultValue, cResultString, 10);
    // copy the result to the write buffer
    strcpy(pcWriteBuffer, cResultString);
    
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_stepx(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString)
{
    char *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength
                        );
    // convert the string to a number

    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    xTaskNotify(stepperXTaskHandle, xValue1, eSetValueWithOverwrite);
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_stepy(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString)
{
    char *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                         /* The command string itself. */
                         pcCommandString,
                         /* Return the first parameter. */
                         1,
                         /* Store the parameter string length. */
                         &xParameter1StringLength
                        );
    // convert the string to a number

    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    xTaskNotify(stepperYTaskHandle, xValue1, eSetValueWithOverwrite);
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_servo_rot(char *pcWriteBuffer, size_t xWriteBufferLen,
    const char *pcCommandString)
{
    char *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                        /* The command string itself. */
                        pcCommandString,
                        /* Return the first parameter. */
                        1,
                        /* Store the parameter string length. */
                        &xParameter1StringLength
                        );
    // convert the string to a number

    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    xTaskNotify(rotServoTaskHandle, xValue1, eSetValueWithOverwrite);
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_servo_lin(char *pcWriteBuffer, size_t xWriteBufferLen,
    const char *pcCommandString)
{
    char *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                        /* The command string itself. */
                        pcCommandString,
                        /* Return the first parameter. */
                        1,
                        /* Store the parameter string length. */
                        &xParameter1StringLength
                        );
    // convert the string to a number

    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    xTaskNotify(linServoTaskHandle, xValue1, eSetValueWithOverwrite);
    return pdFALSE;
}
//*****************************************************************************
BaseType_t cmd_lin_act(char *pcWriteBuffer, size_t xWriteBufferLen,
    const char *pcCommandString)
{
    char *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
    the command string. The name of the source file is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter
                        (
                        /* The command string itself. */
                        pcCommandString,
                        /* Return the first parameter. */
                        1,
                        /* Store the parameter string length. */
                        &xParameter1StringLength
                        );
    // convert the string to a number

    int32_t xValue1 = strtol(pcParameter1, NULL, 10);
    xTaskNotify(linActTaskHandle, xValue1, eSetValueWithOverwrite);
    return pdFALSE;
}


const CLI_Command_Definition_t xCommandList[] = {
    {
        .pcCommand = "cls", /* The command string to type. */
        .pcHelpString = "cls:\r\n Clears screen\r\n\r\n",
        .pxCommandInterpreter = cmd_clearScreen, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* No parameters are expected. */
    },
    {
        .pcCommand = "toggleled", /* The command string to type. */
        .pcHelpString = "toggleled n:\r\n toggles led n amount of times\r\n\r\n",
        .pxCommandInterpreter = cmd_toggle_led, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* No parameters are expected. */
    },
    {
        .pcCommand = "add", /* The command string to type. */
        .pcHelpString = "add n:\r\n add two numbers\r\n\r\n",
        .pxCommandInterpreter = cmd_add, /* The function to run. */
        .cExpectedNumberOfParameters = 2 /* 2 parameters are expected. */
    },
    {
        .pcCommand = "hello", /* The command string to type. */
        .pcHelpString = "hello: prints: \"HelloWorld\"\r\n\r\n",
        .pxCommandInterpreter = cmd_hello, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* 2 parameters are expected. */
    },
    {
        .pcCommand = "stepx",
        .pcHelpString = "stepx: \r\n moves stepper in the x axis\r\n\r\n",
        .pxCommandInterpreter = cmd_stepx,
        .cExpectedNumberOfParameters = 1 
    },
    {
        .pcCommand = "stepy",
        .pcHelpString = "stepy: \r\n moves stepper in the y axis\r\n\r\n",
        .pxCommandInterpreter = cmd_stepy,
        .cExpectedNumberOfParameters = 1 
    },
    {
        .pcCommand = "servo_rot",
        .pcHelpString = "servo_rot: \r\n rotates servo to a given angle\r\n\r\n",
        .pxCommandInterpreter = cmd_servo_rot,
        .cExpectedNumberOfParameters = 1 
    },
    {
        .pcCommand = "servo_lin",
        .pcHelpString = "servo_lin: \r\n moves linear servo to a given pctg length\r\n\r\n",
        .pxCommandInterpreter = cmd_servo_lin,
        .cExpectedNumberOfParameters = 1 
    },
    {
        .pcCommand = "lin_act",
        .pcHelpString = "lin_act: \r\n moves lin actuator to a given pctg length\r\n\r\n",
        .pxCommandInterpreter = cmd_lin_act,
        .cExpectedNumberOfParameters = 1 
    },
    {
        .pcCommand = NULL /* simply used as delimeter for end of array*/
    }
};

void vRegisterCLICommands(void){
    //iterate thourgh the list of commands and register them
    for (int i = 0; xCommandList[i].pcCommand != NULL; i++)
    {
        FreeRTOS_CLIRegisterCommand(&xCommandList[i]);
    }
}
/*************************************************************************************************/
void cliWrite(const char *str)
{
   printf("%s", str);
   // flush stdout
   fflush(stdout);
}
/*************************************************************************************************/
void handleNewline(const char *const pcInputString, char *cOutputBuffer, uint8_t *cInputIndex)
{
    cliWrite("\r\n");

    BaseType_t xMoreDataToFollow;
    do
    {     
        xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString, cOutputBuffer, configCOMMAND_INT_MAX_OUTPUT_SIZE);
        cliWrite(cOutputBuffer);
    } while (xMoreDataToFollow != pdFALSE);

    cliWrite(cli_prompt);
    *cInputIndex = 0;
    memset((void*)pcInputString, 0x00, MAX_INPUT_LENGTH);
}
/*************************************************************************************************/
void handleBackspace(uint8_t *cInputIndex, char *pcInputString)
{
    if (*cInputIndex > 0)
    {
        (*cInputIndex)--;
        pcInputString[*cInputIndex] = '\0';

#if USING_VS_CODE_TERMINAL
        cliWrite((char *)backspace);
#elif USING_OTHER_TERMINAL
        cliWrite((char *)backspace_tt);
#endif
    }
    else
    {
#if USING_OTHER_TERMINAL
        uint8_t right[] = "\x1b\x5b\x43";
        cliWrite((char *)right);
#endif
    }
}
/*************************************************************************************************/
void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString)
{
    if (cRxedChar == '\r')
    {
        return;
    }
    else if (cRxedChar == (uint8_t)0x08 || cRxedChar == (uint8_t)0x7F)
    {
        handleBackspace(cInputIndex, pcInputString);
    }
    else
    {
        if (*cInputIndex < MAX_INPUT_LENGTH)
        {
            pcInputString[*cInputIndex] = cRxedChar;
            (*cInputIndex)++;
        }
    }
}
/*************************************************************************************************/
void vCommandConsoleTask(void *pvParameters)
{
    uint8_t cInputIndex = 0; // simply used to keep track of the index of the input string
    uint32_t receivedValue; // used to store the received value from the notification
    UNUSED(pvParameters);
    vRegisterCLICommands();
    
    for (;;)
    {
        xTaskNotifyWait(pdFALSE,    // Don't clear bits on entry
                                  0,  // Clear all bits on exit
                                  &receivedValue, // Receives the notification value
                                  portMAX_DELAY); // Wait indefinitely
        //echo recevied char
        cRxedChar = receivedValue & 0xFF;
        // cliWrite((char *)&cRxedChar);
        if (cRxedChar == '\r' || cRxedChar == '\n')
        {
            // user pressed enter, process the command
            handleNewline(pcInputString, cOutputBuffer, &cInputIndex);
        }
        else
        {
            // user pressed a character add it to the input string
            handleCharacterInput(&cInputIndex, pcInputString);
        }
    }
}
#endif /* CLI_COMMANDS_H */
