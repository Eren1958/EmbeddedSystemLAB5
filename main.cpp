//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"
#include <cstdio>
#include <cstring>

//=====[Defines]===============================================================

#define NUMBER_OF_KEYS                    4
#define TIME_INCREMENT_MS                10
#define DEBOUNCE_KEY_TIME_MS             40
#define KEYPAD_NUMBER_OF_ROWS             4
#define KEYPAD_NUMBER_OF_COLS             4
#define NUMBER_OF_AVG_SAMPLES           100
#define EVENT_LOG_SIZE                    5
#define UART_MSG_TIME_MS               1000
#define BUZZER_TOGGLE_TIME_MS           150

//=====[Declaration of public data types]======================================

typedef enum {
    MATRIX_KEYPAD_SCANNING,
    MATRIX_KEYPAD_DEBOUNCE,
    MATRIX_KEYPAD_KEY_HOLD_PRESSED
} matrixKeypadState_t;

typedef struct {
    float temperature;
    float gas;
} eventLog_t;

//=====[Declaration and initialization of public global objects]===============

// Existing hardware from your subsection
DigitalIn alarmTestButton(BUTTON1);
DigitalIn mq2Digital(PE_12);          // optional digital helper from subsection

DigitalOut alarmLed(LED1);
DigitalOut incorrectCodeLed(LED3);
DigitalOut systemBlockedLed(LED2);

DigitalInOut sirenPin(PE_10);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

AnalogIn lm35(A1);
AnalogIn mq2Analog(A2);               // for gas level reading
AnalogIn potentiometer(A0);

// Keypad pins from subsection
DigitalOut keypadRowPins[KEYPAD_NUMBER_OF_ROWS] = {PB_3, PB_5, PC_7, PA_15};
DigitalIn  keypadColPins[KEYPAD_NUMBER_OF_COLS] = {PB_12, PB_13, PB_15, PC_6};

//=====[Declaration and initialization of public global variables]=============

bool alarmState = OFF;
bool incorrectCode = false;

int numberOfIncorrectCodes = 0;
int matrixKeypadCodeIndex = 0;

char codeSequence[NUMBER_OF_KEYS] = {'1', '8', '0', '5'};
char enteredCode[NUMBER_OF_KEYS]  = {'\0', '\0', '\0', '\0'};

float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35ReadingsSum = 0.0f;
float lm35ReadingsAverage = 0.0f;
float temperatureC = 0.0f;
float gasPpm = 0.0f;

float temperatureThreshold = 25.0f;
float gasThreshold = 0.0f;

bool gasWarning = false;
bool tempWarning = false;

int accumulatedDebounceMatrixKeypadTime = 0;
char matrixKeypadLastKeyPressed = '\0';

char matrixKeypadIndexToCharArray[] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

matrixKeypadState_t matrixKeypadState;

eventLog_t eventLog[EVENT_LOG_SIZE];
int eventCount = 0;
int eventWriteIndex = 0;

int uartAccumulatedTime = 0;
int buzzerAccumulatedTime = 0;
bool buzzerOutputState = false;

//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();

void pcSerialComStringWrite(const char* str);
void pcSerialComCharWrite(char chr);

void lm35ReadingsArrayInit();
float analogReadingScaledWithTheLM35Formula(float analogReading);

void sensorReadingsUpdate();
void thresholdsUpdate();
void warningsUpdate();
void alarmUpdate();
void outputsUpdate();
void uartUpdate();

void addEventLog(float tempValue, float gasValue);
void showEventLog();

bool areEqual();

void matrixKeypadInit();
char matrixKeypadScan();
char matrixKeypadUpdate();
void keypadTask();

//=====[Main function, the program entry point after power on or reset]========

int main()
{
    inputsInit();
    outputsInit();

    pcSerialComStringWrite("Main Task 5 Started\r\n");

    while (true) {
        sensorReadingsUpdate();
        thresholdsUpdate();
        warningsUpdate();
        alarmUpdate();
        keypadTask();
        outputsUpdate();
        uartUpdate();
        delay(TIME_INCREMENT_MS);
    }
}

//=====[Implementations of public functions]===================================

void inputsInit()
{
    alarmTestButton.mode(PullDown);
    sirenPin.mode(OpenDrain);
    sirenPin.input();

    lm35ReadingsArrayInit();
    matrixKeypadInit();
}

void outputsInit()
{
    alarmLed = OFF;
    incorrectCodeLed = OFF;
    systemBlockedLed = OFF;
}

void pcSerialComStringWrite(const char* str)
{
    uartUsb.write(str, strlen(str));
}

void pcSerialComCharWrite(char chr)
{
    char buffer[2];
    buffer[0] = chr;
    buffer[1] = '\0';
    uartUsb.write(buffer, 1);
}

void lm35ReadingsArrayInit()
{
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsArray[i] = 0.0f;
    }
}

float analogReadingScaledWithTheLM35Formula(float analogReading)
{
    return (analogReading * 3.3f / 0.01f);
}

void sensorReadingsUpdate()
{
    static int lm35SampleIndex = 0;

    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if (lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }

    lm35ReadingsSum = 0.0f;
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum += lm35ReadingsArray[i];
    }

    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
    temperatureC = analogReadingScaledWithTheLM35Formula(lm35ReadingsAverage);

    // MQ-2 analog mapped to 0-800 ppm for task requirement
    gasPpm = mq2Analog.read() * 800.0f;

    // helper button from old subsection
    if (alarmTestButton) {
        temperatureC = 45.0f;
        gasPpm = 700.0f;
    }

    // optional digital MQ-2 assist from subsection
    if (!mq2Digital) {
        gasPpm = 800.0f;
    }
}

void thresholdsUpdate()
{
    float potValue = potentiometer.read();

    // Main Task 5 requirement:
    // Temperature threshold: 25C to 37C
    // Gas threshold: 0 to 800 ppm
    temperatureThreshold = 25.0f + (potValue * 12.0f);
    gasThreshold = potValue * 800.0f;
}

void warningsUpdate()
{
    tempWarning = (temperatureC > temperatureThreshold);
    gasWarning  = (gasPpm > gasThreshold);
}

void alarmUpdate()
{
    static bool previousAlarmCondition = false;
    bool currentAlarmCondition = (tempWarning || gasWarning);

    if (currentAlarmCondition) {
        alarmState = ON;
    }

    // log only when alarm condition becomes active
    if (currentAlarmCondition && !previousAlarmCondition) {
        addEventLog(temperatureC, gasPpm);
    }

    previousAlarmCondition = currentAlarmCondition;
}

void outputsUpdate()
{
    if (alarmState) {
        alarmLed = ON;

        buzzerAccumulatedTime += TIME_INCREMENT_MS;
        if (buzzerAccumulatedTime >= BUZZER_TOGGLE_TIME_MS) {
            buzzerAccumulatedTime = 0;
            buzzerOutputState = !buzzerOutputState;
        }

        sirenPin.output();
        if (buzzerOutputState) {
            sirenPin = LOW;   // active low
        } else {
            sirenPin = HIGH;
        }
    } else {
        alarmLed = OFF;
        sirenPin.input();
        buzzerOutputState = false;
        buzzerAccumulatedTime = 0;
    }

    incorrectCodeLed = incorrectCode ? ON : OFF;
    systemBlockedLed = (numberOfIncorrectCodes >= 5) ? ON : OFF;
}

void uartUpdate()
{
    char buffer[200];

    uartAccumulatedTime += TIME_INCREMENT_MS;

    if (uartAccumulatedTime >= UART_MSG_TIME_MS) {
        uartAccumulatedTime = 0;

        sprintf(
            buffer,
            "Temp: %.2f C | Temp Threshold: %.2f C | Gas: %.2f ppm | Gas Threshold: %.2f ppm\r\n",
            temperatureC,
            temperatureThreshold,
            gasPpm,
            gasThreshold
        );
        pcSerialComStringWrite(buffer);

        if (alarmState) {
            pcSerialComStringWrite("Enter 4-Digit Code to Deactivate\r\n");
        } else {
            pcSerialComStringWrite("System Normal\r\n");
        }
    }
}

void addEventLog(float tempValue, float gasValue)
{
    eventLog[eventWriteIndex].temperature = tempValue;
    eventLog[eventWriteIndex].gas = gasValue;

    eventWriteIndex++;
    if (eventWriteIndex >= EVENT_LOG_SIZE) {
        eventWriteIndex = 0;
    }

    if (eventCount < EVENT_LOG_SIZE) {
        eventCount++;
    }
}

void showEventLog()
{
    char buffer[120];

    pcSerialComStringWrite("\r\n--- Last 5 Events ---\r\n");

    if (eventCount == 0) {
        pcSerialComStringWrite("No events stored.\r\n\r\n");
        return;
    }

    int startIndex = eventWriteIndex - eventCount;
    if (startIndex < 0) {
        startIndex += EVENT_LOG_SIZE;
    }

    for (int i = 0; i < eventCount; i++) {
        int index = (startIndex + i) % EVENT_LOG_SIZE;
        sprintf(
            buffer,
            "Event %d -> Temp: %.2f C | Gas: %.2f ppm\r\n",
            i + 1,
            eventLog[index].temperature,
            eventLog[index].gas
        );
        pcSerialComStringWrite(buffer);
    }

    pcSerialComStringWrite("\r\n");
}

bool areEqual()
{
    for (int i = 0; i < NUMBER_OF_KEYS; i++) {
        if (codeSequence[i] != enteredCode[i]) {
            return false;
        }
    }
    return true;
}

void matrixKeypadInit()
{
    matrixKeypadState = MATRIX_KEYPAD_SCANNING;

    for (int pinIndex = 0; pinIndex < KEYPAD_NUMBER_OF_COLS; pinIndex++) {
        keypadColPins[pinIndex].mode(PullUp);
    }
}

char matrixKeypadScan()
{
    for (int r = 0; r < KEYPAD_NUMBER_OF_ROWS; r++) {

        for (int i = 0; i < KEYPAD_NUMBER_OF_ROWS; i++) {
            keypadRowPins[i] = ON;
        }

        keypadRowPins[r] = OFF;

        for (int c = 0; c < KEYPAD_NUMBER_OF_COLS; c++) {
            if (keypadColPins[c] == OFF) {
                return matrixKeypadIndexToCharArray[r * KEYPAD_NUMBER_OF_ROWS + c];
            }
        }
    }
    return '\0';
}

char matrixKeypadUpdate()
{
    char keyDetected = '\0';
    char keyReleased = '\0';

    switch (matrixKeypadState) {

        case MATRIX_KEYPAD_SCANNING:
            keyDetected = matrixKeypadScan();
            if (keyDetected != '\0') {
                matrixKeypadLastKeyPressed = keyDetected;
                accumulatedDebounceMatrixKeypadTime = 0;
                matrixKeypadState = MATRIX_KEYPAD_DEBOUNCE;
            }
            break;

        case MATRIX_KEYPAD_DEBOUNCE:
            if (accumulatedDebounceMatrixKeypadTime >= DEBOUNCE_KEY_TIME_MS) {
                keyDetected = matrixKeypadScan();
                if (keyDetected == matrixKeypadLastKeyPressed) {
                    matrixKeypadState = MATRIX_KEYPAD_KEY_HOLD_PRESSED;
                } else {
                    matrixKeypadState = MATRIX_KEYPAD_SCANNING;
                }
            }
            accumulatedDebounceMatrixKeypadTime += TIME_INCREMENT_MS;
            break;

        case MATRIX_KEYPAD_KEY_HOLD_PRESSED:
            keyDetected = matrixKeypadScan();
            if (keyDetected != matrixKeypadLastKeyPressed) {
                if (keyDetected == '\0') {
                    keyReleased = matrixKeypadLastKeyPressed;
                }
                matrixKeypadState = MATRIX_KEYPAD_SCANNING;
            }
            break;

        default:
            matrixKeypadInit();
            break;
    }

    return keyReleased;
}

void keypadTask()
{
    char keyReleased = matrixKeypadUpdate();

    if (keyReleased == '\0') {
        return;
    }

    // optional echo
    pcSerialComCharWrite(keyReleased);
    pcSerialComStringWrite("\r\n");

    if (numberOfIncorrectCodes >= 5) {
        return;
    }

    if (keyReleased >= '0' && keyReleased <= '9') {
        if (matrixKeypadCodeIndex < NUMBER_OF_KEYS) {
            enteredCode[matrixKeypadCodeIndex] = keyReleased;
            matrixKeypadCodeIndex++;
        }
        return;
    }

    if (keyReleased == '#') {
        // if 4 digits entered, treat as ENTER for deactivation
        if (matrixKeypadCodeIndex == NUMBER_OF_KEYS) {
            if (alarmState) {
                if (areEqual()) {
                    alarmState = OFF;
                    incorrectCode = false;
                    numberOfIncorrectCodes = 0;
                    pcSerialComStringWrite("Correct code. Alarm deactivated.\r\n");
                } else {
                    incorrectCode = true;
                    numberOfIncorrectCodes++;
                    pcSerialComStringWrite("Incorrect code.\r\n");
                }
            } else {
                pcSerialComStringWrite("Alarm is already OFF.\r\n");
            }

            matrixKeypadCodeIndex = 0;
            for (int i = 0; i < NUMBER_OF_KEYS; i++) {
                enteredCode[i] = '\0';
            }
        } else {
            // if no 4-digit code entered, show log
            showEventLog();

            matrixKeypadCodeIndex = 0;
            for (int i = 0; i < NUMBER_OF_KEYS; i++) {
                enteredCode[i] = '\0';
            }
        }
    }

    if (keyReleased == '*') {
        matrixKeypadCodeIndex = 0;
        for (int i = 0; i < NUMBER_OF_KEYS; i++) {
            enteredCode[i] = '\0';
        }
        pcSerialComStringWrite("Code entry cleared.\r\n");
    }
}
