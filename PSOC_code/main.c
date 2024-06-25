#include <project.h>

// EEPROM AREA  //////////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

	// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

    // Status Return Levels ///////////////////////////////////////////////////////////////
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

    // Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

	// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						0
#define RIGHT                       1
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_BYTE_READ_SPEED          2
#define AX_RESET_LENGTH				2
#define AX_ACTION_LENGTH			2
#define AX_ID_LENGTH                4
#define AX_LR_LENGTH                4
#define AX_SRL_LENGTH               4
#define AX_RDT_LENGTH               4
#define AX_LEDALARM_LENGTH          4
#define AX_SALARM_LENGTH            4
#define AX_TL_LENGTH                4
#define AX_VL_LENGTH                6
#define AX_CM_LENGTH                6
#define AX_CS_LENGTH                6
#define AX_CCW_CW_LENGTH            8
#define AX_BD_LENGTH                4
#define AX_TEM_LENGTH               4
#define AX_MOVING_LENGTH            4
#define AX_RWS_LENGTH               4
#define AX_VOLT_LENGTH              4
#define AX_LED_LENGTH               4
#define AX_TORQUE_LENGTH            4
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5
#define AX_MT_LENGTH                5
#define AX_PUNCH_LENGTH             5
#define AX_SPEED_LENGTH             5
#define AX_GOAL_SP_LENGTH           7
#define AX_ACTION_CHECKSUM			250
#define BROADCAST_ID                254
#define AX_START                    255
#define AX_CCW_AL_L                 255 
#define AX_CCW_AL_H                 3
#define TIME_OUT                    10
#define TX_MODE                     1
#define RX_MODE                     0
#define LOCK                        1
#define ERROR_CHECK                 1


/// FUNCTIONS DECLARATIONS
// Debug
void debugPrint(const char *str);
void debugPrintInt(int n);

// Write
void LED_Control(uint8_t Status, uint8_t ID);
void Set_ID(uint8_t new_ID, uint8_t ID);
void Move(uint16_t Position, uint8_t ID);
void MoveSpeed(uint16_t Position, uint16_t Speed, uint8_t ID);
void TorqueStatus(uint8_t Status, uint8_t ID);
void SetReturnLevel(uint8_t level, uint8_t ID);
void reset(uint8_t ID);
void ping(uint8_t ID);
void MoveSpeedRW(uint16_t Position, uint16_t Speed, uint8_t ID);
void action(void);


//Read Error for Write Command
int ReadError(void);

// Read
int ReadVoltage(uint8_t ID);
int ReadTemperature(uint8_t ID);
int ReadPosition(uint8_t ID);
int ReadSpeed(uint8_t ID);



/// MAIN FUNCTION
int main(void)
 {
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // OFF LED
    LED_1_Write(0);
    
    // UART INITIALISATION
    UART_Start();
    UART_Debug_Start();
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    // SPI INITIALISATION
    SPIS_1_Start();
    char hexString[5];
    SPIS_1_ClearTxBuffer();
    SPIS_1_ClearRxBuffer();
    SPIS_1_ClearFIFO();

    
    // DYNA IDS DECLARATRION // ID 0XFE  Broadcasting ID
    uint8_t ID_0 = 0;
    uint8_t ID_1 = 1;
    
    // The Range for the Motors
    int flat_0 = 515;
    int flat_1 = 48;
    int max_input = 50;
    
    
    LED_1_Write(1);
    MoveSpeed(flat_0,100,ID_0);
    MoveSpeed(flat_1,100,ID_1);
    LED_Control(1,1);
    LED_Control(1,0);

    
    // Declare variables for storing extracted information
    uint16 receivedData;
    uint8 sign_0;
    uint8 sign_1;
    uint8 value_0;
    uint8 value_1;
    
    int input_0;
    int input_1;

    for (;;) 
    {
        if(SPIS_1_ReadRxStatus() & SPIS_1_STS_RX_FIFO_NOT_EMPTY) 
        {
            receivedData = SPIS_1_ReadRxData(); // Read data from SPI

            // Extracting Information
            debugPrint("Received Value: ");
            debugPrintInt(receivedData);

            // Extract sign_0 (bit 15)
            sign_0 = (receivedData >> 15) & 0x01;

            // Extract sign_1 (bit 14)
            sign_1 = (receivedData >> 14) & 0x01;

            // Extract value_0 (bits 13-8)
            value_0 = (receivedData >> 8) & 0x3F;

            // Extract value_1 (bits 7-2)
            value_1 = (receivedData >> 2) & 0x3F;

            // Check if the two least significant bits are zero
            if ((receivedData & 0x03) != 0) 
            {
                debugPrint("Error: Check bits are not zero.");
                break;
            }

            debugPrint("Sign_0: ");
            debugPrintInt(sign_0);

            debugPrint("Sign_1: ");
            debugPrintInt(sign_1);

            debugPrint("Value_0: ");
            debugPrintInt(value_0);

            debugPrint("Value_1: ");
            debugPrintInt(value_1);
            
            
            // Limits
            if (value_0 > max_input)
            {
                input_0 = max_input;
            }
            else
            {
                input_0 = value_0;
            }
            
            if (value_1 > max_input)
            {
                input_1 = max_input;
            }
            else
            {
                input_1 = value_1;
            }
            
            
            // Sign
            if (sign_0)
            {
                input_0 *= -1;
            }
            
            if (sign_1)
            {
                input_1 *= -1;
            }
            
            // Move Motor
            debugPrint("Input_0: ");
            debugPrintInt(input_0);
            debugPrint("Input_1: ");
            debugPrintInt(input_1);
            debugPrint("\n");
            
            MoveSpeed(flat_0-input_0,1023,ID_0);
            CyDelay(1);
            MoveSpeed(flat_1+input_1,1023,ID_1);
            
        }
    }
 }
 

/// FUNCTIONS
void debugPrint(const char *str) 
{
    UART_Debug_PutString(str);
}

void debugPrintInt(int n) 
{
    char string[5];
    sprintf(string, "%d\n", n);
    debugPrint(string);
}

int ReadError(void)
{
    // Bit 7: Reserved
    // Bit 6: Instruction Error - Set to 1 if an undefined instruction is sent or an action instruction is sent without a Reg_Write instruction.
    // Bit 5: Overload Error - Set to 1 if the specified maximum torque can't control the applied load.
    // Bit 4: Checksum Error - Set to 1 if the checksum of the instruction packet is incorrect.
    // Bit 3: Range Error - Set to 1 if the instruction sent is out of the defined range.
    // Bit 2: Overheating Error - Set to 1 if the internal temperature of the Dynamixel unit is above the operating temperature range as defined in the control table.
    // Bit 1: Angle Limit Error - Set as 1 if the Goal Position is set outside of the range between CW Angle Limit and CCW Angle Limit.
    // Bit 0: Input Voltage Error - Set to 1 if the voltage is out of the operating voltage range as defined in the control table.

    int time = 0;
    while ( (UART_GetRxBufferSize() < 6) && (time < TIME_OUT) )
    {
        time = time + 1;
        CyDelay(5);
    }
    
    // Read Response
    int error = -1;
    
    debugPrint("Bf size : ");
    debugPrintInt(UART_GetRxBufferSize());

    if (UART_GetRxBufferSize() > 0)
    {
        uint8_t incoming_Byte = UART_GetByte();
        
        if (incoming_Byte == 255)
        {
            UART_GetByte(); // Start BYtes
            UART_GetByte(); // ID
            UART_GetByte(); // Length
            
            error = UART_GetByte(); // Error Byte
            debugPrint("ERROR CHECK: Returned : ");
            debugPrintInt(error);
            
            if (error & (1 << 6))
            {
                debugPrint("Instruction Error\n");

            }
            if (error & (1 << 5))
            {
                debugPrint("Overload Error\n");

            }
            if (error & (1 << 4))
            {
                debugPrint("Checksum Error\n");

            }
            if (error & (1 << 3))
            {
                debugPrint("Range Error\n");

            }
            if (error & (1 << 2))
            {
                debugPrint("Overheating Error\n");

            }
            if (error & (1 << 1))
            {
                debugPrint("Angle Limit Error\n");

            }
            if (error & 1)
            {
                debugPrint("Input Voltage Error\n");

            }
            
            return error;
        }
    }
    debugPrint("ERROR CHECK: NO RETURN \n");
    return error;
}

void LED_Control(uint8_t Status, uint8_t ID)
{
    LED_1_Write(1);
    
    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status)) & 0xFF;   
    uint8_t packet[] = {AX_START, AX_START, ID, AX_LED_LENGTH, AX_WRITE_DATA, AX_LED, Status, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

    LED_1_Write(0);
    debugPrint("LED_Control\n");
    
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}


void Set_ID(uint8_t new_ID, uint8_t ID)
{
    LED_1_Write(1);
    
    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + new_ID)) & 0xFF;  
    uint8_t packet[] = {AX_START, AX_START, ID, AX_ID_LENGTH, AX_WRITE_DATA, AX_ID, new_ID, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

    LED_1_Write(0);
    debugPrint("Set_ID: ");
    debugPrintInt(ID);
    debugPrint(" -> ");
    debugPrintInt(new_ID);
    
    if (ERROR_CHECK)
    {   
        ReadError();
    }
    
}

void Move(uint16_t Position, uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    uint8_t Position_H = Position >> 8; // 16 bits - 2 x 8 bits variables
    uint8_t Position_L = Position;
    
    debugPrint("Position: ");
    debugPrintInt(Position_H);
    debugPrintInt(Position_L);
    
    
    const uint8_t length = 9;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    UART_PutArray(packet, length);

    LED_1_Write(0);
    debugPrint("Move\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void MoveSpeed(uint16_t Position, uint16_t Speed, uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    uint8_t Position_H = Position >> 8;
    uint8_t Position_L = Position; // 16 bits - 2 x 8 bits variables
    uint8_t Speed_H = Speed >> 8;
    uint8_t Speed_L = Speed; // 16 bits - 2 x 8 bits variables

    const uint8_t length = 11;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_SP_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;

    UART_PutArray(packet, length);
    
    LED_1_Write(0);
    debugPrint("MoveSpeed\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void TorqueStatus(uint8_t Status, uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    const uint8_t length = 8;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + Status)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_TORQUE_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_TORQUE_ENABLE;
    packet[6] = Status;
    packet[7] = Checksum;

    UART_PutArray(packet, length);
    LED_1_Write(0);
    debugPrint("TorqueStatus\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void SetReturnLevel(uint8_t level, uint8_t ID)
{
    // 0: Do not respond to any instructions
    // 1: Respond only to READ_DATA instructions
    // 2: Respond to all instructions
    
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    const uint8_t length = 8;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + level)) & 0xFF;


    packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_SRL_LENGTH;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_RETURN_LEVEL;
	packet[6] = level;
	packet[7] = Checksum;


    UART_PutArray(packet, length);
    LED_1_Write(0);   
    debugPrint("SetReturnLevel\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void reset(uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    const uint8_t length = 8;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_RESET_LENGTH;
	packet[4] = AX_RESET;
	packet[5] = Checksum;


    UART_PutArray(packet, length);
    LED_1_Write(0);
    debugPrint("reset\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void ping(uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    const uint8_t length = 6;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_READ_DATA + AX_PING)) & 0xFF;
	
	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_READ_DATA;
	packet[4] = AX_PING;
	packet[5] = Checksum;

    UART_PutArray(packet, length);
    LED_1_Write(0);
    debugPrint("ping\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void MoveSpeedRW(uint16_t Position, uint16_t Speed, uint8_t ID)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    uint8_t Position_H = Position >> 8;
    uint8_t Position_L = Position; // 16 bits - 2 x 8 bits variables
    uint8_t Speed_H = Speed >> 8;
    uint8_t Speed_L = Speed; // 16 bits - 2 x 8 bits variables

    const uint8_t length = 11;
    uint8_t packet[length];

    uint8_t Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_SP_LENGTH;
    packet[4] = AX_REG_WRITE;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;

    UART_PutArray(packet, length);
    
    LED_1_Write(0);
    debugPrint("MoveSpeed Register Write\n");
    if (ERROR_CHECK)
    {   
        ReadError();
    }
}

void action(void)
{
    LED_1_Write(1);
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    
    const uint8_t length = 6;
    uint8_t packet[length];

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = BROADCAST_ID;
    packet[3] = AX_ACTION_LENGTH;
    packet[4] = AX_ACTION;
    packet[5] = AX_ACTION_CHECKSUM;

    UART_PutArray(packet, length);
    LED_1_Write(0);
    debugPrint("action\n");
}


int ReadVoltage(uint8_t ID)
{
    LED_1_Write(1);
    
    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_VOLT_LENGTH + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ)) & 0xFF;
    uint8_t packet[] = {AX_START, AX_START, ID, AX_VOLT_LENGTH, AX_READ_DATA, AX_PRESENT_VOLTAGE, AX_BYTE_READ, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

   
    LED_1_Write(0);
    debugPrint("Read Voltage\n");

    // Wait for time out or Recived bytes
    int time = 0;
    while ( (UART_GetRxBufferSize() < 6) && (time < TIME_OUT) )
    {
        time = time + 1;
        CyDelay(5);
    }
    
    // Read Response
    int voltage = -1;
    
    debugPrint("Bf size : ");
    debugPrintInt(UART_GetRxBufferSize());

    if (UART_GetRxBufferSize() > 0)
    {
        uint8_t incoming_Byte = UART_GetByte();
        
        if (incoming_Byte == 255)
        {
            UART_GetByte(); // Start BYtes
            UART_GetByte(); // ID
            UART_GetByte(); // Length
            
            uint8_t error = UART_GetByte(); // Error Byte
            if (error != 0)
            {
                voltage = error;
                debugPrint("Error Detected");
                return voltage;
            }
            
            uint8_t voltage_byte = UART_GetByte();
            voltage = voltage_byte;
        }

    }
    
    return voltage;
}

int ReadTemperature(uint8_t ID)
{
    LED_1_Write(1);

    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_TEM_LENGTH + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ)) & 0xFF;
    uint8_t packet[] = {AX_START, AX_START, ID, AX_TEM_LENGTH, AX_READ_DATA, AX_PRESENT_TEMPERATURE, AX_BYTE_READ, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

    LED_1_Write(0);
    debugPrint("Read Temperature\n");

    // Wait for time out or Recived bytes
    int time = 0;
    while ((UART_GetRxBufferSize() < 6) && (time < TIME_OUT))
    {
        time = time + 1;
        CyDelay(5);
    }

    // Read Response
    int temperature = -1;

    debugPrint("Buffer size: ");
    debugPrintInt(UART_GetRxBufferSize());

    if (UART_GetRxBufferSize() > 0)
    {
        uint8_t incoming_Byte = UART_GetByte();

        if (incoming_Byte == 255)
        {
            UART_GetByte(); // Start Bytes
            UART_GetByte(); // ID
            UART_GetByte(); // Length

            uint8_t error = UART_GetByte(); // Error Byte
            if (error != 0)
            {
                temperature = error;
                debugPrint("Error Detected");
                return temperature;
            }

            uint8_t temperature_byte = UART_GetByte();
            temperature = temperature_byte;
        }
    }

    return temperature;
}

int ReadPosition(uint8_t ID)
{
    LED_1_Write(1);

    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS)) & 0xFF;
    uint8_t packet[] = {AX_START, AX_START, ID, AX_POS_LENGTH, AX_READ_DATA, AX_PRESENT_POSITION_L, AX_BYTE_READ_SPEED, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

    LED_1_Write(0);
    debugPrint("Read Position\n");

    // Wait for time out or Recived bytes
    int time = 0;
    while ((UART_GetRxBufferSize() < 7) && (time < TIME_OUT))
    {
        time = time + 1;
        CyDelay(5);
    }

    // Read Response
    int position = -1;

    debugPrint("Buffer size: ");
    debugPrintInt(UART_GetRxBufferSize());

    if (UART_GetRxBufferSize() > 0)
    {
        uint8_t incoming_Byte = UART_GetByte();

        if (incoming_Byte == 255)
        {
            UART_GetByte(); // Start Bytes
            UART_GetByte(); // ID
            UART_GetByte(); // Length

            int error = UART_GetByte(); // Error Byte
            if (error != 0)
            {
                position = error;
                debugPrint("Error Detected");
                return position;
            }

            uint8_t position_low_byte = UART_GetByte(); // Position Low Byte
            uint8_t position_high_byte = UART_GetByte(); // Position High Byte
            position = (position_high_byte << 8) + position_low_byte;
        }
    }

    return position;
}

int ReadSpeed(uint8_t ID)
{
    LED_1_Write(1);

    // WRITE DATA
    UART_ClearRxBuffer();
    UART_ClearTxBuffer();
    uint8_t Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_SPEED)) & 0xFF;
    uint8_t packet[] = {AX_START, AX_START, ID, AX_SPEED_LENGTH, AX_READ_DATA, AX_PRESENT_SPEED_L, AX_BYTE_READ_SPEED, Checksum}; // Initialize packet
    UART_PutArray(packet, 8); // Send data through UART

    LED_1_Write(0);
    debugPrint("Read Speed\n");

    // Wait for time out or Recived bytes
    int time = 0;
    while ((UART_GetRxBufferSize() < 7) && (time < TIME_OUT))
    {
        time = time + 1;
        CyDelay(5);
    }

    // Read Response
    int speed = -1;

    debugPrint("Buffer size: ");
    debugPrintInt(UART_GetRxBufferSize());

    if (UART_GetRxBufferSize() > 0)
    {
        uint8_t incoming_Byte = UART_GetByte();

        if (incoming_Byte == 255)
        {
            UART_GetByte(); // Start Bytes
            UART_GetByte(); // ID
            UART_GetByte(); // Length

            int error = UART_GetByte(); // Error Byte
            if (error != 0)
            {
                speed = error;
                debugPrint("Error Detected");
                return speed;
            }

            uint8_t speed_low_byte = UART_GetByte(); // Position Low Byte
            uint8_t speed_high_byte = UART_GetByte(); // Position High Byte
            speed = (speed_high_byte << 8) + speed_low_byte;
        }
    }

    return speed;
}
