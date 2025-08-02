#include "gnss_sdk/gnss_protocol/gnss_uart_parser.h"

//$GPGGA,000000.00,0000.0000,S,00000.0000,W,0,00,0.0,0.00,M,0.00,M,,*00
// 用于状态机，表示当前解析处于 "等待帧头" 还是 "等待帧数据" 阶段。
typedef enum
{
    WAIT_FOR_HEAD = 0, // 等待帧头
    WAIT_FOR_DATA, // 等待帧数据
} GnssSerialDecodeState;

#define PAYLOAD_BUFFER_SIZE (GNSS_FRAME_SIZE * 2)

#define FRAME_SOF_LEN ((uint8_t)2)
#define FRAME_FIXED_FIELD_LEN ((uint8_t)4)

#define FRAME_SOF1 ((uint8_t)0x5a)
#define FRAME_SOF2 ((uint8_t)0xa5)


#define FRAME_TYPE_CONTROL ((uint8_t)0x55)
#define FRAME_TYPE_STATUS ((uint8_t)0xaa)

#define FRAME_NONE_ID ((uint8_t)0x00)

typedef union {
    GnssStatusMessage status_msg;
} GnssDecodedMessage;

// frame buffer
static char data[1000];                                              // 数据缓冲区，用于存储接收到的字符
static size_t data_pos = 0;                                          // 数据缓冲区索引
static uint8_t frame_checksum = 0;                                   // 接收的帧校验和
static uint8_t checksum_buffer[2];                                   // 存储校验和字符
static size_t checksum_data_pos = 0;                                 // 校验和字符索引
static bool checksum_flag = false;                                   // 表示是否正在接收校验和字符
static uint8_t internal_checksum = 0;                                // 计算得到的校验和


static bool ParseChar(uint8_t c, GnssDecodedMessage *msg);
static uint8_t Hex2Ascii(uint8_t ch);
static uint8_t CalcGPGGAChecksum(char str[], uint8_t len);
static bool ConstructStatusMessage(GnssStatusMessage *msg);

bool DecodeGnssStatusMsgFromUART(uint8_t c, GnssStatusMessage *msg)
{
//    每次接收到一个字符 c，调用 ParseChar 解析。
//    如果 ParseChar 返回 true，说明接收到完整的 GNSS 数据帧，将解析出的消息存入 msg。
    static GnssDecodedMessage decoded_msg;

    bool result = ParseChar(c, &decoded_msg);
    if (result)
        *msg = decoded_msg.status_msg;
    return result;
}

uint8_t CalcUARTChecksum(uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;

    for (int i = 0; i < len; ++i)
        checksum += buf[i];

    return checksum;
}

uint8_t Hex2Ascii(uint8_t ch)
{
//    将 ASCII 表示的十六进制字符（如 'A'）转换为对应的数值（如 10）
    if(ch>='0' && ch <='9') ch -= '0'; // 数字字符转换为数值
    else if (ch>='A' && ch <='F') ch = ch - 'A' + 0x0A; // 字母字符转换为数值
    return ch;
}

uint8_t CalcGPGGAChecksum(char str[], uint8_t len)
{
//    根据 NMEA 数据格式，计算帧的校验和。
//    从帧头的第二个字符开始（不包含 $ 和校验和部分），逐字符进行按位异或操作。
     uint8_t x = (uint8_t)str[1]; // 从帧的第二个字符开始计算校验和
     uint8_t y;
     for(size_t i=2;i<len;i++)
     {
        y=(uint8_t)str[i];
        x=x^y; // 按位异或
     }

     return x;
}


bool ParseChar(uint8_t c, GnssDecodedMessage *msg)
{
//    这是核心解析逻辑，通过状态机解析接收到的 GNSS 数据流。
    static GnssSerialDecodeState decode_state = WAIT_FOR_HEAD;

    bool new_frame_parsed = false;
    switch (decode_state)
    {
//        WAIT_FOR_HEAD:
//            解析器处于 "等待帧头" 状态，帧头以字符 $ 开头。
//            如果接收到字符 $：
//            初始化缓冲区和相关状态变量。
//            将 $ 存入缓冲区，并切换到 WAIT_FOR_DATA 状态。
    case WAIT_FOR_HEAD:
    {
        if (c == '$')
        {
            data_pos = 0;
            frame_checksum = 0;
            checksum_data_pos = 0;
            checksum_flag = false;
            internal_checksum = 0;
            memset(checksum_buffer, 0, 2);

            data[data_pos++] = c;
            //printf("%c",data[data_pos-1]);
            decode_state = WAIT_FOR_DATA;
        }
        break;
    }
//        WAIT_FOR_DATA:
//            解析器开始接收帧数据，逐字符存储到缓冲区 data 中。
//            如果接收到 *，说明接下来是校验和：
//                 1.将 checksum_flag 置为 true。
//                 2.开始接收校验和字符。
//            如果校验和字符接收完成（checksum_data_pos == 2）：
//                 1.计算接收到的校验和 frame_checksum。
//                 2.计算数据的实际校验和 internal_checksum。
//                 3.比较两者是否相等，若相等则标记 new_frame_parsed = true。
//            如果接收到完整数据帧，将状态切换回 WAIT_FOR_HEAD。
    case WAIT_FOR_DATA:
    {
        data[data_pos++] = c;
        //printf("%c",data[data_pos-1]);
        if(checksum_flag == true)
        {
            checksum_buffer[checksum_data_pos++] = c;
            if(checksum_data_pos == 2)
            {
                data[data_pos]='\0';
                //printf("%s\n",data);
                checksum_buffer[0] = Hex2Ascii(checksum_buffer[0]);
                checksum_buffer[1] = Hex2Ascii(checksum_buffer[1]);
                frame_checksum = (checksum_buffer[0] << 4 | checksum_buffer[1]);
                // 调用 CalcGPGGAChecksum 计算帧的实际校验和，校验和计算基于 XOR 操作
                internal_checksum = CalcGPGGAChecksum(data, data_pos-3);
                //printf("check1=%.2X,check2=%.2X\n\n",frame_checksum,internal_checksum);
                new_frame_parsed = true;
                decode_state = WAIT_FOR_HEAD;
                break;
            }
        }

        if (c == '*')
            checksum_flag = true;
        break;
    }
    }

    if (new_frame_parsed)
    {
        // 如果校验通过，调用 ConstructStatusMessage 将数据缓冲区中的内容解析为具体的 GNSS 消息。
        if (frame_checksum == internal_checksum)
//        if(true)
        {
            ConstructStatusMessage(&(msg->status_msg));

        }
    }

    return new_frame_parsed;
}

bool ConstructStatusMessage(GnssStatusMessage *msg)
{
    if (msg == NULL)
        return false;
    // 检查缓冲区 data 中的帧类型，根据帧标识符（如 GGA、RMC 等），将数据存入对应的 GnssStatusMessage 字段
    // $GPGGA, $GPRMC, $GTIMU,$GPFPD
    if (data[3] == 'G' && data[4] == 'G' && data[5] == 'A')
    {
        strcpy(msg->gpgga_data,data); // $GPGGA: 全球定位系统固定数据
    }
    else if (data[3] == 'R' && data[4] == 'M' && data[5] == 'C')
    {
        strcpy(msg->gprmc_data,data); // $GPRMC: 推荐最小导航信息
    }
    else if (data[3] == 'I' && data[4] == 'M' && data[5] == 'U')
    {
        strcpy(msg->gtimu_data,data); // $GTIMU: 惯性测量单元数据
    }
    else if (data[3] == 'F' && data[4] == 'P' && data[5] == 'D')
    {
        strcpy(msg->gpfpd_data,data); // $GPFPD: 自定义 GNSS 数据帧
    }    

    return true;
}
