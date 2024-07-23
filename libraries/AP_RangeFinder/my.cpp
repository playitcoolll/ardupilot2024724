#include "my.h"


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_InternalError/AP_InternalError.h>

#include <AP_HAL/AP_HAL.h>
#include <cctype>
#include <stdio.h>

uint8_t reset_flag(void);
float hexToFloat(uint32_t hexValue);

extern const AP_HAL::HAL &hal;

uint8_t  packstart1 = 0xEB;//包头
uint8_t  packstart2 = 0x90;
uint8_t  flag = 0;//接收完成标志
uint8_t  index_aoa = 0;//接收字节索引
uint8_t  index_ssa = 0;
uint8_t  index_buf1 = 0;
uint8_t aoa_buf[4];
uint8_t ssa_buf[4];
uint8_t buf1[24];
float floatValue_aoa = 0;
float floatValue_ssa = 0;
uint32_t _last_update_ms = 0;


aoassa::aoassa(): _aoa(),
                  _ssa(),
                  _last_update_ms(0)
{
    
    _singleton = this;
}


static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        return;
    }
    uart->begin(115200,111,111);
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        return;
    }

    // uart->discard_input();
    // hal.scheduler->delay(20);

    static uint8_t state = 0;
    static uint8_t sum = 0;

    
    int16_t nbytes = uart ->available();
    while (nbytes--) { //??????????????

       
        uint8_t c;
        ssize_t bytesRead = uart->read(c);
        
        if(bytesRead == 1){
            reset_flag();
            switch(state){
                case 0:
                    if(c == packstart1){
                        sum += c;
                        state = 1;
                    }else{
                        sum = 0;
                        state = 0;
                    }
                    break;
                case 1:
                    if(c == packstart2){
                        sum += c;
                        state = 2;
                    }else{
                        sum = 0;
                        state = 0;
                    }
                    break;
                case 2:
                    if(c == 0x25){
                        sum += c;
                        state = 3;
                    }else{
                        sum = 0;
                        state = 0;
                    }
                    break;
                case 3:
                    sum += c;
                    state = 4;
                    index_aoa = 0;
                    index_ssa = 0;
                    index_buf1 = 0;
                    break;
                case 4:
                    sum += c;
                    aoa_buf[index_aoa] = c;
                    index_aoa++;
                    if(index_aoa >= 4){
                        state = 5;
                    }
                    break;
                case 5:
                    sum += c;
                    ssa_buf[index_ssa] = c;
                    index_ssa++;
                    if(index_ssa >= 4){
                        state = 6;
                    }
                    break;
                case 6:
                    sum += c;
                    buf1[index_buf1] = c;
                    index_buf1++;
                    if(index_buf1 >= 24){
                        state = 7;
                    }
                    break;
                case 7:
                    if(sum % 256 != c){
                        state = 0;
                        sum = 0;
                    }else{
                        flag = 1;
                    }
            }

          
        }

        if(flag == 1){
            uint32_t combined_aoa = (aoa_buf[0] << 24) | (aoa_buf[1] << 16) | (aoa_buf[2] << 8) | aoa_buf[3];
            uint32_t combined_ssa = (ssa_buf[0] << 24) | (ssa_buf[1] << 16) | (ssa_buf[2] << 8) | ssa_buf[3];
            floatValue_aoa = hexToFloat(combined_aoa);
            hal.console->printf("Number of bytes available: %f\n", floatValue_aoa);//整数，没收到数据那就是0
            floatValue_ssa = hexToFloat(combined_ssa);
            break;  
        }
    }
    //  uart->discard_input();


}



void aoassa::init()
{
    hal.scheduler->delay(100);
    setup_uart(hal.serial(4), "SERIAL4");  
}



void aoassa::update(void)
{
    // const uint32_t now = AP_HAL::millis();
    // if (now - _last_update_ms < 10) {
    //     // don't update at more than 50Hz
    //     return;
    // }
    // _last_update_ms = now;
    test_uart(hal.serial(4), "SERIAL4");
    _aoa = floatValue_aoa;
    _ssa = floatValue_ssa;

}



uint8_t reset_flag(void){
    if(flag == 1){
        flag = 0;
        
    }
    return flag;
}

float hexToFloat(uint32_t hexValue) {
    float floatValue;
    memcpy(&floatValue, &hexValue, sizeof(hexValue));
    return floatValue;
}

aoassa *aoassa::_singleton;


namespace AP {
    aoassa *Aoassa()
    {
        return aoassa::get_singleton();
    }

}



// int16_t nbytes = uart->available();
// if (nbytes >= 111) {
//     uint8_t buffer[111]; // 用于存储读取的111个字节
//     ssize_t bytesRead = uart->read(buffer, 111);

//     if (bytesRead == 111) {
//         uint8_t sum = 0;
//         uint8_t state = 0;
//         uint8_t aoa_buf[4], ssa_buf[4], buf1[24];
//         uint8_t index_aoa = 0, index_ssa = 0, index_buf1 = 0;
//         bool flag = false;

//         for (int i = 0; i < 111; i++) {
//             uint8_t c = buffer[i];
//             switch (state) {
//                 case 0:
//                     if (c == packstart1) {
//                         sum += c;
//                         state = 1;
//                     } else {
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 1:
//                     if (c == packstart2) {
//                         sum += c;
//                         state = 2;
//                     } else {
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 2:
//                     if (c == 0x25) {
//                         sum += c;
//                         state = 3;
//                     } else {
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 3:
//                     sum += c;
//                     state = 4;
//                     index_aoa = 0;
//                     index_ssa = 0;
//                     index_buf1 = 0;
//                     break;
//                 case 4:
//                     sum += c;
//                     aoa_buf[index_aoa++] = c;
//                     if (index_aoa >= 4) {
//                         state = 5;
//                     }
//                     break;
//                 case 5:
//                     sum += c;
//                     ssa_buf[index_ssa++] = c;
//                     if (index_ssa >= 4) {
//                         state = 6;
//                     }
//                     break;
//                 case 6:
//                     sum += c;
//                     buf1[index_buf1++] = c;
//                     if (index_buf1 >= 24) {
//                         state = 7;
//                     }
//                     break;
//                 case 7:
//                     if (sum % 256 != c) {
//                         state = 0;
//                         sum = 0;
//                     } else {
//                         flag = true;
//                     }
//                     break;
//                 default:
//                     state = 0;
//                     sum = 0;
//                     break;
//             }

//             if (flag) {
//                 uint32_t combined_aoa = (aoa_buf[0] << 24) | (aoa_buf[1] << 16) | (aoa_buf[2] << 8) | aoa_buf[3];
//                 uint32_t combined_ssa = (ssa_buf[0] << 24) | (ssa_buf[1] << 16) | (ssa_buf[2] << 8) | ssa_buf[3];
//                 float floatValue_aoa = hexToFloat(combined_aoa);
//                 hal.console->printf("AOA: %f\n", floatValue_aoa);
//                 float floatValue_ssa = hexToFloat(combined_ssa);
//                 hal.console->printf("SSA: %f\n", floatValue_ssa);
//                 break;
//             }
//         }
//     } else {
//         // 如果读取的字节数不正确，则继续读取剩余的字节
//         // 这里可以选择记录错误，或者进行相应处理
//     }
// }