#ifndef FONT_CHINESE_V_16X16_H
#define FONT_CHINESE_V_16X16_H

#ifdef __cplusplus
extern "C" {
#endif
   
/** @brief   Type of the font-talbe.
 */
typedef struct {
    unsigned char Char16x16[2];
    unsigned char code[32];
} fontChineseV16x16_t;

/** @brief   FONT Table(Chinese characters): 16X16
 *      Font:   Fixedsys12, 
 *	Format: portrait, Bytes in reverse order
 */
#define FONTTABLE_CHINESE_V_16x16_NUM  (sizeof(FontTable_Chinese_V_16X16)/sizeof(FontTable_Chinese_V_16X16[0]))
static const fontChineseV16x16_t FontTable_Chinese_V_16X16[]=	  
{
    "��",
    0x80,0x80,0x40,0x20,0x10,0x08,0x24,0xC3,0x04,0x08,0x10,0x20,0x40,0x80,0x80,0x00,
    0x00,0x00,0x00,0x02,0x02,0x02,0x02,0x82,0x42,0x22,0x1A,0x06,0x00,0x00,0x00,0x00,

    "��",
    0x40,0x40,0x42,0x42,0x42,0x42,0x42,0xFE,0x42,0x42,0x42,0x42,0x42,0x40,0x40,0x00,
    0x80,0x80,0x40,0x20,0x10,0x0C,0x03,0x00,0x03,0x0C,0x10,0x20,0x40,0x80,0x80,0x00,

    "��",
    0x20,0x10,0x4C,0x47,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0xD4,0x04,0x04,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x30,0x40,0xF0,0x00,
      
    "��",
    0x10,0x60,0x02,0x8C,0x00,0x00,0xFE,0x92,0x92,0x92,0x92,0x92,0xFE,0x00,0x00,0x00,
    0x04,0x04,0x7E,0x01,0x40,0x7E,0x42,0x42,0x7E,0x42,0x7E,0x42,0x42,0x7E,0x40,0x00,

    "��",
    0x00,0x00,0xFC,0x24,0x24,0x24,0xFC,0x25,0x26,0x24,0xFC,0x24,0x24,0x24,0x04,0x00,
    0x40,0x30,0x8F,0x80,0x84,0x4C,0x55,0x25,0x25,0x25,0x55,0x4C,0x80,0x80,0x80,0x00,

    "��",
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    "��",
    0x06,0x09,0x09,0xE6,0xF8,0x0C,0x04,0x02,0x02,0x02,0x02,0x02,0x04,0x1E,0x00,0x00,
    0x00,0x00,0x00,0x07,0x1F,0x30,0x20,0x40,0x40,0x40,0x40,0x40,0x20,0x10,0x00,0x00,

    "ʪ",
    0x10,0x60,0x02,0x8C,0x00,0xFE,0x92,0x92,0x92,0x92,0x92,0x92,0xFE,0x00,0x00,0x00,
    0x04,0x04,0x7E,0x01,0x44,0x48,0x50,0x7F,0x40,0x40,0x7F,0x50,0x48,0x44,0x40,0x00,
};


#ifdef __cplusplus
}
#endif

#endif /* #ifndef FONT_CHINESE_V_16X16_H */