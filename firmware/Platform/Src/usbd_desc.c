/* USB descriptors for the virtual COM port. VID/PID are ST's VCP pair, the
 * same as the STM32duino build, so existing host tools find the port. The
 * serial number comes from the MCU's 96-bit unique ID. */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "board_identity.h"

#define VCP_VID          0x0483U
#define VCP_PID          0x5740U
#define VCP_LANGID       0x0409U
#define VCP_MANUFACTURER "Livo"

static uint8_t device_desc[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    0x12, USB_DESC_TYPE_DEVICE, 0x00, 0x02,
    0x02, 0x02, 0x00,                  /* CDC class at device level */
    USB_MAX_EP0_SIZE,
    LOBYTE(VCP_VID), HIBYTE(VCP_VID), LOBYTE(VCP_PID), HIBYTE(VCP_PID),
    0x00, 0x02,                        /* bcdDevice 2.00 */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};
static uint8_t langid_desc[USB_LEN_LANGID_STR_DESC] __attribute__((aligned(4))) = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, LOBYTE(VCP_LANGID), HIBYTE(VCP_LANGID)
};
static uint8_t string_desc[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));

static uint8_t *device(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; *len = sizeof(device_desc); return device_desc; }
static uint8_t *langid(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; *len = sizeof(langid_desc); return langid_desc; }
static uint8_t *text(const char *value, uint16_t *len) { USBD_GetString((uint8_t *)value, string_desc, len); return string_desc; }
static uint8_t *manufacturer(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; return text(VCP_MANUFACTURER, len); }
static uint8_t *product(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; return text("Livo Stainer " BOARD_NAME, len); }
static uint8_t *configuration(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; return text("CDC Config", len); }
static uint8_t *interface(USBD_SpeedTypeDef s, uint16_t *len) { (void)s; return text("CDC Interface", len); }

static uint8_t *serial(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    static const char hex[] = "0123456789ABCDEF";
    char value[25];
    const uint32_t *uid = (const uint32_t *)UID_BASE;
    for (int word = 0; word < 3; ++word)
        for (int nibble = 0; nibble < 8; ++nibble)
            value[word * 8 + nibble] = hex[(uid[2 - word] >> (28 - 4 * nibble)) & 0xFU];
    value[24] = '\0';
    return text(value, len);
}

USBD_DescriptorsTypeDef usbd_cdc_descriptors = {
    device, langid, manufacturer, product, serial, configuration, interface
};
