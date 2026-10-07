#include "UsbSerial.h"
#include "Timebase.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"

UsbSerial Serial;

namespace {
USBD_HandleTypeDef device;
uint8_t rxPacket[CDC_DATA_FS_MAX_PACKET_SIZE];
uint8_t txChunk[CDC_DATA_FS_MAX_PACKET_SIZE * 4];
uint8_t lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08}; // 115200 8N1, echoed to the host

// Masks only the USB interrupt while the main loop touches shared state.
struct UsbIrqGuard {
    bool enabled;
    UsbIrqGuard() : enabled(NVIC_GetEnableIRQ(OTG_FS_IRQn)) { NVIC_DisableIRQ(OTG_FS_IRQn); __DSB(); __ISB(); }
    ~UsbIrqGuard() { if (enabled) NVIC_EnableIRQ(OTG_FS_IRQn); }
};

int8_t cdcInit()
{
    USBD_CDC_SetRxBuffer(&device, rxPacket);
    return USBD_OK;
}
int8_t cdcDeInit() { Serial.onLineState(false); return USBD_OK; }
int8_t cdcControl(uint8_t cmd, uint8_t* buf, uint16_t length)
{
    switch (cmd) {
    case CDC_SET_LINE_CODING: if (length >= sizeof(lineCoding)) memcpy(lineCoding, buf, sizeof(lineCoding)); break;
    case CDC_GET_LINE_CODING: memcpy(buf, lineCoding, sizeof(lineCoding)); break;
    case CDC_SET_CONTROL_LINE_STATE: {
        const USBD_SetupReqTypedef* req = (const USBD_SetupReqTypedef*)buf;
        Serial.onLineState(req->wValue & 0x0001U);
        break;
    }
    default: break;
    }
    return USBD_OK;
}
int8_t cdcReceive(uint8_t* buf, uint32_t* length) { Serial.onReceive(buf, *length); return USBD_OK; }
int8_t cdcTransmitComplete(uint8_t* buf, uint32_t* length, uint8_t ep)
{
    (void)buf; (void)length; (void)ep;
    Serial.onTransmitComplete();
    return USBD_OK;
}
USBD_CDC_ItfTypeDef cdcInterface = {cdcInit, cdcDeInit, cdcControl, cdcReceive, cdcTransmitComplete};
}

void UsbSerial::begin(uint32_t)
{
    static bool started = false;
    if (started) return;
    started = true;
    USBD_Init(&device, &usbd_cdc_descriptors, DEVICE_FS);
    USBD_RegisterClass(&device, &USBD_CDC);
    USBD_CDC_RegisterInterface(&device, &cdcInterface);
    USBD_Start(&device);
}

bool UsbSerial::connected() const { return device.dev_state == USBD_STATE_CONFIGURED && dtr_; }

void UsbSerial::onLineState(bool dtr)
{
    dtr_ = dtr;
    if (!dtr) { tx_.clear(); txBusy_ = false; }
}

void UsbSerial::onReceive(const uint8_t* data, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i) rx_.push(data[i]);
    if (rx_.space() >= sizeof(rxPacket)) rearmReceive();
    else rxPaused_ = true;
}

void UsbSerial::rearmReceive()
{
    rxPaused_ = false;
    USBD_CDC_SetRxBuffer(&device, rxPacket);
    USBD_CDC_ReceivePacket(&device);
}

int UsbSerial::read()
{
    const int value = rx_.pop();
    if (rxPaused_ && rx_.space() >= sizeof(rxPacket)) {
        UsbIrqGuard guard;
        if (rxPaused_) rearmReceive();
    }
    return value;
}

void UsbSerial::startTransmit()
{
    // Interrupt context, or main context with the USB interrupt masked.
    if (txBusy_ || !connected()) return;
    const uint8_t* start;
    size_t count = tx_.contiguous(start);
    if (!count) return;
    if (count > sizeof(txChunk)) count = sizeof(txChunk);
    memcpy(txChunk, start, count);
    USBD_CDC_SetTxBuffer(&device, txChunk, (uint32_t)count);
    if (USBD_CDC_TransmitPacket(&device) == USBD_OK) {
        tx_.consume(count);
        txBusy_ = true;
    }
}

void UsbSerial::onTransmitComplete()
{
    txBusy_ = false;
    startTransmit();
}

size_t UsbSerial::write(const uint8_t* buffer, size_t size)
{
    if (!connected()) return size; // no terminal: discard without blocking
    size_t written = 0;
    const uint32_t start = millis();
    while (written < size) {
        while (written < size && tx_.push(buffer[written])) ++written;
        { UsbIrqGuard guard; startTransmit(); }
        if (written < size && (millis() - start >= WriteTimeoutMs || !connected())) break;
    }
    return written;
}

void UsbSerial::flush()
{
    const uint32_t start = millis();
    while (connected() && (!tx_.empty() || txBusy_) && millis() - start < 100U) {
        UsbIrqGuard guard;
        startTransmit();
    }
}
