#include <Arduino.h>
#include <usb/usb_host.h>
#include "show_desc.hpp"
#include "usbhhelp.hpp"
#include "esp_now.h"
#include <WiFi.h>
#define log_v(...)
#include <esp_log.h>
//#define MIDIOUTTEST 1
#if MIDIOUTTEST
#include <elapsedMillis.h>
elapsedMillis MIDIOutTimer;
#endif

bool isMIDI = false;
bool isMIDIReady = false;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_info_t peerInfo;
esp_now_peer_num_t peerNum;
unsigned long long lastTime = 0;

const size_t MIDI_IN_BUFFERS = 8;
const size_t MIDI_OUT_BUFFERS = 8;
usb_transfer_t *MIDIOut = NULL;
usb_transfer_t *MIDIIn[MIDI_IN_BUFFERS] = {NULL};
#define MSG_MIDI 116
#define MSG_ANIMATION 5


enum animationEnum {
    OFF,
    FLASH,
    BLINK,
    CANDLE,
    SYNC_ASYNC_BLINK,
    SYNC_BLINK,
    SLOW_STARTUP,
    SYNC_END,
    LED_ON,
    CONCENTRIC, 
    STROBE, 
    MIDI
};
struct animation_strobe {
  uint8_t frequency;
  unsigned long long startTime;
  uint8_t duration;
  uint8_t rgb1[3];
  uint8_t rgb2[3];
  uint8_t brightness;
};

struct animation_midi {
  uint8_t note;
  uint8_t velocity;
  uint8_t octaveDistance;
};

union animation_params {
  struct animation_strobe strobe;
  struct animation_midi midi;
};


struct message_animate {
  uint8_t messageType = MSG_ANIMATION;
  animationEnum animationType;
  animation_params animationParams;
} animationMessage;


// USB MIDI Event Packet Format (always 4 bytes)
//
// Byte 0 |Byte 1 |Byte 2 |Byte 3
// -------|-------|-------|------
// CN+CIN |MIDI_0 |MIDI_1 |MIDI_2
//
// CN = Cable Number (0x0..0xf) specifies virtual MIDI jack/cable
// CIN = Code Index Number (0x0..0xf) classifies the 3 MIDI bytes.
// See Table 4-1 in the MIDI 1.0 spec at usb.org.
//

static void midi_transfer_cb(usb_transfer_t *transfer)
{
  ESP_LOGI("", "midi_transfer_cb context: %d", transfer->context);
  if (Device_Handle == transfer->device_handle) {
    int in_xfer = transfer->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK;
    if ((transfer->status == 0) && in_xfer) {
      uint8_t *const p = transfer->data_buffer;
      for (int i = 0; i < transfer->actual_num_bytes; i += 4) {
        if ((p[i] + p[i+1] + p[i+2] + p[i+3]) == 0) break;

        uint8_t status = p[i+1];
        uint8_t data1 = p[i+2];
        uint8_t data2 = p[i+3];
        if (status & 0x0F != 0) {
            
        }
        else {
            switch (status & 0xF0) {
            case 0x80: // Note Off
                ESP_LOGI("", "Note Off: Channel %d, Note %d, Velocity %d", status & 0x0F, data1, data2);
                digitalWrite(LED_BUILTIN, LOW);
                animationMessage.animationType = MIDI;
                animationMessage.animationParams.midi.note = data1;
                animationMessage.animationParams.midi.velocity = 0;
                esp_now_send(broadcastAddress, (uint8_t *) &animationMessage, sizeof(animationMessage));
                break;
            case 0x90: // Note On
                ESP_LOGI("", "Note On: Channel %d, Note %d, Velocity %d", status & 0x0F, data1, data2);
                animationMessage.animationType = MIDI;
                animationMessage.animationParams.midi.note = data1;
                animationMessage.animationParams.midi.velocity = data2;
                rgbLedWrite(LED_BUILTIN, data2*2, data2*2, data2*2);
                esp_now_send(broadcastAddress, (uint8_t *) &animationMessage, sizeof(animationMessage));
            break;
            case 0xA0: // Polyphonic Key Pressure (Aftertouch)
                ESP_LOGI("", "Polyphonic Key Pressure: Channel %d, Note %d, Pressure %d", status & 0x0F, data1, data2);
                break;
            case 0xB0: // Control Change
                ESP_LOGI("", "Control Change: Channel %d, Controller %d, Value %d", status & 0x0F, data1, data2);
                break;
            case 0xC0: // Program Change
                ESP_LOGI("", "Program Change: Channel %d, Program %d", status & 0x0F, data1);
                break;
            case 0xD0: // Channel Pressure (Aftertouch)
                ESP_LOGI("", "Channel Pressure: Channel %d, Pressure %d", status & 0x0F, data1);
                break;
            case 0xE0: // Pitch Bend Change
                ESP_LOGI("", "Pitch Bend Change: Channel %d, LSB %d, MSB %d", status & 0x0F, data1, data2);
                break;
            default:
                ESP_LOGI("", "Unknown MIDI message: %02x %02x %02x %02x", p[i], p[i+1], p[i+2], p[i+3]);
                break;
            }
        }
      }
      ESP_LOGI("", "midi_transfer_cb In: %d bytes", transfer->actual_num_bytes);
      esp_err_t err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
      }
    }
    else {
      ESP_LOGI("", "transfer->status %d", transfer->status);
    }
  }
}

void check_interface_desc_MIDI(const void *p)
{
  const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
  ESP_LOGI("", "bInterfaceClass: 0x%02x", intf->bInterfaceClass);
  // USB MIDI
  if ((intf->bInterfaceClass == USB_CLASS_AUDIO) &&
      (intf->bInterfaceSubClass == 3) &&
      (intf->bInterfaceProtocol == 0))
  {
    isMIDI = true;
    ESP_LOGI("", "Claiming a MIDI device!");
    esp_err_t err = usb_host_interface_claim(Client_Handle, Device_Handle,
        intf->bInterfaceNumber, intf->bAlternateSetting);
    if (err != ESP_OK) ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
  }
}

void prepare_endpoints(const void *p)
{
  const usb_ep_desc_t *endpoint = (const usb_ep_desc_t *)p;
  esp_err_t err;
  ESP_LOGI("", "Endpoint: 0x%02x", endpoint->bEndpointAddress);
  // must be bulk for MIDI
  if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK) {
    ESP_LOGI("", "Not bulk endpoint: 0x%02x", endpoint->bmAttributes);
    return;
  }
  if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) {
    for (int i = 0; i < MIDI_IN_BUFFERS; i++) {
      err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIIn[i]);
      if (err != ESP_OK) {
        MIDIIn[i] = NULL;
        ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
      }
      else {
        MIDIIn[i]->device_handle = Device_Handle;
        MIDIIn[i]->bEndpointAddress = endpoint->bEndpointAddress;
        MIDIIn[i]->callback = midi_transfer_cb;
        MIDIIn[i]->context = (void *)i;
        MIDIIn[i]->num_bytes = endpoint->wMaxPacketSize;
        esp_err_t err = usb_host_transfer_submit(MIDIIn[i]);
        if (err != ESP_OK) {
          ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
        }
      }
    }
  }
  else {
    err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIOut);
    if (err != ESP_OK) {
      MIDIOut = NULL;
      ESP_LOGI("", "usb_host_transfer_alloc Out fail: %x", err);
      return;
    }
    ESP_LOGI("", "Out data_buffer_size: %d", MIDIOut->data_buffer_size);
    MIDIOut->device_handle = Device_Handle;
    MIDIOut->bEndpointAddress = endpoint->bEndpointAddress;
    MIDIOut->callback = midi_transfer_cb;
    MIDIOut->context = NULL;
//    MIDIOut->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
  }
  isMIDIReady = ((MIDIOut != NULL) && (MIDIIn[0] != NULL));
}

void show_config_desc_full(const usb_config_desc_t *config_desc)
{
  ESP_LOGI("", "bLength: %d", config_desc->bLength);
  // Full decode of config desc.
  const uint8_t *p = &config_desc->val[0];
  uint8_t bLength = 0;
  for (int i = 0; i < config_desc->wTotalLength; i+=bLength, p+=bLength) {
    bLength = *p;
    if (bLength == 0) {
        ESP_LOGE("", "Invalid descriptor length: 0");
        return;
    }
    if ((i + bLength) <= config_desc->wTotalLength) {
      const uint8_t bDescriptorType = *(p + 1);
      switch (bDescriptorType) {
        case USB_B_DESCRIPTOR_TYPE_DEVICE:
          ESP_LOGI("", "USB Device Descriptor should not appear in config");
          break;
        case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
          ESP_LOGI("", "Configuration Descriptor");
          show_config_desc(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_STRING:
          ESP_LOGI("", "USB string desc TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_INTERFACE:
          ESP_LOGI("", "Interface Descriptor");
          show_interface_desc(p);
          if (!isMIDI) check_interface_desc_MIDI(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
          ESP_LOGI("", "Endpoint Descriptor");
          show_endpoint_desc(p);
          if (isMIDI && !isMIDIReady) {
            prepare_endpoints(p);
          }
          break;
        case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
          // Should not be in config?
          ESP_LOGI("", "USB device qual desc TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
          // Should not be in config?
          ESP_LOGI("", "USB Other Speed TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
          // Should not be in config?
          ESP_LOGI("", "USB Interface Power TBD");
          break;
        default:
          ESP_LOGI("", "Unknown USB Descriptor Type: 0x%x", *p);
          break;
      }
    }
    else {
      ESP_LOGI("", "USB Descriptor invalid");
      return;
    }
  }
}

void setup()
{
  WiFi.mode(WIFI_STA);
  esp_log_level_set("*", ESP_LOG_INFO);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

 if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  ESP_LOGI("", "ESP-NOW setup done");
  usbh_setup(show_config_desc_full);
  ESP_LOGI("", "USBH setup done");
}

void loop()
{

if (lastTime+1000 < millis()) {
    lastTime = millis();
    animationMessage.animationType = MIDI;
    animationMessage.animationParams.midi.note = 60;
    animationMessage.animationParams.midi.velocity = 5;
    esp_now_send(broadcastAddress, (uint8_t *) &animationMessage, sizeof(animationMessage));
}

  usbh_task();
#ifdef MIDIOUTTEST
  if (isMIDIReady && (MIDIOutTimer > 1000)) {
    ESP_LOGI("", "MIDI send 4 bytes");
    MIDIOut->num_bytes = 4;
    memcpy(MIDIOut->data_buffer, "\x09\x90\x3c\x7a", 4);
    err = usb_host_transfer_submit(MIDIOut);
    if (err != ESP_OK) {
      ESP_LOGI("", "usb_host_transfer_submit Out fail: %x", err);
    }
    MIDIOutTimer = 0;
  }
#endif
}