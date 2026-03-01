#include "usb_hid_sinput.h"
#include <sample_usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(usb_hid_sinput, LOG_LEVEL_INF);

static K_SEM_DEFINE(ep_write_sem, 0, 1);
static const struct device *hid_device = NULL;

// IMU timestamp counter
static uint32_t imu_timestamp_us = 0;

// Haptics callback storage
static void (*haptics_callback)(uint8_t left_amp, uint8_t right_amp) = NULL;

// Feature query response flag
static bool feature_query_pending = false;

// Feature response packet (Report ID 0x02, 63 bytes)
static uint8_t feature_response[63] = {
    0x02,       // Command ID
    0x00, 0x00, // Protocol Version
    0xFF,       // Feature Flags 1
    0xFF,       // Feature Flags 2
    0x05,       // Gamepad Physical Type (PS4 style)
    0x1F,       // Face Style / Sub-product
    0b10100000, 0b00001111, // Polling rate (paste right side, then left side)
    0x00, 0x10, // Accelerometer range
    0b11110100, 0b00000001, // Gyroscope range
    0xFF, 0xFF, 0xFF, 0x00, // Button Usage Mask (24 buttons: face, dpad, shoulders, triggers, sticks, paddles, touchpads, guide, start/select, capture)
    0x01,       // Touchpad count (1 touchpad - split in half by application)
    0x02,       // Touchpad finger count (2 fingers)
    0x00, 0x00, 0x00, 0x69, 0x04, 0x20, // MAC address
    // Remaining 37 bytes reserved/padding
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// S-Input HID Report Descriptor with proper touchpad support
static const uint8_t sinput_hid_report_descriptor[] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,                    // Usage (Gamepad)
    0xA1, 0x01,                    // Collection (Application)
    
    // INPUT REPORT ID 0x01 - Main gamepad data
    0x85, 0x01,                    //   Report ID (1)
    
    // Padding bytes (bytes 1-2) - Plug status and Charge Percent
    0x06, 0x00, 0xFF,              //   Usage Page (Vendor Defined)
    0x09, 0x01,                    //   Usage (Vendor Usage 1)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x25, 0xFF,                    //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8)
    0x95, 0x02,                    //   Report Count (2)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // --- 24 buttons (reduced from 32 to match actual usage) ---
    0x05, 0x09,                    // Usage Page (Button)
    0x19, 0x01,                    //   Usage Minimum (Button 1)
    0x29, 0x18,                    //   Usage Maximum (Button 24)
    0x15, 0x00,                    //   Logical Min (0)
    0x25, 0x01,                    //   Logical Max (1)
    0x75, 0x01,                    //   Report Size (1)
    0x95, 0x18,                    //   Report Count (24)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // Analog Sticks and Triggers
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    // Left Stick X, Y (centered)
    0x09, 0x30,                    //   Usage (X)
    0x09, 0x31,                    //   Usage (Y)
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x02,                    //   Report Count (2)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // Right Stick X, Y (centered)
    0x09, 0x32,                    //   Usage (Z)
    0x09, 0x35,                    //   Usage (Rz)
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x02,                    //   Report Count (2)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // Triggers (centered at 0: -32768 to 32767)
    0x09, 0x34,                    //   Usage (Ry) - Left trigger
    0x09, 0x33,                    //   Usage (Rx) - Right trigger
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x02,                    //   Report Count (2)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // IMU Timestamp (vendor-defined, 4 bytes)
    0x06, 0x00, 0xFF,              // Usage Page (Vendor Defined)
    0x09, 0x20,                    //   Usage (Vendor Usage 0x20)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0xFF, 0xFF,  //   Logical Maximum (4294967295)
    0x75, 0x20,                    //   Report Size (32)
    0x95, 0x01,                    //   Report Count (1)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // Motion data and Reserved data - 44 bytes
    // This includes gyro/accel data and touchpad that apps can use if supported
    0x06, 0x00, 0xFF,              // Usage Page (Vendor Defined)
    
    // Motion Input Accelerometer XYZ (Gs) and Gyroscope XYZ (Degrees Per Second)
    0x09, 0x21,                    //   Usage (Vendor Usage 0x21)
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x06,                    //   Report Count (6)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // Reserved padding (29 bytes includes touchpad data)
    0x09, 0x22,                    //   Usage (Vendor Usage 0x22)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8)
    0x95, 0x1D,                    //   Report Count (29)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // INPUT REPORT ID 0x02 - Vendor COMMAND data
    0x85, 0x02,                    //   Report ID (2)
    0x09, 0x23,                    //   Usage (Vendor Usage 0x23)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8 bits)
    0x95, 0x3F,                    //   Report Count (63)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // OUTPUT REPORT ID 0x03 - Vendor COMMAND data (haptics)
    0x85, 0x03,                    //   Report ID (3)
    0x09, 0x24,                    //   Usage (Vendor Usage 0x24)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8 bits)
    0x95, 0x2F,                    //   Report Count (47)
    0x91, 0x02,                    //   Output (Data,Var,Abs)

    0xC0                           // End Collection 
};

static int enable_usb_device_next(void)
{
    struct usbd_context *sample_usbd;
    int err;

    sample_usbd = sample_usbd_init_device(NULL);
    if (sample_usbd == NULL)
    {
        LOG_ERR("Failed to initialize USB device");
        return -ENODEV;
    }

    // Set VID/PID for S-Input
    // Using Raspberry Pi Foundation VID with S-Input generic PID
    usbd_device_set_vid(sample_usbd, 0x2E8A); // Raspberry Pi Foundation
    usbd_device_set_pid(sample_usbd, 0x10C6); // S-Input Generic Device

    err = usbd_enable(sample_usbd);
    if (err)
    {
        LOG_ERR("Failed to enable device support");
        return err;
    }

    LOG_INF("USB device support enabled (S-Input protocol)");
    return 0;
}

static void int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
    k_sem_give(&ep_write_sem);
}

// Feature/Output report callback for HID Get Report requests
static int get_report_cb(const struct device *dev, uint8_t type, uint8_t id, uint16_t len, uint8_t *buf)
{
    ARG_UNUSED(dev);
    
    LOG_DBG("GET REPORT: type=%u, id=0x%02x, len=%u", type, id, len);
    
    // S-Input doesn't require complex feature reports like DS4
    // Most configuration is done via the HID descriptor itself
    
    return -ENOTSUP;
}

// Output report callback for HID Set Report requests (haptics, LEDs, feature query)
static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, const uint8_t *const buf)
{
    ARG_UNUSED(dev);

    LOG_INF("SET REPORT: type=%u, id=0x%02x, len=%u", type, id, len);
    
    // Log first few bytes for debugging
    if (buf && len > 0) {
        LOG_INF("  Data: %02x %02x %02x %02x", 
                buf[0], 
                len > 1 ? buf[1] : 0,
                len > 2 ? buf[2] : 0,
                len > 3 ? buf[3] : 0);
    }

    // Handle output reports (type 2) - can be Report ID 0x00 or 0x03
    if (type == 2)
    {
        if (buf && len >= 1)
        {
            // Some hosts include report ID in buf[0], others don't.
            // Normalize so payload[0] is always command_id.
            const uint8_t *payload = buf;
            uint16_t payload_len = len;
            if (id != 0 && buf[0] == id && len > 1) {
                payload = &buf[1];
                payload_len = len - 1;
            }
 
            if (payload_len < 1) {
                return 0;
            }

            // payload[0] = command ID (0x01, 0x02, 0x03, 0x04, etc.)
            uint8_t command_id = payload[0];
            
            LOG_INF("S-Input command received: 0x%02x (report_id=0x%02x)", command_id, id);
            
            switch (command_id)
            {
                case 0x02: // Feature Query - Steam/SDL uses this to discover touchpad
                {
                    LOG_INF("Feature query received - will send feature response");
                    feature_query_pending = true;
                    return 0; // Acknowledge immediately
                }                
                case 0x01: // Haptics command
                {
                    if (payload_len >= 2)
                    {
                        uint8_t haptic_type = payload[1];
                        LOG_INF("RUMBLE CMD: len=%u type=0x%02x b0=%02x b1=%02x b2=%02x b3=%02x b4=%02x b5=%02x",
                                len,
                                haptic_type,
                                buf[0],
                                len > 1 ? buf[1] : 0,
                                len > 2 ? buf[2] : 0,
                                len > 3 ? buf[3] : 0,
                                len > 4 ? buf[4] : 0,
                                len > 5 ? buf[5] : 0);
                        
                        if (haptic_type == 0x02 && payload_len >= 6)
                        {
                            // Type 2 - ERM Stereo Haptics (simple)
                            uint8_t left_amplitude = payload[2];
                            // bool left_brake = payload[3]; // Could be used for more advanced control
                            uint8_t right_amplitude = payload[4];
                            // bool right_brake = payload[5];
                            
                            LOG_DBG("Haptics Type 2: L=%u, R=%u", left_amplitude, right_amplitude);
                            LOG_INF("RUMBLE TYPE2: L=%u R=%u", left_amplitude, right_amplitude);
                            
                            // Call haptics callback if registered
                            if (haptics_callback)
                            {
                                haptics_callback(left_amplitude, right_amplitude);
                            }
                        }
                        else if (haptic_type == 0x01 && payload_len >= 18)
                        {
                            // Type 1 - Precise Stereo Haptics (frequency/amplitude pairs)
                            // This is more complex - could convert to simple amplitude
                            uint16_t left_amp1 = ((uint16_t)payload[4] << 8) | payload[3];
                            uint16_t right_amp1 = ((uint16_t)payload[12] << 8) | payload[11];
                            
                            // Convert 16-bit to 8-bit amplitude
                            uint8_t left_amp = (uint8_t)(left_amp1 >> 8);
                            uint8_t right_amp = (uint8_t)(right_amp1 >> 8);
                            
                            LOG_DBG("Haptics Type 1: L=%u, R=%u", left_amp, right_amp);
                            LOG_INF("RUMBLE TYPE1: L=%u R=%u", left_amp, right_amp);
                            
                            if (haptics_callback)
                            {
                                haptics_callback(left_amp, right_amp);
                            }
                        }
                        else
                        {
                            LOG_WRN("RUMBLE CMD: unhandled format type=0x%02x len=%u", haptic_type, len);
                        }
                    }
                    break;
                }
                
                case 0x03: // Player LEDs
                {
                    if (payload_len >= 2)
                    {
                        uint8_t player_num = payload[1];
                        LOG_DBG("Player LED: %u", player_num);
                        // TODO: Could implement player LED control
                    }
                    break;
                }
                
                case 0x04: // Joystick RGB
                {
                    if (payload_len >= 4)
                    {
                        uint8_t red = payload[1];
                        uint8_t green = payload[2];
                        uint8_t blue = payload[3];
                        LOG_DBG("Joystick RGB: R=%u, G=%u, B=%u", red, green, blue);
                        // TODO: Could implement RGB LED control
                    }
                    break;
                }
                
                default:
                    LOG_DBG("Unknown S-Input command: 0x%02x", command_id);
                    break;
            }
        }
        
        // Acknowledge the output request
        return 0;
    }

    return -ENOTSUP;
}

static const struct hid_device_ops ops = {
    .input_report_done = int_in_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

// Initialize S-Input USB HID device
int usb_hid_sinput_init(void)
{
    int ret;

    hid_device = DEVICE_DT_GET_ONE(zephyr_hid_device);

    if (hid_device == NULL)
    {
        LOG_ERR("Cannot get USB HID Device");
        return -ENODEV;
    }

    hid_device_register(hid_device,
                        sinput_hid_report_descriptor, 
                        sizeof(sinput_hid_report_descriptor),
                        &ops);

    ret = enable_usb_device_next();

    if (ret != 0)
    {
        LOG_ERR("Failed to enable USB");
        return ret;
    }

    LOG_INF("S-Input USB HID device initialized");
    return 0;
}

// Get the HID device handle
const struct device *usb_hid_sinput_get_device(void)
{
    return hid_device;
}

// Send S-Input gamepad report
int usb_hid_sinput_send_report(const struct device *hid_dev, const sinput_input_report_t *report)
{
    if (!hid_dev || !report)
    {
        return -EINVAL;
    }

    // Check if feature query is pending - send feature response first
    if (feature_query_pending)
    {
        LOG_INF("Sending feature response (Report ID 0x02)");
        feature_query_pending = false;
        
        // Send feature response as Report ID 0x02
        uint8_t feature_report[64] = {SINPUT_COMMAND_REPORT_ID}; // Report ID 0x02
        memcpy(&feature_report[1], feature_response, 63);
        
        int ret = hid_int_ep_write(hid_dev, feature_report, 64, NULL);
        if (ret == 0)
        {
            k_sem_take(&ep_write_sem, K_FOREVER);
        }
        else
        {
            LOG_ERR("Feature response write failed: %d", ret);
        }
    }

    // Update IMU timestamp (increment by ~4ms worth of microseconds for 250Hz polling)
    imu_timestamp_us += 4000;
    
    // Create mutable copy to set timestamp
    sinput_input_report_t report_copy = *report;
    report_copy.imu_timestamp_us = imu_timestamp_us;

    int ret = hid_int_ep_write(hid_dev, (uint8_t *)&report_copy, sizeof(sinput_input_report_t), NULL);
    if (ret == 0)
    {
        k_sem_take(&ep_write_sem, K_FOREVER);
    }
    else
    {
        LOG_ERR("HID write failed: %d", ret);
    }

    return ret;
}

// Register haptics callback (called when host sends haptics commands)
void usb_hid_sinput_register_haptics_callback(void (*callback)(uint8_t left_amp, uint8_t right_amp))
{
    haptics_callback = callback;
}
