/***********************************************************************************************************************
 * File Name    : application_utils.h
 * Description  : Contains data structures and functions used in application_utils.c.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2024 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
#ifndef __APPLICATION_UTILS_H__
#define __APPLICATION_UTILS_H__
#include "hal_data.h"
#include "hdmi_cec_utils.h"

#define DEVICE_KIT_NAME "RA4E2 MCU, EK-RA4E2 "
#define POWER_STATUS_LED_PIN   BSP_IO_PORT_02_PIN_07 /* LED1 (Blue) */
#define VOLUME_STATUS_LED_PIN  BSP_IO_PORT_01_PIN_04 /* LED2 (Green) */
#define ERROR_INDICATE_LED_PIN BSP_IO_PORT_01_PIN_12 /* LED3 (Red) */

#define ERROR_INDICATE_LED_ON  R_IOPORT_PinWrite(&g_ioport_ctrl, ERROR_INDICATE_LED_PIN, BSP_IO_LEVEL_HIGH)
#define ERROR_INDICATE_LED_OFF R_IOPORT_PinWrite(&g_ioport_ctrl, ERROR_INDICATE_LED_PIN, BSP_IO_LEVEL_LOW)

#define BANNER_INFO "\r\n\r\n\r\n******************************************************************"\
                            "\r\n*   Renesas FSP Sample Program for HDMI-CEC Module               *"\
                            "\r\n*   Flexible Software Package Version: %d.%d.%d                     *"\
                            "\r\n*   Device, Kit: "DEVICE_KIT_NAME"                            *"\
                            "\r\n******************************************************************\r\n"\

#define APP_DESCRIPTION                    "This application demonstrates HDMI-CEC device control with RA Family MCU.\r\n"\
                                           "Application can send/receive a CEC message to/from CEC device.\r\n"

#define APP_LED_DESCRIPTION                "On-board LEDs can be used to indicate device status.\r\n"\
                                           " - LED1 (Blue) indicates my device power status.\r\n"\
                                           " - LED2 (Green, PWM) brightness indicates my device sound volume status.\r\n"\
                                           " - LED3 (Red) indicates error status in API call. If turn on, reset the MCU.\r\n\r\n\r\n"

#define APP_COMMAND_OPTION             "\r\n[App Menu] Select command option.\r\n"\
                                           " 0. Send control command to other CEC device\r\n"\
                                           " 1. Scan CEC bus\r\n"\
                                           " 2. Display internal CEC device status buffer data\r\n"\
                                           " 3. Enable/Disable System Audio Mode function support (Current status: %s)\r\n"\
                                           " 4. Send System Audio Mode On/Off request (Current status: %s)\r\n"

#define SYS_AUDIO_FUNC_E "Enabled"
#define SYS_AUDIO_FUNC_D "Disabled"
#define SYS_AUDIO_ON     "On"
#define SYS_AUDIO_OFF    "Off"

#define CEC_LOGICAL_DEVICE_SELECT_MENU "\r\nSelect my CEC logical address\r\n"\
                                           " 1. TV\r\n"\
                                           " 2. Recording device\r\n"\
                                           " 3. Tuner\r\n"\
                                           " 4. Playback device\r\n"\
                                           " 5. Audio system\r\n"

#define CEC_CONTROL_DEVICE_SELECT_MENU "\r\nSelect CEC device logical address to be controlled:\r\n"\
                                           " 0. TV\r\n"\
                                           " 1. Recording Device 1\r\n"\
                                           " 2. Recording Device 2\r\n"\
                                           " 3. Tuner 1\r\n"\
                                           " 4. Playback Device 1\r\n"\
                                           " 5. Audio System\r\n"\
                                           " 6. Tuner 2\r\n"\
                                           " 7. Tuner 3\r\n"\
                                           " 8. Playback Device 2\r\n"\
                                           " 9. Recording Device 3\r\n"\
                                           " a. Tuner 4\r\n"\
                                           " b. Playback Device 3\r\n"\
                                           " f. Broadcast\r\n"\

#define CEC_CONTROL_SELECT_MENU        "\r\nSelect CEC device control option:\r\n"\
                                           " a. Power On\r\n"\
                                           " b. Power Off\r\n"\
                                           " c. Volume Up\r\n"\
                                           " d. Volume Down\r\n"\
                                           " e. Mute\r\n"

#define USER_ACTION_NONE                               (0U)
#define USER_ACTION_BUS_SCAN                           (1U)
#define USER_ACTION_DISPLAY_CEC_BUS_STATUS_BUFF        (2U)
#define USER_ACTION_ENABLING_SYSTEM_AUDIO_MODE_SUPPORT (3U)
#define USER_ACTION_SYSTEM_AUDIO_MODE_REQUEST          (4U)
#define USER_ACTION_REQUEST_POWER_ON                   ('a')
#define USER_ACTION_REQUEST_POWER_OFF                  ('b')
#define USER_ACTION_REQUEST_VOLUME_UP                  ('c')
#define USER_ACTION_REQUEST_VOLUME_DONW                ('d')
#define USER_ACTION_REQUEST_VOLUME_MUTE                ('e')

#define CEC_ACTION_NONE        (0U)
#define CEC_ACTION_POWER_ON    (1U)
#define CEC_ACTION_POWER_OFF   (2U)
#define CEC_ACTION_VOLUME_UP   (3U)
#define CEC_ACTION_VOLUME_DOWN (4U)
#define CEC_ACTION_VOLUME_MUTE (5U)

typedef struct cec_device_status
{
    bool          is_device_active;
    bool          is_my_device;
    bool          is_active_source;

    bool          is_power_status_store;
    uint8_t       power_status; // 0: Standby, 1: On

    bool          is_physical_address_store;
    uint8_t       physical_address[4];

    bool          is_vendor_id_store;
    uint8_t       vendor_id[3];

    bool          is_version_store;
    cec_version_t cec_version;
}cec_device_status_t;

typedef struct cec_rx_message_buff
{
    bool       is_new_data; ///< A flag to indicate if new data is stored
    bool       is_error;

    cec_addr_t source;
    cec_addr_t destination;
    uint8_t    opcode;
    uint8_t    data_buff[CEC_DATA_BUFFER_LENGTH];
    uint8_t    byte_counter; ///< Byte counter including header code
} cec_rx_message_buff_t;

extern volatile bool system_audio_mode_support_function;
extern volatile bool system_audio_mode_status;

void user_button_irq_initialize(void);
void demo_system_initialize(void);
void demo_system_power_on(void);
void demo_system_power_off(void);
void demo_system_volume_change(bool is_mute, bool is_volume_up);
void demo_system_volume_status_get(bool *mute_status, uint8_t *volume_status);

void user_action_check(void);

void cec_device_status_display(cec_addr_t cec_addr, cec_device_status_t * p_buff);

fsp_err_t cec_message_send(cec_addr_t destination,
    uint8_t opcode, uint8_t const * data_buff, uint8_t data_buff_length);
void cec_system_audio_mode_support_enabling(void);
void vendor_id_install(uint8_t * my_vendor_id_buff);
void irq_sw1_callback(external_irq_callback_args_t * p_args);
void cec_system_audio_mode_request(void);
void cec_bus_scan(void);
void cec_bus_status_buffer_display(void);
void user_action_process(uint8_t *cec_data);
extern cec_device_status_t cec_bus_device_list[16];
extern cec_addr_t my_logical_address;

#endif /* End of __APPLICATION_UTILS_H__ */
