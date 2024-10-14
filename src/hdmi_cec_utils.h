/***********************************************************************************************************************
 * File Name    : hdmi_cec_utils.h
 * Description  : Contains data structures and functions used in hdmi_cec_utils.c.
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
#ifndef __HDMI_CEC_UTILS_H__
#define __HDMI_CEC_UTILS_H__
#include "hal_data.h"

/* List of characters of Features. Refer to CEC 3.1 and 3.2 in HDMI Specification */
typedef enum e_cec_feature_type
{
    /* End-User Features */
    CEC_FEAT_ONE_TOUCH_PLAY                   =     0x1,
    CEC_FEAT_SYSTEM_STANDBY                   =     0x2,
    CEC_FEAT_ONE_TOUCH_RECORD                 =     0x4,
    CEC_FEAT_TIMER_PROGRAMMING                =     0x8,
    CEC_FEAT_DECK_CONTROL                     =    0x10,
    CEC_FEAT_TUNER_CONTROL                    =    0x20,
    CEC_FEAT_DEVICE_MENU_CONTROL              =    0x40,
    CEC_FEAT_REMOTE_CONTROL_PASS_THROUGH      =    0x80,
    CEC_FEAT_SYSTEM_AUDIO_CONTROL             =   0x100,
    /* Supporting Features */
    CEC_FEAT_DEVICE_OSD_NAME_TRANS            =   0x200,
    CEC_FEAT_DEVICE_POWER_STATUS              =   0x400,
    CEC_FEAT_OSD_DISPLAY                      =   0x800,
    CEC_FEAT_ROUTING_CONTROL                  =  0x1000,
    CEC_FEAT_SYSTEM_INFO                      =  0x2000,
    CEC_FEAT_VENDOR_SPECIFIC                  =  0x4000,
    CEC_FEAT_AUDIO_RATE_CONTROL               =  0x8000,
    CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL     = 0x10000,
    CEC_FEAT_CAPABILITY_DISCOVERY_AND_CONTROL = 0x20000,
}cec_feature_t;

/* List of characters of Operand Description. Refer to CEC 15 in HDMI Specification */
typedef enum e_cec_opcode
{
    CEC_OPCODE_FEATURE_ABORT                  = 0x00,
    CEC_OPCODE_ABORT                          = 0xFF,
    CEC_OPCODE_ACTIVE_SOURCE                  = 0x82,
    CEC_OPCODE_IMAGE_VIEW_ON                  = 0x04,
    CEC_OPCODE_TEXT_VIEW_ON                   = 0x0D,
    CEC_OPCODE_STANDBY                        = 0x36,
    CEC_OPCODE_RECORD_OFF                     = 0x0B,
    CEC_OPCODE_RECORD_ON                      = 0x09,
    CEC_OPCODE_RECORD_STATUS                  = 0x0A,
    CEC_OPCODE_RECORD_TV_SCREEN               = 0x0F,
    CEC_OPCODE_CLEAR_ANALOG_TIMER             = 0x33,
    CEC_OPCODE_CLEAR_DIGITAL_TIMER            = 0x99,
    CEC_OPCODE_CLEAR_EXTERNAL_TIMER           = 0xA1,
    CEC_OPCODE_SET_ANALOG_TIMER               = 0x34,
    CEC_OPCODE_SET_DIGITAL_TIMER              = 0x97,
    CEC_OPCODE_SET_EXTERNAL_TIMER             = 0xA2,
    CEC_OPCODE_SET_TIMER_PROGRAM_TITLE        = 0x67,
    CEC_OPCODE_TIMER_CLEARED_STATUS           = 0x43,
    CEC_OPCODE_TIMER_STATUS                   = 0x35,
    CEC_OPCODE_DECK_CONTROL                   = 0x42,
    CEC_OPCODE_DECK_STATUS                    = 0x1B,
    CEC_OPCODE_GIVE_DECK_STATUS               = 0x1A,
    CEC_OPCODE_PLAY                           = 0x41,
    CEC_OPCODE_GIVE_TUNER_STATUS              = 0x08,
    CEC_OPCODE_SELECT_ANALOG_SERVICE          = 0x92,
    CEC_OPCODE_SELECT_DIGITAL_SERVICE         = 0x93,
    CEC_OPCODE_TUNER_DEVICE_STATUS            = 0x07,
    CEC_OPCODE_TUNER_STEP_DECREMENT           = 0x06,
    CEC_OPCODE_TUNER_STEP_INCREMENT           = 0x05,
    CEC_OPCODE_MENU_REQUEST                   = 0x8D,
    CEC_OPCODE_MENU_STATUS                    = 0x8E,
    CEC_OPCODE_USER_CONTROL_PRESSED           = 0x44,
    CEC_OPCODE_USER_CONTROL_RELEASED          = 0x45,
    CEC_OPCODE_GIVE_AUDIO_STATUS              = 0x71,
    CEC_OPCODE_GIVE_SYSTEM_AUDIO_MODE_STATUS  = 0x7D,
    CEC_OPCODE_REPORT_AUDIO_STATUS            = 0x7A,
    CEC_OPCODE_REPORT_SHORT_AUDIO_DESCRIPTOR  = 0xA3,
    CEC_OPCODE_REQUEST_SHORT_AUDIO_DESCRIPTOR = 0xA4,
    CEC_OPCODE_SET_SYSTEM_AUDIO_MODE          = 0x72,
    CEC_OPCODE_SYSTEM_AUDIO_MODE_REQUEST      = 0x70,
    CEC_OPCODE_SYSTEM_AUDIO_MODE_STATUS       = 0x7E,
    CEC_OPCODE_GIVE_OSD_NAME                  = 0x46,
    CEC_OPCODE_SET_OSD_NAME                   = 0x47,
    CEC_OPCODE_GIVE_POWER_STATUS              = 0x8F,
    CEC_OPCODE_REPORT_POWER_STATUS            = 0x90,
    CEC_OPCODE_SET_OSD_STRING                 = 0x64,
    CEC_OPCODE_INACTIVE_SOURCE                = 0x9D,
    CEC_OPCODE_REQUEST_ACTIVE_SOURCE          = 0x85,
    CEC_OPCODE_ROUTING_CHANGE                 = 0x80,
    CEC_OPCODE_ROUTING_INFORMATION            = 0x81,
    CEC_OPCODE_SET_STREAM_PATH                = 0x86,
    CEC_OPCODE_CEC_VERSION                    = 0x9E,
    CEC_OPCODE_GET_CEC_VERSION                = 0x9F,
    CEC_OPCODE_GIVE_PHYSICAL_ADDRESS          = 0x83,
    CEC_OPCODE_GET_MENU_LANGUAGE              = 0x91,
    CEC_OPCODE_REPORT_PHYSICAL_ADDRESS        = 0x84,
    CEC_OPCODE_SET_MENU_LANGUAGE              = 0x32,
    CEC_OPCODE_DEVICE_VENDOR_ID               = 0x87,
    CEC_OPCODE_GIVE_DEVICE_VENDOR_ID          = 0x8C,
    CEC_OPCODE_VENDOR_COMMAND                 = 0x89,
    CEC_OPCODE_VENDOR_COMMNAD_W_ID            = 0xA0,
    CEC_OPCODE_VENDOR_REMOTE_BUTTON_DOWN      = 0x8A,
    CEC_OPCODE_VENDOR_REMOTE_BUTTON_UP        = 0x8B,
    CEC_OPCODE_SET_AUDIO_RATE                 = 0x9A,
    CEC_OPCODE_INITIATE_ARC                   = 0xC0,
    CEC_OPCODE_REPORT_ARC_INITIATED           = 0xC1,
    CEC_OPCODE_REPORT_ARC_TERMINATED          = 0xC2,
    CEC_OPCODE_REPORT_ARC_INITIATION          = 0xC3,
    CEC_OPCODE_REPORT_ARC_TERMINATION         = 0xC4,
    CEC_OPCODE_TERMINATE_ARC                  = 0xC5,
    CEC_OPCODE_CDC_MESSAGE                    = 0xF8,

    CEC_OPCODE_UNKNOWN = 0xFFFF, // Original of this project
}cec_opcode_t;

/* List of characters of User Control Codes. Refer to CEC Table 30 in HDMI Specification */
typedef enum e_cec_user_control_code
{
    USER_CONTROL_SELECT                       = 0x00,
    USER_CONTROL_UP                           = 0x01,
    USER_CONTROL_DOWN                         = 0x02,
    USER_CONTROL_LEFT                         = 0x03,
    USER_CONTROL_RIGHT                        = 0x04,
    USER_CONTROL_RIGHT_UP                     = 0x05,
    USER_CONTROL_RIGHT_DOWN                   = 0x06,
    USER_CONTROL_LEFT_UP                      = 0x07,
    USER_CONTROL_LEFT_DOWN                    = 0x08,
    USER_CONTROL_ROOT_MENU                    = 0x09,
    USER_CONTROL_SETUP_MENU                   = 0x0A,
    USER_CONTROL_CONTENTS_MENU                = 0x0B,
    USER_CONTROL_FAVORITE_MENU                = 0x0C,
    USER_CONTROL_EXIT                         = 0x0D,
    USER_CONTROL_NUMBER_0                     = 0x20,
    USER_CONTROL_NUMBER_1                     = 0x21,
    USER_CONTROL_NUMBER_2                     = 0x22,
    USER_CONTROL_NUMBER_3                     = 0x23,
    USER_CONTROL_NUMBER_4                     = 0x24,
    USER_CONTROL_NUMBER_5                     = 0x25,
    USER_CONTROL_NUMBER_6                     = 0x26,
    USER_CONTROL_NUMBER_7                     = 0x27,
    USER_CONTROL_NUMBER_8                     = 0x28,
    USER_CONTROL_NUMBER_9                     = 0x29,
    USER_CONTROL_DOT                          = 0x2A,
    USER_CONTROL_ENTER                        = 0x2B,
    USER_CONTROL_CLEAR                        = 0x2C,
    USER_CONTROL_NEXT_FAVORITE                = 0x2F,
    USER_CONTROL_CHANNEL_UP                   = 0x30,
    USER_CONTROL_CHANNEL_DOWN                 = 0x31,
    USER_CONTROL_PREVIOUS_CHANNEL             = 0x32,
    USER_CONTROL_SOUND_SELECT                 = 0x33,
    USER_CONTROL_INPUT_SELECT                 = 0x34,
    USER_CONTROL_DISPLAY_INFO                 = 0x35,
    USER_CONTROL_HELP                         = 0x36,
    USER_CONTROL_PAGE_UP                      = 0x37,
    USER_CONTROL_PAGE_DOWN                    = 0x38,
    USER_CONTROL_POWER                        = 0x40,
    USER_CONTROL_VOLUME_UP                    = 0x41,
    USER_CONTROL_VOLUME_DOWN                  = 0x42,
    USER_CONTROL_MUTE                         = 0x43,
    USER_CONTROL_PLAY                         = 0x44,
    USER_CONTROL_STOP                         = 0x45,
    USER_CONTROL_PAUSE                        = 0x46,
    USER_CONTROL_RECORD                       = 0x47,
    USER_CONTROL_REWIND                       = 0x48,
    USER_CONTROL_FAST_FORWARD                 = 0x49,
    USER_CONTROL_EJECT                        = 0x4A,
    USER_CONTROL_FORWARD                      = 0x4B,
    USER_CONTROL_BACKWARD                     = 0x4C,
    USER_CONTROL_STOP_RECORD                  = 0x4D,
    USER_CONTROL_PAUSE_RECORD                 = 0x4E,
    USER_CONTROL_ANGLE                        = 0x50,
    USER_CONTROL_SUB_PICTURE                  = 0x51,
    USER_CONTROL_VIDEO_ON_DEMAND              = 0x52,
    USER_CONTROL_ELECTRONIC_PROGRAM_GUIDE     = 0x53,
    USER_CONTROL_TIMER_PROGRAMMING            = 0x54,
    USER_CONTROL_INITIAL_CONFIG               = 0x55,
    USER_CONTROL_PLAY_FUNCTION                = 0x60,
    USER_CONTROL_PAUSE_PLAY_FUNCTION          = 0x61,
    USER_CONTROL_RECORD_FUNCTION              = 0x62,
    USER_CONTROL_PAUSE_RECORD_FUNCTION        = 0x63,
    USER_CONTROL_STOP_FUNCTION                = 0x64,
    USER_CONTROL_MUTE_FUNCTION                = 0x65,
    USER_CONTROL_RESTORE_VOLUME_FUNCTION      = 0x66,
    USER_CONTROL_TUNE_FUNCTION                = 0x67,
    USER_CONTROL_SELECT_MEDIA_FUNCTION        = 0x68,
    USER_CONTROL_SELECT_AV_INPUT_FUNCTION     = 0x69,
    USER_CONTROL_SELECT_AUDIO_INTPUT_FUNCTION = 0x6A,
    USER_CONTROL_POWER_TOGGLE_FUNCTION        = 0x6B,
    USER_CONTROL_POWER_OFF_FUNCTION           = 0x6C,
    USER_CONTROL_POWER_ON_FUNCTION            = 0x6D,
    USER_CONTROL_F1_BLUE                      = 0x71,
    USER_CONTROL_F2_RED                       = 0x72,
    USER_CONTROL_F3_GREEN                     = 0x73,
    USER_CONTROL_F4_YELLOW                    = 0x74,
    USER_CONTROL_F5                           = 0x75,
    USER_CONTROL_DATA                         = 0x76
}cec_user_control_code_t;

/* List of characters of [CEC Version]. Refer to CEC 17 in HDMI Specification */
typedef enum cec_version
{
    CEC_VERSION_1_1  = 0x00,
    CEC_VERSION_1_2  = 0x01,
    CEC_VERSION_1_2a = 0x02,
    CEC_VERSION_1_3  = 0x03,
    CEC_VERSION_1_3a = 0x04,
    CEC_VERSION_1_4  = 0x05,
}cec_version_t;

/* List of characters of [Power Status]. Refer to CEC 17 in HDMI Specification */
typedef enum cec_power_status
{
    CEC_POWER_STATUS_ON                       = 0x00,
    CEC_POWER_STATUS_STANDBY                  = 0x01,
    CEC_POWER_STATUS_IN_TRANSITION_TO_ON      = 0x02,
    CEC_POWER_STATUS_IN_TRANSITION_TO_STANDBY = 0x03,
}cec_power_status_t;

/* List of characters of [Device Type]. Refer to CEC 17 in HDMI Specification */
typedef enum cec_device_type
{
    CEC_DEVICE_TYPE_TV               = 0x00,
    CEC_DEVICE_TYPE_RECORDING_DEVICE = 0x01,
    CEC_DEVICE_TYPE_TUNER            = 0x03,
    CEC_DEVICE_TYPE_PLAYBACK_DEVICE  = 0x04,
    CEC_DEVICE_TYPE_AUDIO_SYSTEM     = 0x05,
    CEC_DEVICE_TYPE_PURE_CEC_SWITCH  = 0x06,
    CEC_DEVICE_TYPE_UNKNOWN          = 0xFF,
}cec_device_type_t;

/* List of characters of [Abort Reason]. Refer to CEC 17 in HDMI Specification */
typedef enum cec_about_reason
{
    CEC_ABOUT_REASON_UNRECOFNIZED_OPCODE            = 0x00,
    CEC_ABOUT_REASON_NOT_IN_CORRECT_MODE_TO_RESPOND = 0x01,
    CEC_ABOUT_REASON_CANNOT_PROVIDE_SOURCE          = 0x02,
    CEC_ABOUT_REASON_INVALID_OPERAND                = 0x03,
    CEC_ABOUT_REASON_REFUSED                        = 0x04,
    CEC_ABOUT_REASON_UNABLE_TO_DETERMINE            = 0x05,
}cec_about_reason_t;

/* List of characters of [System Audio Status]. Refer to CEC 17 in HDMI Specification */
typedef enum cec_system_audio_status
{
    CEC_SYSTEM_AUDIO_STATUS_OFF = 0x00,
    CEC_SYSTEM_AUDIO_STATUS_ON  = 0x01,
}cec_system_audio_status_T;

typedef struct cec_feature_type_define
{
    cec_feature_t feature;
    uint8_t       feature_desc_str[35];
} cec_feature_define_t;

typedef struct cec_message_type
{
    cec_opcode_t opcode;
    uint8_t      opcode_desc_str[35];
    uint32_t     feature_bits;
} cec_opcode_define_t;

typedef struct cec_audio_status
{
    uint8_t audio_volume_status : 7;
    uint8_t audio_mute_status   : 1;
}cec_audio_status_t; /* Total 1 byte */

extern uint8_t cec_logical_device_list[16][23];
extern uint8_t cec_version_list[6][5];
extern uint8_t cec_power_status_list[4][28];

extern cec_feature_define_t cec_feature_list[];
extern uint32_t             cec_feature_list_number;

extern cec_opcode_define_t cec_opcode_list[];
extern uint32_t            cec_opcode_list_number;

uint32_t opcode_description_find(uint8_t opcode);
cec_device_type_t convert_logical_address_to_device_type(cec_addr_t addr);

enum {
    /* CEC_OPCODE_TEXT_VIEW_ON, CEC_OPCODE_IMAGE_VIEW_ON */
    EV_IMG_VIEW_ON = 0,
    EV_TEX_VIEW_ON,
    EV_ACT_SRCE,
    EV_IACT_SRCE,
    EV_STANDBY,
    EV_SET_MENU_LANGUAGE,
    EV_VENDOR_COMMAND,
    EV_REMOTE_BUTTON_DOWN,
    EV_OSD_STRING,
    EV_GET_MENU_LANGUAGE,
    EV_SET_AUDIO_MODE,
    EV_RECORD_TV_SCREEN,
    EV_RECORD_OFF,
    EV_SET_TIMER_PROG_TITLE,
    EV_SET_DIGITAL_TIMER,
    EV_CLEAR_DIGITAL_TIMER,
    EV_MENU_REQUEST,
    EV_DECK_CONTROL,
    EV_SET_TUNER_DIGITAL_SERVICE,
    EV_SET_TUNER_ANALOGUE_SERVICE,
    EV_REQ_TUNER_CTL,
    EV_AUDIO_MODE_REQUEST,
    EV_SET_AUDIO_RATE,
};

enum {
    CMD_IMG_VIEW_ON = 0,
    CMD_TEX_VIEW_ON,
    CMD_ACT_SRCE,
    CMD_IACT_SRCE,
    CMD_STANDBY,
    CMD_SET_MENU_LANGUAGE,
    CMD_VENDOR_COMMAND,
    CMD_REMOTE_BUTTON_DOWN,
    CMD_OSD_STRING,
    CMD_GET_MENU_LANGUAGE,
    CMD_SET_AUDIO_MODE,
    CMD_RECORD_TV_SCREEN,
    CMD_RECORD_OFF,
    CMD_SET_TIMER_PROG_TITLE,
    CMD_SET_DIGITAL_TIMER,
    CMD_CLEAR_DIGITAL_TIMER,
    CMD_MENU_REQUEST,
    CMD_DECK_CONTROL,
    CMD_SET_TUNER_DIGITAL_SERVICE,
    CMD_SET_TUNER_ANALOGUE_SERVICE,
    CMD_REQ_TUNER_CTL,
    CMD_AUDIO_MODE_REQUEST,
    CMD_SET_AUDIO_RATE,
};

struct cec_event {
    uint8_t ev_id;
    uint8_t laddr;
    uint8_t opencode;
    uint8_t param[16];
    uint8_t param_len;
};

struct cec_cmd
{
    uint8_t cmd_id;
    uint8_t ladd;
    uint8_t param[16];
    uint8_t opencode;
    uint8_t param_len;
};

#endif /* End of __CEC_HDMI_UTILS_H__ */
