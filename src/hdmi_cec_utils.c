/***********************************************************************************************************************
 * File Name    : hdmi_cec_utils.c
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
#include "hdmi_cec_utils.h"
#include "rtt_common_utils.h"

/* List of characters of Logical Address. Refer to CEC Table 5 in HDMI Specification */
uint8_t cec_logical_device_list[16][23] =
{
 "TV",
 "Recording Device 1",
 "Recording Device 2",
 "Tuner 1",
 "Playback Device 1",
 "Audio System",
 "Tuner 2",
 "Tuner 3",
 "Playback Device 2",
 "Recording Device 3",
 "Tuner 4",
 "Playback Device 3",
 "(Reserved)",
 "(Reserved)",
 "Specific Use",
 "Unregistered/Broadcast"
};

/* List of characters of [CEC Version]. Refer to CEC 17 in HDMI Specification */
uint8_t cec_version_list[6][5] =
{
 "1.1",
 "1.2",
 "1.2a",
 "1.3",
 "1.3a",
 "1.4"
};

/* List of characters of [Power Status]. Refer to CEC 17 in HDMI Specification */
uint8_t cec_power_status_list[4][28] =
{
 "On",
 "Standby",
 "In transition Standby to On",
 "In transition On to Standby",
};

/* List of characters of Features. Refer to CEC 3.1 and 3.2 in HDMI Specification */
cec_feature_define_t cec_feature_list[] =
{
 /* End-User Features */
 {.feature = CEC_FEAT_ONE_TOUCH_PLAY,                   .feature_desc_str = "One Touch Play"},
 {.feature = CEC_FEAT_SYSTEM_STANDBY,                   .feature_desc_str = "System Standby"},
 {.feature = CEC_FEAT_ONE_TOUCH_RECORD,                 .feature_desc_str = "One Touch Record"},
 {.feature = CEC_FEAT_TIMER_PROGRAMMING,                .feature_desc_str = "Timer Programming"},
 {.feature = CEC_FEAT_DECK_CONTROL,                     .feature_desc_str = "Deck Control"},
 {.feature = CEC_FEAT_TUNER_CONTROL,                    .feature_desc_str = "Tuner Control"},
 {.feature = CEC_FEAT_DEVICE_MENU_CONTROL,              .feature_desc_str = "Device Menu Control"},
 {.feature = CEC_FEAT_REMOTE_CONTROL_PASS_THROUGH,      .feature_desc_str = "Remote Ctrl Pass Thru"},
 {.feature = CEC_FEAT_SYSTEM_AUDIO_CONTROL,             .feature_desc_str = "System Audio Control"},
 /* Supporting Features */
 {.feature = CEC_FEAT_DEVICE_OSD_NAME_TRANS,            .feature_desc_str = "OSD Name Transfer"},
 {.feature = CEC_FEAT_DEVICE_POWER_STATUS,              .feature_desc_str = "Device Power Status"},
 {.feature = CEC_FEAT_OSD_DISPLAY,                      .feature_desc_str = "OSD Display"},
 {.feature = CEC_FEAT_ROUTING_CONTROL,                  .feature_desc_str = "Routing Control"},
 {.feature = CEC_FEAT_SYSTEM_INFO,                      .feature_desc_str = "System Information"},
 {.feature = CEC_FEAT_VENDOR_SPECIFIC,                  .feature_desc_str = "Vendor Specific"},
 {.feature = CEC_FEAT_AUDIO_RATE_CONTROL,               .feature_desc_str = "Audio Rate Control"},
 {.feature = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL,     .feature_desc_str = "Audio Return Channel Control"},
 {.feature = CEC_FEAT_CAPABILITY_DISCOVERY_AND_CONTROL, .feature_desc_str = "Capability Discovery And Control"},
};

uint32_t cec_feature_list_number = sizeof(cec_feature_list) / sizeof(cec_feature_define_t);

/* List of characters of Operand Description. Refer to CEC 15 in HDMI Specification */
cec_opcode_define_t cec_opcode_list[] =
{
 {.opcode = CEC_OPCODE_FEATURE_ABORT,                  .opcode_desc_str = "Feature Abort",                  .feature_bits = 0xFFFFFFFF},
 {.opcode = CEC_OPCODE_ABORT,                          .opcode_desc_str = "Abort Message",                  .feature_bits = 0xFFFFFFFF},
 {.opcode = CEC_OPCODE_ACTIVE_SOURCE,                  .opcode_desc_str = "Active Source",                  .feature_bits = CEC_FEAT_ONE_TOUCH_PLAY | CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_IMAGE_VIEW_ON,                  .opcode_desc_str = "Image View On",                  .feature_bits = CEC_FEAT_ONE_TOUCH_PLAY},
 {.opcode = CEC_OPCODE_TEXT_VIEW_ON,                   .opcode_desc_str = "Text View On",                   .feature_bits = CEC_FEAT_ONE_TOUCH_PLAY},
 {.opcode = CEC_OPCODE_STANDBY,                        .opcode_desc_str = "Standby",                        .feature_bits = CEC_FEAT_SYSTEM_STANDBY},
 {.opcode = CEC_OPCODE_RECORD_OFF,                     .opcode_desc_str = "Record Off",                     .feature_bits = CEC_FEAT_ONE_TOUCH_RECORD},
 {.opcode = CEC_OPCODE_RECORD_ON,                      .opcode_desc_str = "Record On",                      .feature_bits = CEC_FEAT_ONE_TOUCH_RECORD},
 {.opcode = CEC_OPCODE_RECORD_STATUS,                  .opcode_desc_str = "Record Status",                  .feature_bits = CEC_FEAT_ONE_TOUCH_RECORD},
 {.opcode = CEC_OPCODE_RECORD_TV_SCREEN,               .opcode_desc_str = "Record TV Screen",               .feature_bits = CEC_FEAT_ONE_TOUCH_RECORD},
 {.opcode = CEC_OPCODE_CLEAR_ANALOG_TIMER,             .opcode_desc_str = "Clear Analogue Timer",           .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_CLEAR_DIGITAL_TIMER,            .opcode_desc_str = "Clear Digital Timer",            .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_CLEAR_EXTERNAL_TIMER,           .opcode_desc_str = "Clear External Timer",           .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_SET_ANALOG_TIMER,               .opcode_desc_str = "Set Analogue Timer",             .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_SET_DIGITAL_TIMER,              .opcode_desc_str = "Set Digital Timer",              .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_SET_EXTERNAL_TIMER,             .opcode_desc_str = "Set External Timer",             .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_SET_TIMER_PROGRAM_TITLE,        .opcode_desc_str = "Set Timer Program Title",        .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_TIMER_CLEARED_STATUS,           .opcode_desc_str = "Timer Cleared Status",           .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_TIMER_STATUS,                   .opcode_desc_str = "Timer Status",                   .feature_bits = CEC_FEAT_TIMER_PROGRAMMING},
 {.opcode = CEC_OPCODE_DECK_CONTROL,                   .opcode_desc_str = "Deck Control",                   .feature_bits = CEC_FEAT_DECK_CONTROL},
 {.opcode = CEC_OPCODE_DECK_STATUS,                    .opcode_desc_str = "Deck Status",                    .feature_bits = CEC_FEAT_DECK_CONTROL},
 {.opcode = CEC_OPCODE_GIVE_DECK_STATUS,               .opcode_desc_str = "Give Deck Status",               .feature_bits = CEC_FEAT_DECK_CONTROL},
 {.opcode = CEC_OPCODE_PLAY,                           .opcode_desc_str = "Play",                           .feature_bits = CEC_FEAT_DECK_CONTROL},
 {.opcode = CEC_OPCODE_GIVE_TUNER_STATUS,              .opcode_desc_str = "Give Tuner Status",              .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_SELECT_ANALOG_SERVICE,          .opcode_desc_str = "Select Analogue Service",        .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_SELECT_DIGITAL_SERVICE,         .opcode_desc_str = "Select Digital Service",         .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_TUNER_DEVICE_STATUS,            .opcode_desc_str = "Tuner Device Status",            .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_TUNER_STEP_DECREMENT,           .opcode_desc_str = "Tuner Step Decrement",           .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_TUNER_STEP_INCREMENT,           .opcode_desc_str = "Tuner Step Increment",           .feature_bits = CEC_FEAT_TUNER_CONTROL},
 {.opcode = CEC_OPCODE_MENU_REQUEST,                   .opcode_desc_str = "Menu Request",                   .feature_bits = CEC_FEAT_DEVICE_MENU_CONTROL},
 {.opcode = CEC_OPCODE_MENU_STATUS,                    .opcode_desc_str = "Menu Status",                    .feature_bits = CEC_FEAT_DEVICE_MENU_CONTROL},
 {.opcode = CEC_OPCODE_USER_CONTROL_PRESSED,           .opcode_desc_str = "User Control Pressed",           .feature_bits = CEC_FEAT_DEVICE_MENU_CONTROL | CEC_FEAT_REMOTE_CONTROL_PASS_THROUGH | CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_USER_CONTROL_RELEASED,          .opcode_desc_str = "User Control Released",          .feature_bits = CEC_FEAT_DEVICE_MENU_CONTROL | CEC_FEAT_REMOTE_CONTROL_PASS_THROUGH | CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_GIVE_AUDIO_STATUS,              .opcode_desc_str = "Give Audio Status",              .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_GIVE_SYSTEM_AUDIO_MODE_STATUS,  .opcode_desc_str = "Give Audio Mode Status",         .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_AUDIO_STATUS,            .opcode_desc_str = "Report Audio Status",            .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_SHORT_AUDIO_DESCRIPTOR,  .opcode_desc_str = "Report Short Audio Descriptor",  .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_REQUEST_SHORT_AUDIO_DESCRIPTOR, .opcode_desc_str = "Request Short Audio Descriptor", .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_SET_SYSTEM_AUDIO_MODE,          .opcode_desc_str = "Set System Audio Mode",          .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_SYSTEM_AUDIO_MODE_REQUEST,      .opcode_desc_str = "System Audio Mode Request",      .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_SYSTEM_AUDIO_MODE_STATUS,       .opcode_desc_str = "System Audio Mode Status",       .feature_bits = CEC_FEAT_SYSTEM_AUDIO_CONTROL},
 {.opcode = CEC_OPCODE_GIVE_OSD_NAME,                  .opcode_desc_str = "Give OSD Name",                  .feature_bits = CEC_FEAT_DEVICE_OSD_NAME_TRANS},
 {.opcode = CEC_OPCODE_SET_OSD_NAME,                   .opcode_desc_str = "Set OSD Name",                   .feature_bits = CEC_FEAT_DEVICE_OSD_NAME_TRANS},
 {.opcode = CEC_OPCODE_GIVE_POWER_STATUS,              .opcode_desc_str = "Give Power Status",              .feature_bits = CEC_FEAT_DEVICE_POWER_STATUS},
 {.opcode = CEC_OPCODE_REPORT_POWER_STATUS,            .opcode_desc_str = "Report Power Status",            .feature_bits = CEC_FEAT_DEVICE_POWER_STATUS},
 {.opcode = CEC_OPCODE_SET_OSD_STRING,                 .opcode_desc_str = "Set OSD String",                 .feature_bits = CEC_FEAT_OSD_DISPLAY},
 {.opcode = CEC_OPCODE_INACTIVE_SOURCE,                .opcode_desc_str = "Inactive Source",                .feature_bits = CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_REQUEST_ACTIVE_SOURCE,          .opcode_desc_str = "Request Active Source",          .feature_bits = CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_ROUTING_CHANGE,                 .opcode_desc_str = "Routing Change",                 .feature_bits = CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_ROUTING_INFORMATION,            .opcode_desc_str = "Routing Information",            .feature_bits = CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_SET_STREAM_PATH,                .opcode_desc_str = "Set Stream Path",                .feature_bits = CEC_FEAT_ROUTING_CONTROL},
 {.opcode = CEC_OPCODE_CEC_VERSION,                    .opcode_desc_str = "CEC Version",                    .feature_bits = CEC_FEAT_SYSTEM_INFO | CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_GET_CEC_VERSION,                .opcode_desc_str = "Get CEC Version",                .feature_bits = CEC_FEAT_SYSTEM_INFO | CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_GIVE_PHYSICAL_ADDRESS,          .opcode_desc_str = "Give Physical Address",          .feature_bits = CEC_FEAT_SYSTEM_INFO},
 {.opcode = CEC_OPCODE_GET_MENU_LANGUAGE,              .opcode_desc_str = "Get Menu Language",              .feature_bits = CEC_FEAT_SYSTEM_INFO},
 {.opcode = CEC_OPCODE_REPORT_PHYSICAL_ADDRESS,        .opcode_desc_str = "Report Physical Address",        .feature_bits = CEC_FEAT_SYSTEM_INFO},
 {.opcode = CEC_OPCODE_SET_MENU_LANGUAGE,              .opcode_desc_str = "Set Menu Language",              .feature_bits = CEC_FEAT_SYSTEM_INFO},
 {.opcode = CEC_OPCODE_DEVICE_VENDOR_ID,               .opcode_desc_str = "Device Vendor ID",               .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_GIVE_DEVICE_VENDOR_ID,          .opcode_desc_str = "Give Device Vendor ID",          .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_VENDOR_COMMAND,                 .opcode_desc_str = "Vendor Command",                 .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_VENDOR_COMMNAD_W_ID,            .opcode_desc_str = "Vendor Command w/ ID",           .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_VENDOR_REMOTE_BUTTON_DOWN,      .opcode_desc_str = "Vendor Remote Button Down",      .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_VENDOR_REMOTE_BUTTON_UP,        .opcode_desc_str = "Vendor Remote Button Up",        .feature_bits = CEC_FEAT_VENDOR_SPECIFIC},
 {.opcode = CEC_OPCODE_SET_AUDIO_RATE,                 .opcode_desc_str = "Set Audio Rate",                 .feature_bits = CEC_FEAT_AUDIO_RATE_CONTROL},
 {.opcode = CEC_OPCODE_INITIATE_ARC,                   .opcode_desc_str = "Initiate ARC",                   .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_ARC_INITIATED,           .opcode_desc_str = "Report ARC Initiated",           .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_ARC_TERMINATED,          .opcode_desc_str = "Report ARC Terminated",          .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_ARC_INITIATION,          .opcode_desc_str = "Report ARC Initiation",          .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_REPORT_ARC_TERMINATION,         .opcode_desc_str = "Report ARC Termination",         .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_TERMINATE_ARC,                  .opcode_desc_str = "Terminate ARC",                  .feature_bits = CEC_FEAT_AUDIO_RETURN_CHANNEL_CONTROL},
 {.opcode = CEC_OPCODE_CDC_MESSAGE,                    .opcode_desc_str = "CDC Message",                    .feature_bits = CEC_FEAT_CAPABILITY_DISCOVERY_AND_CONTROL},

 {.opcode = CEC_OPCODE_UNKNOWN,                        .opcode_desc_str = "Unknown Opcode",                 .feature_bits = 0xFFFFFFFF},
};

uint32_t cec_opcode_list_number = sizeof(cec_opcode_list) / sizeof(cec_opcode_define_t);

uint32_t opcode_description_find(uint8_t opcode)
{
    for(uint32_t i=0; i<(cec_opcode_list_number - 1); i++)
    {
        if(cec_opcode_list[i].opcode == opcode)
        {
            return i;
        }
    }

    return cec_opcode_list_number - 1;
}

cec_device_type_t convert_logical_address_to_device_type(cec_addr_t addr)
{
    cec_device_type_t device_type;

    switch(addr)
    {
        case CEC_ADDR_TV:
            device_type = CEC_DEVICE_TYPE_TV;
            break;
        case CEC_ADDR_RECORDING_DEVICE_1:
        case CEC_ADDR_RECORDING_DEVICE_2:
        case CEC_ADDR_RECORDING_DEVICE_3:
            device_type = CEC_DEVICE_TYPE_RECORDING_DEVICE;
            break;
        case CEC_ADDR_TUNER_1:
        case CEC_ADDR_TUNER_2:
        case CEC_ADDR_TUNER_3:
        case CEC_ADDR_TUNER_4:
            device_type = CEC_DEVICE_TYPE_TUNER;
            break;
        case CEC_ADDR_PLAYBACK_DEVICE_1:
        case CEC_ADDR_PLAYBACK_DEVICE_2:
        case CEC_ADDR_PLAYBACK_DEVICE_3:
            device_type = CEC_DEVICE_TYPE_PLAYBACK_DEVICE;
            break;
        case CEC_ADDR_AUDIO_SYSTEM:
            device_type = CEC_DEVICE_TYPE_AUDIO_SYSTEM;
            break;
        default:
            device_type = CEC_DEVICE_TYPE_UNKNOWN;
            break;
    }

    return device_type;
}
