/***********************************************************************************************************************
 * File Name    : application_utils.c
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
#include "application_utils.h"
#include "rtt_common_utils.h"

#define DEMO_SYSTEM_VOLUME_CHANGE_AMOUNT (10)

static volatile bool sw1_pushed_flag = false;
static volatile bool sw2_pushed_flag = false;

static int demo_system_current_volume = 20; /* Volume value in percent */
static volatile bool demo_system_mute_status = false;

extern volatile bool user_action_detect_flag;
extern cec_addr_t    user_action_cec_target;
extern uint8_t       user_action_type;

void led_pwm_duty_change(uint8_t duty_percent);

void user_button_irq_initialize(void)
{
    /* Open external irq driver for SW1 */
    R_ICU_ExternalIrqOpen(&g_external_irq_sw1_ctrl, &g_external_irq_sw1_cfg);

    /* Open external irq driver for SW2 */
    R_ICU_ExternalIrqOpen(&g_external_irq_sw2_ctrl, &g_external_irq_sw2_cfg);

    /* Enable irq for SW1 */
    R_ICU_ExternalIrqEnable(&g_external_irq_sw1_ctrl);

    /* Enable irq for SW2 */
    R_ICU_ExternalIrqEnable(&g_external_irq_sw2_ctrl);
}

void demo_system_initialize(void)
{
    /* Turn on the power status LED */
    R_IOPORT_PinWrite(&g_ioport_ctrl, POWER_STATUS_LED_PIN, BSP_IO_LEVEL_HIGH);

    /* Initialize internal flag */
    demo_system_mute_status = false;

    /* Open GPT driver to output PWM signal for the volume status LED */
    R_GPT_Open(&g_led_pwm_gpt_timer_ctrl, &g_led_pwm_gpt_timer_cfg);

    /* Applies initial volume value */
    R_GPT_DutyCycleSet(&g_led_pwm_gpt_timer_ctrl, (uint32_t)(g_led_pwm_gpt_timer_cfg.period_counts / 100 * (uint32_t) demo_system_current_volume), GPT_IO_PIN_GTIOCA);

    /* Start timer to output PWM */
    R_GPT_Start(&g_led_pwm_gpt_timer_ctrl);
}

void demo_system_power_on(void)
{
    /* Turn on the power status LED */
    R_IOPORT_PinWrite(&g_ioport_ctrl, POWER_STATUS_LED_PIN, BSP_IO_LEVEL_HIGH);
}

void demo_system_power_off(void)
{
    /* Turn off the power status LED */
    R_IOPORT_PinWrite(&g_ioport_ctrl, POWER_STATUS_LED_PIN, BSP_IO_LEVEL_LOW);
}

void demo_system_volume_change(bool is_mute, bool is_volume_up)
{
    if(is_mute)
    {
        if(demo_system_mute_status == false)
        {
            /* Update mute status flag */
            demo_system_mute_status = true;

            /* Apply volume 0 */
            led_pwm_duty_change(0);
        }
        else
        {
            /* Update mute status flag */
            demo_system_mute_status = false;

            /* Update PWM duty cycle */
            led_pwm_duty_change((uint8_t) demo_system_current_volume);
        }
    }
    else
    {
        /* Clear mute status flag */
        demo_system_mute_status = false;

        if(is_volume_up)
        {
            /* Volume up */
            demo_system_current_volume += DEMO_SYSTEM_VOLUME_CHANGE_AMOUNT;
            if(demo_system_current_volume >= 100)
            {
                demo_system_current_volume = 100;
            }
        }
        else
        {
            /* Volume down */
            demo_system_current_volume -= DEMO_SYSTEM_VOLUME_CHANGE_AMOUNT;
            if(demo_system_current_volume <= 0)
            {
                demo_system_current_volume = 0;
            }
        }

        /* Update PWM duty cycle */
        led_pwm_duty_change((uint8_t) demo_system_current_volume);
    }
}

void demo_system_volume_status_get(bool * p_mute_status, uint8_t * p_volume_status)
{
    *p_mute_status = demo_system_mute_status;
    *p_volume_status = (uint8_t)demo_system_current_volume;
}

void led_pwm_duty_change(uint8_t duty_percent)
{
    timer_status_t timer_status;

    if(duty_percent == 0)
    {
        R_GPT_StatusGet(&g_led_pwm_gpt_timer_ctrl, &timer_status);
        if(timer_status.state == TIMER_STATE_COUNTING)
        {
            R_GPT_Stop(&g_led_pwm_gpt_timer_ctrl);
        }

        R_IOPORT_PinWrite(&g_ioport_ctrl, VOLUME_STATUS_LED_PIN, BSP_IO_LEVEL_LOW);
    }
    else if(duty_percent == 100)
    {
        R_GPT_StatusGet(&g_led_pwm_gpt_timer_ctrl, &timer_status);
        if(timer_status.state == TIMER_STATE_COUNTING)
        {
            R_GPT_Stop(&g_led_pwm_gpt_timer_ctrl);
        }

        R_IOPORT_PinWrite(&g_ioport_ctrl, VOLUME_STATUS_LED_PIN, BSP_IO_LEVEL_HIGH);
    }
    else
    {
        R_GPT_DutyCycleSet(&g_led_pwm_gpt_timer_ctrl, (uint32_t)(g_led_pwm_gpt_timer_cfg.period_counts / 100 * duty_percent), GPT_IO_PIN_GTIOCA);

        R_GPT_StatusGet(&g_led_pwm_gpt_timer_ctrl, &timer_status);
        if(timer_status.state == TIMER_STATE_STOPPED)
        {
            R_GPT_Start(&g_led_pwm_gpt_timer_ctrl);
        }
    }
}

void user_action_check(void)
{
    fsp_err_t fsp_err = FSP_ERR_INVALID_DATA;
    uint8_t rtt_read_data_c;

    if(sw1_pushed_flag)
    {
        sw1_pushed_flag = false;

        user_action_detect_flag = true;
        user_action_type = USER_ACTION_REQUEST_POWER_ON;
        user_action_cec_target = CEC_ADDR_TV;
    }
    else if(sw2_pushed_flag)
    {
        sw2_pushed_flag = false;

        user_action_detect_flag = true;
        user_action_type = USER_ACTION_REQUEST_POWER_OFF;
        user_action_cec_target = CEC_ADDR_TV;
    }
    else
    {
        if(APP_CHECK_DATA)
        {
            /* Read data from RTT buffer */
            SEGGER_RTT_Read(0, &rtt_read_data_c, 1);

            if(rtt_read_data_c == '1')
            {
                user_action_detect_flag = true;
                user_action_type = USER_ACTION_BUS_SCAN;
            }
            else if(rtt_read_data_c == '2')
            {
                user_action_detect_flag = true;
                user_action_type = USER_ACTION_DISPLAY_CEC_BUS_STATUS_BUFF;
            }
            else if(rtt_read_data_c == '3')
            {
                user_action_detect_flag = true;
                user_action_type = USER_ACTION_ENABLING_SYSTEM_AUDIO_MODE_SUPPORT;
            }
            else if(rtt_read_data_c == '4')
            {
                user_action_detect_flag = true;
                user_action_type = USER_ACTION_SYSTEM_AUDIO_MODE_REQUEST;
            }
            else if(rtt_read_data_c == '0')
            {
                /* Specify command type */
                APP_PRINT(CEC_CONTROL_SELECT_MENU);

                while(1)
                {
                    if(APP_CHECK_DATA)
                    {
                        SEGGER_RTT_Read(0, &rtt_read_data_c, 1);

                        if(('a' <= rtt_read_data_c) && (rtt_read_data_c <= 'e'))
                        {
                            user_action_type = rtt_read_data_c;

                            fsp_err = FSP_SUCCESS;
                            break;
                        }
                        else if((rtt_read_data_c == '\r') || (rtt_read_data_c == '\n'))
                        {
                            /* Do nothing */
                        }
                        else
                        {
                            APP_PRINT("Invalid input.\r\n");
                            APP_PRINT(APP_COMMAND_OPTION,
                                      system_audio_mode_support_function ? SYS_AUDIO_FUNC_E : SYS_AUDIO_FUNC_D,
                                      system_audio_mode_status ? SYS_AUDIO_ON : SYS_AUDIO_OFF);
                            break;
                        }
                    }
                }

                /* Specify CEC target device */
                if(FSP_SUCCESS == fsp_err)
                {
                    user_action_cec_target = USER_ACTION_NONE;
                    APP_PRINT(CEC_CONTROL_DEVICE_SELECT_MENU);

                    while(1)
                    {
                        if(APP_CHECK_DATA)
                        {
                            SEGGER_RTT_Read(0, &rtt_read_data_c, 1);

                            if(('0' <= rtt_read_data_c) && (rtt_read_data_c <= '9'))
                            {
                                user_action_detect_flag = true;
                                user_action_cec_target = rtt_read_data_c - '0';
                                break;
                            }
                            else if ((rtt_read_data_c == 'a') || (rtt_read_data_c == 'b') || (rtt_read_data_c == 'f'))
                            {
                                user_action_detect_flag = true;
                                user_action_cec_target = rtt_read_data_c - 'a' + 0xa;
                                break;
                            }
                            else if((rtt_read_data_c == '\r') || (rtt_read_data_c == '\n'))
                            {
                                /* Do nothing */
                            }
                            else
                            {
                                APP_PRINT("Invalid input.\r\n");
                                APP_PRINT(APP_COMMAND_OPTION,
                                          system_audio_mode_support_function ? SYS_AUDIO_FUNC_E : SYS_AUDIO_FUNC_D,
                                          system_audio_mode_status ? SYS_AUDIO_ON : SYS_AUDIO_OFF);
                                break;
                            }
                        }
                    }
                }
            }
//            else if (rtt_read_data_c == <character>) ToDo
//            {
//                /* Add your additional operation */
//            }
            else if((rtt_read_data_c == '\r') || (rtt_read_data_c == '\n'))
            {
                /* Do nothing */
            }
            else
            {
                APP_PRINT("Invalid input.\r\n");
                APP_PRINT(APP_COMMAND_OPTION,
                          system_audio_mode_support_function ? SYS_AUDIO_FUNC_E : SYS_AUDIO_FUNC_D,
                          system_audio_mode_status ? SYS_AUDIO_ON : SYS_AUDIO_OFF);
            }
        }
    }
}

void user_action_process(uint8_t *cec_data)
{
        /* Basically, operating the remote controller turns on the device, so turn POWER_STATUS_LED_ON on. */
        if(cec_bus_device_list[my_logical_address].power_status == 0x0)
        {
            APP_PRINT("[System] Power On.\r\n");
            demo_system_power_on();
            cec_bus_device_list[my_logical_address].power_status = 0x1;
        }

        switch(user_action_type)
        {
            case USER_ACTION_BUS_SCAN: /* Scan CEC bus */
                cec_bus_scan();
                break;
            case USER_ACTION_DISPLAY_CEC_BUS_STATUS_BUFF: /* Display CEC bus buffer data */
                cec_bus_status_buffer_display();
                break;
            case USER_ACTION_REQUEST_POWER_ON: /* Power On (Image View On 0x04) */
                cec_message_send(user_action_cec_target, CEC_OPCODE_IMAGE_VIEW_ON, NULL, 0);
                break;
            case USER_ACTION_REQUEST_POWER_OFF: /* Power Off (Standby 0x36) */
                cec_message_send(user_action_cec_target, CEC_OPCODE_STANDBY, NULL, 0);
                break;
            case USER_ACTION_ENABLING_SYSTEM_AUDIO_MODE_SUPPORT:
                cec_system_audio_mode_support_enabling();
                break;
            case USER_ACTION_SYSTEM_AUDIO_MODE_REQUEST:
                cec_system_audio_mode_request();
                break;
            case USER_ACTION_REQUEST_VOLUME_UP: /* Volume Up. User Control Pressed 0x44 => User Control Released 0x45 */
                cec_data[0] = USER_CONTROL_VOLUME_UP;
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_PRESSED, &cec_data[0], 1); /* User Control Pressed */
                R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_RELEASED, NULL, 0); /* User Control Released */
                break;
            case USER_ACTION_REQUEST_VOLUME_DONW: /* Volume Down. User Control Pressed 0x44 => User Control Released 0x45 */
                cec_data[0] = USER_CONTROL_VOLUME_DOWN;
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_PRESSED, &cec_data[0], 1); /* User Control Pressed */
                R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_RELEASED, NULL, 0); /* User Control Released */
                break;
            case USER_ACTION_REQUEST_VOLUME_MUTE: /* Mute. User Control Pressed 0x44 => User Control Released 0x45 */
                cec_data[0] = USER_CONTROL_MUTE;
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_PRESSED, &cec_data[0], 1); /* User Control Pressed */
                R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
                cec_message_send(user_action_cec_target, CEC_OPCODE_USER_CONTROL_RELEASED, NULL, 0); /* User Control Released */
                break;
//            case <type defined> ToDo
//            {
//                /* Add your additional operation */
//                break;
//            }
            default:
                break;

        user_action_type = 0x0;
        APP_PRINT(APP_COMMAND_OPTION,
                  system_audio_mode_support_function ? SYS_AUDIO_FUNC_E : SYS_AUDIO_FUNC_D,
                  system_audio_mode_status ? SYS_AUDIO_ON : SYS_AUDIO_OFF);
        }
}

void irq_sw1_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    sw1_pushed_flag = true;
}

void irq_sw2_callback(external_irq_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    sw2_pushed_flag = true;
}

void led_pwm_gpt_callback(timer_callback_args_t * p_args)
{
    if(p_args->event == TIMER_EVENT_CAPTURE_A)
    {
        /* Turn off the volume status LED */
        R_IOPORT_PinWrite(&g_ioport_ctrl, VOLUME_STATUS_LED_PIN, BSP_IO_LEVEL_LOW);
    }
    else if(p_args->event == TIMER_EVENT_CYCLE_END)
    {
        /* Turn on the volume status LED */
        R_IOPORT_PinWrite(&g_ioport_ctrl, VOLUME_STATUS_LED_PIN, BSP_IO_LEVEL_HIGH);
    }
}

void cec_device_status_display(cec_addr_t cec_addr, cec_device_status_t * p_buff)
{
    APP_PRINT("+ %s", &cec_logical_device_list[cec_addr][0]);
    if(p_buff->is_my_device)
    {
        APP_PRINT("    <-- My device -->");
    }
    APP_PRINT("\r\n");

    APP_PRINT("|   Physical address : ");
    if(p_buff->is_physical_address_store)
    {
        APP_PRINT("%x.%x.%x.%x\r\n", p_buff->physical_address[3], p_buff->physical_address[2],
                  p_buff->physical_address[1], p_buff->physical_address[0]);
    }
    else
    {
        APP_PRINT("Unknown\r\n");
    }

    APP_PRINT("|   CEC version      : ");
    if(p_buff->is_version_store)
    {
        APP_PRINT("%s \r\n", &cec_version_list[p_buff->cec_version][0]);
    }
    else
    {
        APP_PRINT("Unknown\r\n");
    }

    APP_PRINT("|   Vendor ID        : ");
    if(p_buff->is_vendor_id_store)
    {
        APP_PRINT("0x%02x, 0x%02x, 0x%02x\r\n", p_buff->vendor_id[0], p_buff->vendor_id[1],
                  p_buff->vendor_id[2]);
    }
    else
    {
        APP_PRINT("Unknown\r\n");
    }

    APP_PRINT("|   Power status     : ");
    if(p_buff->is_power_status_store)
    {
        if(p_buff->power_status == 0x1)
        {
            APP_PRINT("On\r\n");
        }
        else
        {
            APP_PRINT("Standby\r\n");
        }
    }
    else
    {
        APP_PRINT("Unknown\r\n");
    }

    APP_PRINT("|   Active source    : ");
    if(p_buff->is_active_source)
    {
        APP_PRINT("Yes\r\n");
    }
    else
    {
        APP_PRINT("No\r\n");
    }
}

void vendor_id_install(uint8_t * p_vendor_id_buff)
{
    uint8_t rtt_data_count = 0;
    uint8_t rtt_data_buff[15] = {0x0};
    bool    rtt_data_error = false;
    bool    valid_data_received = false;
    uint8_t attempt_count = 0;

    APP_PRINT("Please input preferred vendor ID (Format: XX XX XX (X=Hex(0~F), little-endian)).\r\n");

    while(1)
    {
        if(APP_CHECK_DATA)
        {
            /* Read data from RTT buffer */
            uint8_t rtt_read_data_c;
            SEGGER_RTT_Read(0, &rtt_read_data_c, 1);

            rtt_data_buff[rtt_data_count] = rtt_read_data_c;

            if((rtt_data_count == 0) && ((rtt_read_data_c == '\r') || (rtt_read_data_c == '\n')))
            {
                rtt_data_count = 0;
            }
            else if(rtt_data_count >= 8)
            {
                if(rtt_read_data_c == '\r' || rtt_read_data_c == '\n')
                {
                    rtt_data_error = false;

                    for(int i=0; i<8; i++)
                    {
                        if((i % 3 == 0) || (i % 3 == 1))
                        {
                            if(!((('0' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= '9')) ||
                                 (('a' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'f')) ||
                                 (('A' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'F'))))
                            {
                                rtt_data_error |= true;
                            }
                        }
                    }

                    if(rtt_data_error == false)
                    {
                        valid_data_received = true;
                        break;
                    }
                    else
                    {
                        APP_PRINT("Invalid data.\r\n");
                        rtt_data_count = 0;
                        attempt_count++;
                    }
                }
                else
                {
                    APP_PRINT("Invalid data.\r\n");
                    rtt_data_count = 0;
                    attempt_count++;
                }
            }
            else
            {
                rtt_data_count++;
            }
        }

        if(2 < attempt_count)
        {
            break;
        }
    }

    if(valid_data_received)
    {
        uint8_t data[3];
        for(int i=0; i<8; i++)
        {
            if(i % 3 == 0)
            {
                data[i/3] = 0;

                if(('0' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= '9'))
                {
                    data[i/3] += (uint8_t)((rtt_data_buff[i] - '0') * 16);
                }
                else if(('a' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'f'))
                {
                    data[i/3] += (uint8_t)((rtt_data_buff[i] - 'a' + 10) * 16);
                }
                else if(('A' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'F'))
                {
                    data[i/3] += (uint8_t)((rtt_data_buff[i] - 'A' + 10) * 16);
                }
            }
            else if(i % 3 == 1)
            {
                if(('0' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= '9'))
                {
                    data[i/3] += (uint8_t)(rtt_data_buff[i] - '0');
                }
                else if(('1' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'f'))
                {
                    data[i/3] += (uint8_t)(rtt_data_buff[i] - 'a' + 10);
                }
                else if(('A' <= rtt_data_buff[i]) && (rtt_data_buff[i] <= 'F'))
                {
                    data[i/3] += (uint8_t)(rtt_data_buff[i] - 'A' + 10);
                }
            }
        }

        p_vendor_id_buff[0] = data[0];
        p_vendor_id_buff[1] = data[1];
        p_vendor_id_buff[2] = data[2];

        APP_PRINT("New vendor ID installed.\r\n");
    }
    else
    {
        APP_PRINT("Initial value will be used.\r\n");
    }
}
