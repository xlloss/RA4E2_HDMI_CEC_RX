/***********************************************************************************************************************
 * File Name    : hdmi_ddc_utils.h
 * Description  : Contains data structures and functions used in hdmi_ddc_utils.c.
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
#ifndef __HDMI_DDC_UTILS_H__
#define __HDMI_DDC_UTILS_H__
#include "hal_data.h"

#define HDMI_DDC_I2C_ADDR_DDC_CI (0x37)
#define HDMI_DDC_I2C_ADDR_HDCP   (0x3A)
#define HDMI_DDC_I2C_ADDR_EDID   (0x50)
#define HDMI_DDC_I2C_ADDR_SCDC   (0x54)

#define EDID_DATA_SIZE     (128)
#define EDID_CTA_DATA_SIZE (128)

#define EDID_HEADER_FIXED_HEADER_PATTERN_FIRST_32BIT  (0xFFFFFF00)
#define EDID_HEADER_FIXED_HEADER_PATTERN_SECOND_32BIT (0x00FFFFFF)
#define EDID_CTA_EXTENSION_TAG (0x02)
#define EDID_CTA_IEEE_IDENTIFIER_HDMI (0x000C03)

/*
 * Definition for EDID field
 */
typedef struct edid_header
{
    uint32_t header_pattern[2];
    uint8_t manifacturer_id[2];
    uint8_t manufacturer_product_code[2];
    uint8_t serial_number[4];
    uint8_t week_of_manufacture;
    uint8_t year_of_manufacture;
    uint8_t edid_version;
    uint8_t edid_revision;
}edid_header_t; /* Total 20 bytes */

typedef struct edid_basic_display_params
{
    uint8_t video_input_params;
    uint8_t horizontal_screen_size;
    uint8_t vertical_screen_size;
    uint8_t display_gamma;
    uint8_t supported_features;
}edid_basic_display_params_t; /* Total 5 bytes */

typedef struct edid_chromaticity_coordinates
{
    uint8_t red_and_green_least_significant;
    uint8_t blue_and_white_least_significant;
    uint8_t red_x_most_significant;
    uint8_t red_y_most_significant;
    uint8_t green_x_and_y_most_significant[2];
    uint8_t blue_x_and_y_most_significant[2];
    uint8_t default_white_point_x_and_y_most_significant[2];
}edid_chromaticity_coordinates_t; /* Total 10 bytes */

typedef struct edid_established_timing_bitmap
{
    uint8_t bitmap_1;
    uint8_t bitmap_2;
    uint8_t bitmap_3;
}edid_established_timing_bitmap_t; /* Total 3 bytes */

typedef struct edid_standard_timing_info
{
    uint8_t standard_timing_1_x;
    uint8_t standard_timing_1_aspect_and_vertical_freq;
    uint8_t standard_timing_2[2];
    uint8_t standard_timing_3[2];
    uint8_t standard_timing_4[2];
    uint8_t standard_timing_5[2];
    uint8_t standard_timing_6[2];
    uint8_t standard_timing_7[2];
    uint8_t standard_timing_8[2];
}edid_standard_timing_info_t; /* Total 16 bytes */

typedef struct edid_display_timing_descriptor
{
    uint8_t preferred_timing_descriptor[18];
    uint8_t descriptor_2[18];
    uint8_t descriptor_3[18];
    uint8_t descriptor_4[18];
}edid_display_timing_descriptor_t; /* Total 72 bytes */

typedef struct edid_data
{
    edid_header_t                    header;
    edid_basic_display_params_t      basic_display_params;
    edid_chromaticity_coordinates_t  chromaticity_coordinates;
    edid_established_timing_bitmap_t established_timing_bitmap;
    edid_standard_timing_info_t      standard_timing_info;
    edid_display_timing_descriptor_t display_timing_descriptor;
    uint8_t                          number_of_extensions;
    uint8_t                          checksum;
}edid_data_t; /* Total 128 bytes */

/*
 * Definition for EDID CTA-861 Extension field
 */
typedef struct edid_cta_extention_data
{
    uint8_t extention_tag;
    uint8_t revision_number;
    uint8_t byte_number;
    uint8_t number_of_native_dtd_present;
    uint8_t data_block[123];
    uint8_t checksum;
}edid_cta_extention_data_t; /* Total 128 bytes */

typedef enum edid_cta_data_block_type
{
    CTA_DATA_TYPE_AUDIO                      = 0x1,
    CTA_DATA_TYPE_VIDEO                      = 0x2,
    CTA_DATA_TYPE_VENDOR_SPECIFIC            = 0x3,
    CTA_DATA_TYPE_SPEAKER_ALLOCATION         = 0x4,
    CTA_DATA_TYPE_VESA_DISPLAY_TRANSFER_CHAR = 0x5,
    CTA_DATA_TYPE_VIDEO_FORMAT               = 0x6,
    CTA_DATA_TYPE_EXTENDED                   = 0x7
}edid_cta_data_block_type_t;

typedef struct edid_cta_data_block_header
{
    uint8_t                    byte_number    : 5;
    edid_cta_data_block_type_t block_type_tag : 3;
}edid_cta_data_block_header_t; /* Total 1 byte */

typedef struct edid_cta_data_vendor_specific
{
    edid_cta_data_block_header_t header;
    uint32_t                     ieee_identifier : 24;

    uint8_t physical_address_3 : 4;
    uint8_t physical_address_4 : 4;
    uint8_t physical_address_1 : 4;
    uint8_t physical_address_2 : 4;

    uint8_t optional_data[3];
    uint8_t latency[4];
    /* Additional bytes may be present, but the HDMI spec. says they shall be 00. */
}edid_cta_data_vendor_specific_t;

fsp_err_t edid_format_check(edid_data_t const *data);
fsp_err_t edid_cta_format_check(edid_cta_extention_data_t const *data);

fsp_err_t physical_address_get(uint8_t *addr);
fsp_err_t edid_cta_physical_address_find(edid_cta_extention_data_t const *data, uint8_t *address);

#endif /* End of __HDMI_DDC_UTILS_H__ */
