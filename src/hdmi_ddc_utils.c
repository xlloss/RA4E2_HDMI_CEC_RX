/***********************************************************************************************************************
 * File Name    : hdmi_ddc_utils.c
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
#include "hdmi_ddc_utils.h"
#include "rtt_common_utils.h"

///############# Application Option Setting #############
#define DEBUG_EDID_RECEIVED_DATA_OUTPUT  (0) // 0: Disabled, 1: Enabled
///########## End of Application Option Setting #########

static volatile bool iic_rx_complete_flag = false;
static volatile bool iic_tx_complete_flag = false;
static volatile bool iic_error_flag = false;

static edid_data_t               edid_base_data;
static edid_cta_extention_data_t edid_cta_data;


fsp_err_t edid_format_check(edid_data_t const *data)
{
    uint8_t checksum_sum_result = 0x0;

    /* Check the header pattern. 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 */
    if(!((data->header.header_pattern[0] == EDID_HEADER_FIXED_HEADER_PATTERN_FIRST_32BIT) &&
         (data->header.header_pattern[1] == EDID_HEADER_FIXED_HEADER_PATTERN_SECOND_32BIT)))
    {
        return FSP_ERR_INVALID_DATA;
    }

    /* Verify the checksum */
    for(int i=0; i<127; i++)
    {
        checksum_sum_result += *((uint8_t *)data + i);
    }

    if((0xFF - checksum_sum_result + 1) != data->checksum)
    {
        return FSP_ERR_INVALID_DATA;
    }

    return FSP_SUCCESS;
}

fsp_err_t edid_cta_format_check(edid_cta_extention_data_t const *data)
{
    uint8_t checksum_sum_result = 0x0;

    /* Check the extension flag. 0x02 for CTA EDID */
    if(data->extention_tag != EDID_CTA_EXTENSION_TAG)
    {
        return FSP_ERR_INVALID_DATA;
    }

    /* Verify the checksum */
    for(int i=0; i<127; i++)
    {
        checksum_sum_result += *((uint8_t *)data + i);
    }

    if((0xFF - checksum_sum_result + 1) != data->checksum)
    {
        return FSP_ERR_INVALID_DATA;
    }

    return FSP_SUCCESS;
}

fsp_err_t physical_address_get(uint8_t *addr)
{
    fsp_err_t fsp_err = FSP_SUCCESS;
    uint8_t iic_transfer_data[10] = {0x0};
    uint8_t iic_read_data[128] = {0x0};

    /* Open R_IIC master driver */
//    fsp_err = R_IIC_MASTER_Open(&g_ddc_source_i2c_master_ctrl, &g_ddc_source_i2c_master_cfg);
//    fsp_err = R_SCI_I2C_Open(&g_ddc_source_i2c_master_ctrl, &g_ddc_source_i2c_master_cfg);
//    if(FSP_SUCCESS != fsp_err)
//    {
//        APP_PRINT("R_IIC_MASTER_Open failed\r\n");
//        return fsp_err;
//    }

    /* Initialize internal flags */
    iic_tx_complete_flag = false;
    iic_error_flag = false;

    /* Set slave address of DDC EDID */
//    fsp_err = R_SCI_I2C_SlaveAddressSet(&g_ddc_source_i2c_master_ctrl, HDMI_DDC_I2C_ADDR_EDID, I2C_MASTER_ADDR_MODE_7BIT);
//    if(FSP_SUCCESS != fsp_err)
//    {
//        APP_PRINT("R_IIC_MASTER_SlaveAddressSet failed\r\n");
//        return fsp_err;
//    }

    /// Read EDID 128 byte
    /* Offset of first 128 byte EDID is 0x00 */
    iic_transfer_data[0] = 0x00;

    /* Execute writing offset */
//    fsp_err = R_SCI_I2C_Write(&g_ddc_source_i2c_master_ctrl, iic_transfer_data, 1, false);
//    if(FSP_SUCCESS != fsp_err)
//    {
//        APP_PRINT("R_IIC_MASTER_Write failed\r\n");
//        return fsp_err;
//
//    }

    /* Wait for tx completion */
    while(!iic_tx_complete_flag)
    {
        if(iic_error_flag)
        {
            iic_error_flag = false;
            APP_PRINT("CEC-DDC channel error\r\n");
            return FSP_ERR_ABORTED;
        }
    }
    iic_tx_complete_flag = false;

    /* Execute 128 byte read */
//    fsp_err = R_IIC_MASTER_Read(&g_ddc_source_i2c_master_ctrl, iic_read_data, 128, false);
//    if(FSP_SUCCESS != fsp_err)
//    {
//        APP_PRINT("R_IIC_MASTER_Read failed\r\n");
//        return fsp_err;
//    }

    /* Wait for rx completion */
    while(!iic_rx_complete_flag)
    {
        if(iic_error_flag)
        {
            APP_PRINT("CEC-DDC channel error\r\n");
            return FSP_ERR_ABORTED;
        }
    }
    iic_rx_complete_flag = false;

    memcpy(&edid_base_data, &iic_read_data[0], EDID_DATA_SIZE);

#if (DEBUG_EDID_RECEIVED_DATA_OUTPUT == 1)
    APP_PRINT("\r\nEDID (First 128byte):");
    for(uint32_t i=0; i<sizeof(iic_read_data); i++)
    {
        if(i % 16 == 0)
        {
            APP_PRINT("\r\n");
        }
        APP_PRINT("0x%02x ", iic_read_data[i]);
    }
    APP_PRINT("\r\n");
#endif

    fsp_err = edid_format_check(&edid_base_data);
    if(FSP_SUCCESS != fsp_err)
    {
        APP_PRINT("Received EDID data is broken\r\n");
        return fsp_err;
    }

    if(edid_base_data.number_of_extensions != 0)
    {
        /* Read EDID next 128 byte */
//        iic_transfer_data[0] = 0x80;

        iic_tx_complete_flag = false;
        iic_error_flag = false;
//        fsp_err = R_IIC_MASTER_Write(&g_ddc_source_i2c_master_ctrl, iic_transfer_data, 1, false);
//        if(FSP_SUCCESS != fsp_err)
//        {
//            APP_PRINT("R_IIC_MASTER_Write failed\r\n");
//            return fsp_err;
//        }

        while(!iic_tx_complete_flag)
        {
            if(iic_error_flag)
            {
                APP_PRINT("CEC-DDC channel error\r\n");
                return FSP_ERR_ABORTED;
            }
        }
        iic_tx_complete_flag = false;

        iic_rx_complete_flag = false;
        iic_error_flag = false;
//        fsp_err = R_IIC_MASTER_Read(&g_ddc_source_i2c_master_ctrl, iic_read_data, 128, false);
//        if(FSP_SUCCESS != fsp_err)
//        {
//            APP_PRINT("R_IIC_MASTER_Write failed\r\n");
//            return fsp_err;
//        }

        while(!iic_rx_complete_flag)
        {
            if(iic_error_flag)
            {
                APP_PRINT("CEC-DDC channel error\r\n");
                return FSP_ERR_ABORTED;
            }
        }
        iic_rx_complete_flag = false;

        memcpy(&edid_cta_data, &iic_read_data[0], EDID_CTA_DATA_SIZE);

#if (DEBUG_EDID_RECEIVED_DATA_OUTPUT == 1)
        APP_PRINT("EDID (Next 128byte):");
        for(uint32_t i=0; i<sizeof(iic_read_data); i++)
        {
            if(i % 16 == 0)
            {
                APP_PRINT("\r\n");
            }
            APP_PRINT("0x%02x ", iic_read_data[i]);
        }
        APP_PRINT("\r\n\r\n");
#endif

        fsp_err = edid_cta_format_check(&edid_cta_data);
        if(FSP_SUCCESS != fsp_err)
        {
            APP_PRINT("Received EDID CTA data is broken or it's not CTA formatted data\r\n");
            return fsp_err;
        }

        fsp_err = edid_cta_physical_address_find((edid_cta_extention_data_t *)&edid_cta_data, addr);
        if(fsp_err != FSP_SUCCESS)
        {
            return fsp_err;
        }
    }

    return FSP_SUCCESS;
}

//void ddc_source_iic_callback(i2c_master_callback_args_t *p_args)
//{
//    if(I2C_MASTER_EVENT_RX_COMPLETE == p_args->event)
//    {
//        iic_rx_complete_flag = true;
//    }
//    else if(I2C_MASTER_EVENT_TX_COMPLETE == p_args->event)
//    {
//        iic_tx_complete_flag = true;
//    }
//    else
//    {
//        iic_error_flag = true;
//    }
//}

fsp_err_t edid_cta_physical_address_find(edid_cta_extention_data_t const *data, uint8_t *address)
{
    fsp_err_t                    result = FSP_SUCCESS;

    uint8_t                      scan_point = 0x0;
    edid_cta_data_block_header_t header;

    /* Check the revision number. 0x03 for version 3 (from CTA 861-B onward) */
    /* HDMI 1.3a and 1.4 use 861-D. So this value should be 0x03 */
    if(data->revision_number != 0x3)
    {
        return FSP_ERR_UNSUPPORTED;
    }

    /* Attempt to find Vendor Specific Data Block for "HDMI Licensing LLC" in data_block field */
    result = FSP_ERR_NOT_FOUND;

    do{
        memcpy(&header, &data->data_block[scan_point], sizeof(edid_cta_data_block_header_t));

        if(header.block_type_tag == CTA_DATA_TYPE_VENDOR_SPECIFIC)
        {
            edid_cta_data_vendor_specific_t data_vendor;
            memcpy(&data_vendor, &data->data_block[scan_point], sizeof(edid_cta_data_vendor_specific_t));

            /* Check IEEE Registration Identifier */
            if(data_vendor.ieee_identifier == EDID_CTA_IEEE_IDENTIFIER_HDMI)
            {
                address[0] = data_vendor.physical_address_1;
                address[1] = data_vendor.physical_address_2;
                address[2] = data_vendor.physical_address_3;
                address[3] = data_vendor.physical_address_4;

                result = FSP_SUCCESS;
                break;
            }
        }

        if((1 <= header.block_type_tag) && (header.block_type_tag <= 7))
        {
            scan_point += (uint8_t)(header.byte_number + 1);
        }
        else
        {
            scan_point++;
        }
    }while(scan_point < 123);

    return result;
}
