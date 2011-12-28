/* drivers/input/touchscreen/atmel_mxt165.c
 *
 * Copyright (c) 2011 2011 Foxconn Communication Technology Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <../smd_private.h>	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+

#include <linux/atmel_mxt165.h>
#include <linux/atmel_mxt165_info_block_driver.h>
#include <linux/atmel_mxt165_std_objects_driver.h>
#include <linux/atmel_mxt165_touch_driver.h>

info_block_t *info_block;
static uint16_t g_I2CAddress;
static uint16_t g_RegAddr[DEFAULT_REG_NUM];
static uint8_t g_MsgData[8];
static uint8_t g_ReportID[256];

static int mxt_debug_level;
module_param_named(
    debug_level, mxt_debug_level, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int cap_touch_fw_version;
module_param_named(
    fw_version, cap_touch_fw_version, int, S_IRUGO | S_IWUSR | S_IWGRP
);

// Return 1: Down, 0: Up, -1: Failed.
int Touch_FakeProximity(void)
{
    if (atmel_mxt165 == NULL)
    {
        TCH_ERR("atmel_mxt165 is a null pointer.");
        return -1;
    }
    else
    {
        TCH_DBG(DB_LEVEL1, "fake_proximity = %d", atmel_mxt165->fake_proximity);
        return atmel_mxt165->fake_proximity;
    }
}
EXPORT_SYMBOL(Touch_FakeProximity);

uint8_t read_mem(uint16_t start, uint8_t size, uint8_t *mem)
{
    struct i2c_adapter *adap = atmel_mxt165->client->adapter;
    struct i2c_msg msgs[]= {
        [0] = {
                .addr = atmel_mxt165->client->addr,
                .flags = 0,
                .len = 2,
                .buf = (uint8_t*)&start,
        },

        [1] = {
                .addr = atmel_mxt165->client->addr,
                .flags = 1,
                .len = size,
                .buf = mem,
        },
    };

    return (i2c_transfer(adap, msgs, 2) < 0) ? -EIO : 0;
}

uint8_t write_mem(uint16_t start, uint8_t size, uint8_t *mem)
{
    uint8_t data[256];  // max tranfer data size = 256-2 (16 bit reg address)
    uint8_t status = 0;

    struct i2c_adapter *adap = atmel_mxt165->client->adapter;
    struct i2c_msg msgs[]= {
        [0] = {
            .addr = atmel_mxt165->client->addr,
            .flags = 0,
            .len = size+2,
            .buf = data,
        }
    };

    if (size <= 0)
        return status;

    data[0] = LSB(start);
    data[1] = MSB(start);

    memcpy(data+2, mem, size);
    TCH_DBG(DB_LEVEL2, "write mem, start = 0x%x, size = %d", start, size);

    status = (i2c_transfer(adap, msgs, 1) < 0) ? -EIO : 0;
    if (status != 0)
        TCH_ERR("I2C transfer failed.");

    return status;
}

// brief Reads the id part of info block.
// Reads the id part of the info block (7 bytes) from touch IC to info_block struct.
uint8_t read_id_block(info_id_t *id)
{
    uint8_t status = 0;
    uint8_t data[7];

    status = read_mem(0, 7, (void *) data);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read id information failed, status = %d", status);
        return status;
    }
    id->family_id            = data[0];
    id->variant_id           = data[1];
    id->version              = data[2];
    id->build                = data[3];
    id->matrix_x_size        = data[4];
    id->matrix_y_size        = data[5];
    id->num_declared_objects = data[6];

    TCH_DBG(DB_LEVEL1, "Done.");
    return status;
}

// brief Reads the object table of info block.
// Reads the object table part of the info block from touch IC to info_block struct.
uint8_t read_object_table(object_t *obj, unsigned int num)
{
    uint8_t status = 0, iLoop = 0, jLoop = 0, id = 0;
    uint8_t data[DEFAULT_REG_NUM][6];

    if (num > DEFAULT_REG_NUM)
    {
        TCH_ERR("Please increase default reg number, REG_NUM > %d",DEFAULT_REG_NUM);
        return READ_MEM_FAILED;
    }

    status = read_mem(7, num*6, (void *) data);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read object table information failed, status = %d", status);
        return status;
    }

    atmel_mxt165->first_finger_id = -1;
    for (iLoop=0; iLoop<num; iLoop++)
    {
        obj->object_type = data[iLoop][0];
        obj->i2c_address = ((uint16_t) data[iLoop][2] << 8) | data[iLoop][1];
        obj->size        = data[iLoop][3];
        obj->instances   = data[iLoop][4];
        obj->num_report_ids = data[iLoop][5];

        // Save the address to global variable
        if (obj->object_type < DEFAULT_REG_NUM)
        {
            g_RegAddr[obj->object_type] = obj->i2c_address;
        }

        // Save the Report ID information
        for (jLoop=0; jLoop<obj->num_report_ids; jLoop++)
        {
           id++;
           g_ReportID[id] = obj->object_type;

           // Save the first touch ID
           if ((obj->object_type == TOUCH_MULTITOUCHSCREEN_T9) && (atmel_mxt165->first_finger_id == -1))
               atmel_mxt165->first_finger_id = id;
        }

        obj++;
    }

    TCH_DBG(DB_LEVEL1, "Done.");
    return status;
}

uint32_t MXT_CRCSoft24(uint32_t crc, uint8_t FirstByte, uint8_t SecondByte)
{
    uint32_t crcPoly;
    uint32_t Result;
    uint16_t WData;

    crcPoly = 0x80001b;

    WData = (uint16_t) ((uint16_t)(SecondByte << 8) | FirstByte);

    Result = ((crc << 1) ^ ((uint32_t) WData));
    if (Result & 0x1000000)
    {
        Result ^= crcPoly;
    }

    return Result;
}

uint32_t MXT_CheckCRC(void)
{
    uint8_t crc_data[256];
    uint8_t iLoop = 0;
    uint32_t CRC = 0;
    object_t *obj_index;

    crc_data[0] = info_block->info_id->family_id;
    crc_data[1] = info_block->info_id->variant_id;
    crc_data[2] = info_block->info_id->version;
    crc_data[3] = info_block->info_id->build;
    crc_data[4] = info_block->info_id->matrix_x_size;
    crc_data[5] = info_block->info_id->matrix_y_size;
    crc_data[6] = info_block->info_id->num_declared_objects;

    obj_index = info_block->objects;
    for (iLoop=0; iLoop<info_block->info_id->num_declared_objects; iLoop++)
    {
        crc_data[iLoop*6+7+0] = obj_index->object_type;
        crc_data[iLoop*6+7+1] = obj_index->i2c_address & 0xFF;
        crc_data[iLoop*6+7+2] = obj_index->i2c_address >> 8;
        crc_data[iLoop*6+7+3] = obj_index->size;
        crc_data[iLoop*6+7+4] = obj_index->instances;
        crc_data[iLoop*6+7+5] = obj_index->num_report_ids;

        obj_index++;
    }
    crc_data[iLoop*6+7] = 0x0;

    for (iLoop=0; iLoop<(info_block->info_id->num_declared_objects*3+4); iLoop++)
    {
        CRC = MXT_CRCSoft24(CRC, crc_data[iLoop*2], crc_data[iLoop*2+1]);
    }
    CRC = CRC & 0x00FFFFFF;

    TCH_DBG(DB_LEVEL1, "Done. CRC = 0x%x", CRC);
    return CRC;
}

void MXT_GetConfigStatus(void)
{
    uint16_t addr;
    uint8_t data = 0x01; // Nonzero value
    uint8_t msg_data[8];
    uint8_t i;

    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 3;  // Cal Report all field address.
                                                    // T6 address + 3
    write_mem(addr, 1, &data);  // write one nonzero value;
    mdelay(5);

    addr = g_RegAddr[GEN_MESSAGEPROCESSOR_T5];
    while (1)
    {
        read_mem(addr, 8, msg_data);
        TCH_DBG(DB_LEVEL1, "Message Address = 0x%x", addr);

        for (i=0; i<8; i++)
        {
            TCH_DBG(DB_LEVEL1, "Message Data = 0x%x", msg_data[i]);
        }

        if ((g_ReportID[msg_data[0]] == GEN_COMMANDPROCESSOR_T6) ||
            (g_ReportID[msg_data[0]] == 0xFF) ||
            (g_ReportID[msg_data[0]] == 0x00))
            break;
    }

    TCH_DBG(DB_LEVEL1, "Done.");
}

void MXT_BackupNV(void)
{
    uint16_t addr   = 0;
    uint8_t data    = 0x55; 
    //g_NVBackUp = 1;
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 1;  // Call the BackNV field, command processor + 1
    write_mem(addr, 1, &data);
    msleep(10);
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 0;  // Call the Reset field, command processor + 0
    write_mem(addr, 1, &data);
    msleep(100);
    TCH_DBG(DB_LEVEL0, "Reset Done.");
    data = 0x00;
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 1;  
    write_mem(addr, 1, &data);
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 0;  
    write_mem(addr, 1, &data);
    //g_NVBackUp = 0;
    TCH_DBG(DB_LEVEL0, "Done.");
}

void MXT_Calibrate(void)
{
    uint16_t addr = 0;
    uint8_t data = 0x55; // Nonzero value

    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 2;  // Call the calibrate field, command processor + 2
    write_mem(addr, 1, &data);  // Write one nonzero value.

    TCH_DBG(DB_LEVEL1, "Done.");
}

void MXT_InitConfig(void)
{
    uint8_t v16_T7[]  = { 32, 16, 50 };
    uint8_t v16_T8[]  = { 9, 0, 5, 5, 0, 0, 15, 30 };
    uint8_t v16_T9[]  = { 143, 0, 0, 15, 11, 0, 32, 50, 2, 3, 0, 5, 5, 32, 5, 10, 30, 5, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0, 20 };	//Div2-D5-Peripheral-FG-TouchFineTuneConfig-00*	//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*
    uint8_t v16_T15[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t v16_T18[] = { 0, 0, };
    uint8_t v16_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t v16_T20[] = { 0, 0, 0, 5, 5, 0, 0, 30, 20, 4, 15, 5 };
    uint8_t v16_T22[] = { 7, 0, 0, 0x0, 0x0, 0x0, 0x0, 0, 30, 0, 0, 0, 10, 15, 20, 30, 0 };
    uint8_t v16_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0x0, 0x0, 0, 0, 0x0, 0x0 };
    uint8_t v16_T24[] = { 0, 0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0x0, 0, 0x0, 0, 0x0, 0, 0x0, 0 };
    uint8_t v16_T25[] = { 0, 0, 0xF8, 0x2A, 0x70, 0x17, 0x28, 0x23, 0x88, 0x13, 0x0, 0x0, 0x0, 0x0 };
    uint8_t v16_T27[] = { 0, 0, 0, 0, 0, 0x0, 0x0 };
    uint8_t v16_T28[] = { 0, 0, 0, 16, 16, 30 };

    uint8_t v16_T9_phase2[]  = { 143, 0, 0, 16, 10, 0, 32, 50, 2, 3, 0, 5, 5, 32, 5, 10, 30, 5, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0, 20 };	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+	//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*
    uint8_t v16_T15_phase2[] = { 131, 6, 10, 4, 1, 1, 16, 50, 2, 0, 0 };	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+

    uint8_t iLoop = 0, MaxOBJSize = 0;
    uint16_t addr;
    object_t *obj_index;

    MaxOBJSize = info_block->info_id->num_declared_objects;
    obj_index = info_block->objects;
    cap_touch_fw_version = info_block->info_id->version;

    //if (info_block->info_id->version == 16)
    {
        TCH_MSG("Use 1.6 firmware config.");
        for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
        {
            addr = obj_index->i2c_address;
            switch(obj_index->object_type)
            {
            case GEN_POWERCONFIG_T7 :
                write_mem(addr, sizeof(v16_T7), v16_T7);
                break;
            case GEN_ACQUISITIONCONFIG_T8 :
                write_mem(addr, sizeof(v16_T8), v16_T8);
                break;
            case TOUCH_MULTITOUCHSCREEN_T9 :
                //Div2-D5-Peripheral-FG-TouchAddNewPhase-00*[
                if (atmel_mxt165->dynamic.TS_PHASE == TOUCH_PHASE1)
                    write_mem(addr, sizeof(v16_T9), v16_T9);
                else
                    write_mem(addr, sizeof(v16_T9_phase2), v16_T9_phase2);
                //Div2-D5-Peripheral-FG-TouchAddNewPhase-00*[
                break;
            case TOUCH_KEYARRAY_T15 :
                //Div2-D5-Peripheral-FG-TouchAddNewPhase-00*[
                if (atmel_mxt165->dynamic.TS_PHASE == TOUCH_PHASE1)
                    write_mem(addr, sizeof(v16_T15), v16_T15);
                else
                    write_mem(addr, sizeof(v16_T15_phase2), v16_T15_phase2);
                //Div2-D5-Peripheral-FG-TouchAddNewPhase-00*]
                break;
            case SPT_COMCONFIG_T18 :
                write_mem(addr, sizeof(v16_T18), v16_T18);
                break;
            case SPT_GPIOPWM_T19 :
                write_mem(addr, sizeof(v16_T19), v16_T19);
                break;
            case PROCI_GRIPFACESUPPRESSION_T20 :
                write_mem(addr, sizeof(v16_T20), v16_T20);
                break;
            case PROCG_NOISESUPPRESSION_T22 :
                write_mem(addr, sizeof(v16_T22), v16_T22);
                break;
            case TOUCH_PROXIMITY_T23 :
                write_mem(addr, sizeof(v16_T23), v16_T23);
                break;
            case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                write_mem(addr, sizeof(v16_T24), v16_T24);
                break;
            case SPT_SELFTEST_T25 :
                write_mem(addr, sizeof(v16_T25), v16_T25);
                break;
            case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
                write_mem(addr, sizeof(v16_T27), v16_T27);
                break;
            case SPT_CTECONFIG_T28 :
                write_mem(addr, sizeof(v16_T28), v16_T28);
                break;
            default:
                break;
            }
            obj_index++;
        }
    }

    MXT_Calibrate();
    TCH_DBG(DB_LEVEL1, "Done.");
}

static void atmel_mxt165_reset(void)
{
    int i;

    if (gpio_get_value(GPIO_TP_RST_N) == LOW)
        gpio_set_value(GPIO_TP_RST_N, HIGH);
    gpio_set_value(GPIO_TP_RST_N, LOW);
    mdelay(10);
    gpio_set_value(GPIO_TP_RST_N, HIGH);
    for (i=0; i<10; i++)
    {
        mdelay(20);
        if (gpio_get_value(GPIO_TP_INT_N) == LOW)
            return ;
    }

    TCH_ERR("Failed.");
}

//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+[
static void atmel_mxt165_dynamicalloc(void)
{
    int product_id = fih_get_product_id();
    int product_phase = fih_get_product_phase();

    if (product_id == Product_SF8 && product_phase <= Product_PR1p5)
    {
        atmel_mxt165->dynamic.TS_PHASE = TOUCH_PHASE1;
        atmel_mxt165->dynamic.TS_MAX_Y = 975;
    }
    else
    {
        atmel_mxt165->dynamic.TS_PHASE = TOUCH_PHASE2;
        atmel_mxt165->dynamic.TS_MAX_Y = 1023;
    }
    TCH_MSG("fih_get_product_id = %d, fih_get_product_phase = %d.", product_id, product_phase);
    TCH_MSG("Phase = %d", atmel_mxt165->dynamic.TS_PHASE);
}
//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+]

uint8_t atmel_mxt165_check(uint8_t I2C_address)
{
    uint8_t iLoop = 0, status = 0;
    info_id_t *id;
    object_t *obj;
    uint8_t crc24[3];

    // Read the info block data.
    id = kmalloc(sizeof(info_id_t), GFP_KERNEL);
    if (id == NULL)
    {
        return DRIVER_SETUP_INCOMPLETE;
    }

    if (read_id_block(id) != READ_MEM_OK)
    {
        TCH_ERR("read_id_block(id) != READ_MEM_OK");
        kfree(id);
        return DRIVER_SETUP_INCOMPLETE;
    }
    info_block->info_id = id;

    TCH_MSG("Family ID       = 0x%x", info_block->info_id->family_id);
    TCH_MSG("Variant ID      = 0x%x", info_block->info_id->variant_id);
    TCH_MSG("Version         = 0x%x", info_block->info_id->version);
    TCH_MSG("Build           = 0x%x", info_block->info_id->build);
    TCH_MSG("Matrix X size   = 0x%x", info_block->info_id->matrix_x_size);
    TCH_MSG("Matrix Y size   = 0x%x", info_block->info_id->matrix_y_size);
    TCH_MSG("Number of Table = 0x%x", info_block->info_id->num_declared_objects);

    // Read the Object Table data.
    obj = kmalloc(sizeof(object_t) * info_block->info_id->num_declared_objects, GFP_KERNEL);
    if (obj == NULL)
    {
        goto ERR_1;
    }

    if (read_object_table(obj, id->num_declared_objects) != READ_MEM_OK)
    {
        TCH_ERR("read_object_table(obj) != READ_MEM_OK");
        goto ERR_2;
    }
    info_block->objects = obj;

    for (iLoop=0; iLoop<info_block->info_id->num_declared_objects; iLoop++)
    {
        TCH_DBG(DB_LEVEL2, "type       = 0x%x", obj->object_type);
        TCH_DBG(DB_LEVEL2, "address    = 0x%x", obj->i2c_address);
        TCH_DBG(DB_LEVEL2, "size       = 0x%x", obj->size);
        TCH_DBG(DB_LEVEL2, "instances  = 0x%x", obj->instances);
        TCH_DBG(DB_LEVEL2, "report ids = 0x%x", obj->num_report_ids);
        TCH_DBG(DB_LEVEL2, "************************************************************");

        obj++;
    }

    // Read the Checksum info
    status = read_mem(info_block->info_id->num_declared_objects*6+7, 3, crc24);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read CRC information failed, status = 0x%x", status);
        goto ERR_2;
    }
    info_block->CRC = (crc24[2] << 16) | (crc24[1] << 8) | crc24[0];
    TCH_DBG(DB_LEVEL1, "Internal CRC = 0x%x", info_block->CRC);

    if (info_block->CRC != MXT_CheckCRC())
    {
        TCH_ERR("Checksum failed.");
        goto ERR_2;
    }

    MXT_GetConfigStatus();
    MXT_InitConfig();
    MXT_BackupNV(); //Ming

    TCH_DBG(DB_LEVEL1, "Done.");
    return(DRIVER_SETUP_OK);

ERR_2:
   kfree(obj);
ERR_1:
   kfree(id);
   TCH_ERR("Failed.");
   return DRIVER_SETUP_INCOMPLETE;
}

static void atmel_mxt165_report(struct atmel_mxt165_info *ts, int i, int x, int y, int pressure)
{
    if (pressure == NO_TOUCH)
    {	// Finger Up.
        if (ts->points[i].first_area == PRESS_TOUCH_AREA)
        {	// Touch Area
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
#ifdef ATMEL_MXT165_MT
            input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
            input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
            input_report_abs(ts->touch_input, ABS_X, x);
            input_report_abs(ts->touch_input, ABS_Y, y);
            input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
            input_report_key(ts->touch_input, BTN_TOUCH, 0);
#endif
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]
        }
        else if (ts->points[i].first_area == PRESS_KEY1_AREA)
        {	// Key1 Area
            input_report_key(ts->key_input, KEY_HOME, 0);
            TCH_MSG("POS%d Report HOME key Up.", i+1);
        }
        else if (ts->points[i].first_area == PRESS_KEY2_AREA)
        {	// Key2 Area
            input_report_key(ts->key_input, KEY_MENU, 0);
            TCH_MSG("POS%d Report MENU key Up.", i+1);
        }
        else if (ts->points[i].first_area == PRESS_KEY3_AREA)
        {	// Key3 Area
            input_report_key(ts->key_input, KEY_BACK, 0);
            TCH_MSG("POS%d Report BACK key Up.", i+1);
        }
        else if (ts->points[i].first_area == PRESS_KEY4_AREA)
        {	// Key4 Area
            input_report_key(ts->key_input, KEY_SEARCH, 0);
            TCH_MSG("POS%d Report SEARCH key Up.", i+1);
        }
        ts->points[i].num = 0;
        ts->points[i].first_area = NO_TOUCH;
        ts->points[i].last_area = NO_TOUCH;
    }
    else
    {	// Finger Down.
        if (y <= ts->dynamic.TS_MAX_Y)	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00*
        {	// Touch Area
            if (ts->points[i].num == 0)
                ts->points[i].first_area = PRESS_TOUCH_AREA;
            if (ts->points[i].first_area == PRESS_TOUCH_AREA)
            {
                //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
#ifdef ATMEL_MXT165_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_X, x);
                input_report_abs(ts->touch_input, ABS_Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_TOUCH, 1);
#endif
                //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]
            }
            else if (ts->points[i].first_area == PRESS_KEY1_AREA)
            {	// Flick from home key area to touch area.
                input_report_key(ts->key_input, KEY_HOME, 0);
                TCH_MSG("POS%d Report HOME key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY2_AREA)
            {	// Flick from menu key area to touch area.
                input_report_key(ts->key_input, KEY_MENU, 0);
                TCH_MSG("POS%d Report MENU key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY3_AREA)
            {	// Flick from back key area to touch area.
                input_report_key(ts->key_input, KEY_BACK, 0);
                TCH_MSG("POS%d Report BACK key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY4_AREA)
            {	// Flick from search key area to touch area.
                input_report_key(ts->key_input, KEY_SEARCH, 0);
                TCH_MSG("POS%d Report SEARCH key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            ts->points[i].last_area = PRESS_TOUCH_AREA;
        }
        else
        {	//Key Area
            if (x < TS_KEY1_X)
            {	// Key1 Area
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->key_input, KEY_HOME, 1);
                    TCH_MSG("POS%d Report HOME key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY1_AREA;
                }
                ts->points[i].last_area = PRESS_KEY1_AREA;
            }
            else if (x < TS_KEY2_X)
            {	// Key2 Area
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->key_input, KEY_MENU, 1);
                    TCH_MSG("POS%d Report MENU key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY2_AREA;
                }
                ts->points[i].last_area = PRESS_KEY2_AREA;
            }
            else if (x < TS_KEY3_X)
            {	// Key3 Area
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->key_input, KEY_BACK, 1);
                    TCH_MSG("POS%d Report BACK key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY3_AREA;
                }
                ts->points[i].last_area = PRESS_KEY3_AREA;
            }
            else
            {	// Key4 Area
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->key_input, KEY_SEARCH, 1);
                    TCH_MSG("POS%d Report SEARCH key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY4_AREA;
                }
                ts->points[i].last_area = PRESS_KEY4_AREA;
            }
        }
        ts->points[i].num++;
    }
    //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
#ifdef ATMEL_MXT165_MT
    input_mt_sync(ts->touch_input);
#else
    input_sync(ts->touch_input);
#endif
    //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]
}

//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
static void atmel_mxt165_report_up(struct atmel_mxt165_info *ts, uint8_t *data)
{
    int i;

    for (i=0; i<TS_MAX_POINTS; i++)
    {
        if (ts->points[i].last_area > NO_TOUCH)
        {
            atmel_mxt165_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
            TCH_MSG("POS%d Up, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
        }
    }
#ifdef ATMEL_MXT165_MT
    input_sync(ts->touch_input);
#endif
}
//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]

static void atmel_mxt165_process_none(struct atmel_mxt165_info *ts, uint8_t *data)
{
    int i;

    TCH_DBG(DB_LEVEL1, "Get other report ID = %d", g_ReportID[data[0]]);
    for (i=0; i<8; i++)
        TCH_DBG(DB_LEVEL1, "Report T%d[%d] = %d", g_ReportID[data[0]], i ,data[i]);

    TCH_DBG(DB_LEVEL1, "Done.");
}

static void atmel_mxt165_process_T6(struct atmel_mxt165_info *ts, uint8_t *data)
{
    if (data[1] & 0x04)
        TCH_ERR("I2C-Compatible Checksum Errors.");
    if (data[1] & 0x08)
        TCH_ERR("Configuration Errors.");
    if (data[1] & 0x10)
    {
        TCH_MSG("Calibrating.");
        atmel_mxt165_report_up(ts, data);	//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*
    }
    if (data[1] & 0x20)
        TCH_ERR("Signal Errors.");
    if (data[1] & 0x40)
        TCH_ERR("Overflow Errors.");
    if (data[1] & 0x80)
    {
        TCH_MSG("Reseting.");
        //Div2-D5-Peripheral-FG-TouchAddErrorHandle-00+[
        MXT_InitConfig();
        TCH_MSG("GPIO_TP_RST_N = %d", gpio_get_value(GPIO_TP_RST_N));
        TCH_MSG("GPIO_TP_INT_N = %d", gpio_get_value(GPIO_TP_INT_N));
        //Div2-D5-Peripheral-FG-TouchAddErrorHandle-00+]
    }
    if (data[1] & 0x04 || ((data[1] & 0x10) && !gpio_get_value(GPIO_TP_INT_N)) || data[1] & 0x20)
    {
        TCH_MSG("Prepare to reset touch IC. Please wait a moment...");
        atmel_mxt165_reset();
        MXT_GetConfigStatus();
        MXT_InitConfig();
        TCH_MSG("Finish to reset touch IC.");
        TCH_MSG("GPIO_TP_RST_N = %d", gpio_get_value(GPIO_TP_RST_N));
        TCH_MSG("GPIO_TP_INT_N = %d", gpio_get_value(GPIO_TP_INT_N));
    }
}

static void atmel_mxt165_process_T9(struct atmel_mxt165_info *ts, uint8_t *data)
{
    int first_touch_id = ts->first_finger_id;	//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*
    int i, id = (data[0] - first_touch_id);

    if (id < TS_MAX_POINTS)
    {
        ts->points[id].x = (data[2] << 2) | (data[4] >> 6);
        ts->points[id].y = (data[3] << 2) | ((data[4] >> 2) & 0x03);

        if (data[1] & 0xC0)
        {	// Finger Down.
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
            for (i=0; i<TS_MAX_POINTS; i++)
            {
                if (ts->points[i].last_area > NO_TOUCH || i == id)
                {
                    atmel_mxt165_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                    if (ts->points[i].num == 1)
                        TCH_DBG(DB_LEVEL0, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                    else
                        TCH_DBG(DB_LEVEL1, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                }
            }
#ifdef ATMEL_MXT165_MT
            input_sync(ts->touch_input);
#endif
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]
        }
        else if ((data[1] & 0x20) == 0x20)
        {	// Finger Up.
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*[
            for (i=0; i<TS_MAX_POINTS; i++)
            {
                if (ts->points[i].last_area > NO_TOUCH)
                {
                    if (i == id)
                    {
                        atmel_mxt165_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
                        TCH_MSG("POS%d Up, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                    }
                    else
                    {
                        atmel_mxt165_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                        if (ts->points[i].num == 1)
                            TCH_DBG(DB_LEVEL0, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                        else
                            TCH_DBG(DB_LEVEL1, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                    }
                }
            }
#ifdef ATMEL_MXT165_MT
            input_sync(ts->touch_input);
#endif
            for (i=0; i<TS_MAX_POINTS; i++)
            {
                if (ts->points[i].last_area > NO_TOUCH)
                {
                    atmel_mxt165_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                    if (ts->points[i].num == 1)
                        TCH_DBG(DB_LEVEL0, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                    else
                        TCH_DBG(DB_LEVEL1, "POS%d Down %d, Status = 0x%x, X = %3d, Y = %3d, ID = %d", i+1, ts->points[i].num, data[1], ts->points[i].x, ts->points[i].y, g_ReportID[data[0]]);
                }
            }
#ifdef ATMEL_MXT165_MT
            input_sync(ts->touch_input);
#endif
            //Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*]
        }
    }
}

static void atmel_mxt165_process_T15(struct atmel_mxt165_info *ts, uint8_t *data)
{
    // Key1
    if (data[2] & 0x01)
    {
        if (!ts->key_state[0])
        {
            input_report_key(ts->key_input, KEY_HOME, 1);
            TCH_MSG("Report HOME key Down.");
            ts->key_state[0] = TRUE;
        }
    }
    else
    {
        if (ts->key_state[0])
        {
            input_report_key(ts->key_input, KEY_HOME, 0);
            TCH_MSG("Report HOME key Up.");
            ts->key_state[0] = FALSE;
        }
    }
    // Key2
    if (data[2] & 0x02)
    {
        if (!ts->key_state[1])
        {
            input_report_key(ts->key_input, KEY_MENU, 1);
            TCH_MSG("Report MENU key Down.");
            ts->key_state[1] = TRUE;
        }
    }
    else
    {
        if (ts->key_state[1])
        {
            input_report_key(ts->key_input, KEY_MENU, 0);
            TCH_MSG("Report MENU key Up.");
            ts->key_state[1] = FALSE;
        }
    }
    // Key3
    if (data[2] & 0x04)
    {
        if (!ts->key_state[2])
        {
            input_report_key(ts->key_input, KEY_BACK, 1);
            TCH_MSG("Report BACK key Down.");
            ts->key_state[2] = TRUE;
        }
    }
    else
    {
        if (ts->key_state[2])
        {
            input_report_key(ts->key_input, KEY_BACK, 0);
            TCH_MSG("Report BACK key Up.");
            ts->key_state[2] = FALSE;
        }
    }
    // Key4
    if (data[2] & 0x08)
    {
        if (!ts->key_state[3])
        {
            input_report_key(ts->key_input, KEY_SEARCH, 1);
            TCH_MSG("Report SEARCH key Down.");
            ts->key_state[3] = TRUE;
        }
    }
    else
    {
        if (ts->key_state[3])
        {
            input_report_key(ts->key_input, KEY_SEARCH, 0);
            TCH_MSG("Report SEARCH key Up.");
            ts->key_state[3] = FALSE;
        }
    }
}

static void atmel_mxt165_process_T20(struct atmel_mxt165_info *ts, uint8_t *data)
{
    if (data[1] & 0x01)
    {
        TCH_MSG("Face Down, Status = 0x%x, ID = %d", data[1], g_ReportID[data[0]]);
        ts->fake_proximity = TRUE;
    }
    else if (ts->fake_proximity)
    {
        TCH_MSG("Face Up, Status = 0x%x, ID = %d", data[1], g_ReportID[data[0]]);
        ts->fake_proximity = FALSE;
        atmel_mxt165_report_up(ts, data);	//Div2-D5-Peripheral-FG-TouchAddto5Multitouch-00*
    }
}

static void atmel_mxt165_isr_workqueue(struct work_struct *work)
{
    struct atmel_mxt165_info *ts = container_of(work, struct atmel_mxt165_info, work_queue);
    uint8_t count = 3, MSG_TYPE = 0;

    g_I2CAddress = g_RegAddr[GEN_MESSAGEPROCESSOR_T5];

    // If read/write I2C data faield, re-action 3 times.
    while (read_mem(g_I2CAddress , 8, (void *) g_MsgData ) != 0)
    {
        TCH_ERR("Read data failed, Re-read.");
        mdelay(3);

        count--;
        if (count == 0)
        {
            TCH_MSG("Can't Read/write data, reset chip.");
            TCH_MSG("GPIO_TP_RST_N = %d", gpio_get_value(GPIO_TP_RST_N));
            TCH_MSG("GPIO_TP_INT_N = %d", gpio_get_value(GPIO_TP_INT_N));
            atmel_mxt165_reset();  // Re-try 3 times, can't read/write data, reset atmel_mxt165 chip
            MXT_GetConfigStatus();
            MXT_InitConfig();
            return ;
        }
    }

    MSG_TYPE = g_ReportID[g_MsgData[0]];
    switch (MSG_TYPE)
    {
        case GEN_COMMANDPROCESSOR_T6:
            atmel_mxt165_process_T6(ts, g_MsgData);
            break;
        case TOUCH_MULTITOUCHSCREEN_T9:
            atmel_mxt165_process_T9(ts, g_MsgData);
            break;
        case TOUCH_KEYARRAY_T15:
            atmel_mxt165_process_T15(ts, g_MsgData);
            break;
        case PROCI_GRIPFACESUPPRESSION_T20:
            atmel_mxt165_process_T20(ts, g_MsgData);
            break;
        //case GEN_COMMANDPROCESSOR_T6:
        case GEN_POWERCONFIG_T7:
        case GEN_ACQUISITIONCONFIG_T8:
        //case TOUCH_MULTITOUCHSCREEN_T9:
        case TOUCH_SINGLETOUCHSCREEN_T10:
        case TOUCH_XSLIDER_T11:
        case TOUCH_YSLIDER_T12:
        case TOUCH_XWHEEL_T13:
        case TOUCH_YWHEEL_T14:
        //case TOUCH_KEYARRAY_T15:
        case PROCG_SIGNALFILTER_T16:
        case PROCI_LINEARIZATIONTABLE_T17:
        case SPT_COMCONFIG_T18:
        case SPT_GPIOPWM_T19:
        //case PROCI_GRIPFACESUPPRESSION_T20:
        case RESERVED_T21:
        case PROCG_NOISESUPPRESSION_T22:
        case TOUCH_PROXIMITY_T23:
        case PROCI_ONETOUCHGESTUREPROCESSOR_T24:
        case SPT_SELFTEST_T25:
        case DEBUG_CTERANGE_T26:
        case PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
        case SPT_CTECONFIG_T28:
        case SPT_GPI_T29:
        case SPT_GATE_T30:
        case TOUCH_KEYSET_T31:
        case TOUCH_XSLIDERSET_T32:
        default:
            atmel_mxt165_process_none(ts, g_MsgData);
            break;
    }

}

static irqreturn_t atmel_mxt165_isr(int irq, void * handle)
{
    struct atmel_mxt165_info *ts = handle;

    if (!ts->suspend_state)
        schedule_work(&ts->work_queue);

    return IRQ_HANDLED;
}

static int atmel_mxt165_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct input_dev *touch_input;
    struct input_dev *key_input;
    struct vreg *device_vreg;
    int i, ret = 0;

    // Check I2C
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        TCH_ERR("Check I2C functionality failed.");
        ret = -EIO;
        goto err_check_functionality_failed;
    }

    // Allocate Kernel Memory
    atmel_mxt165 = kzalloc(sizeof(struct atmel_mxt165_info), GFP_KERNEL);
    if (atmel_mxt165 == NULL)
    {
        TCH_ERR("Allocate atmel_mxt165 kernel memory failed.");
        ret = -ENOMEM;
        goto err_alloc_atmel_mxt165_failed;
    }
    info_block = kzalloc(sizeof(info_block_t), GFP_KERNEL);
    if (info_block == NULL)
    {
        TCH_ERR("Allocate info_block kernel memory failed.");
        ret = -ENOMEM;
        goto err_alloc_info_block_failed;
    }

    // Power On
    device_vreg = vreg_get(0, TOUCH_DEVICE_VREG);
    if (!device_vreg) {
        TCH_ERR("Get vreg(%s) failed.", TOUCH_DEVICE_VREG);
        ret = -EIO;
        goto err_power_failed;
    }
    vreg_set_level(device_vreg, 3000);
    TCH_DBG(DB_LEVEL0, "Power status = %d", vreg_enable(device_vreg));

    // Request and Config GPIO
    if (gpio_request(GPIO_TP_INT_N, "TP_INT_N"))
        TCH_ERR("Request GPIO %d failed.", GPIO_TP_INT_N);
    if (gpio_request(GPIO_TP_RST_N, "TP_RST_N"))
        TCH_ERR("Request GPIO %d failed.", GPIO_TP_RST_N);
    gpio_tlmm_config(GPIO_CFG(GPIO_TP_INT_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TP_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(GPIO_TP_RST_N, HIGH);
    atmel_mxt165_reset();

    // Check Touch IC
    atmel_mxt165->client = client;
    i2c_set_clientdata(client, atmel_mxt165);
    atmel_mxt165->client->addr = TOUCH_DEVICE_I2C_ADDRESS;
    atmel_mxt165_dynamicalloc();	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00+
    for (i=0; i<5; i++)
    {
        if (atmel_mxt165_check(atmel_mxt165->client->addr) == DRIVER_SETUP_INCOMPLETE)
            TCH_ERR("Check touch IC failed %d times.", i+1);
        else
            break;
    }
    if (i==5)
        goto err_check_touch_IC_failed;

    // Allocate Input Device
    touch_input = input_allocate_device();
    if (touch_input == NULL)
    {
        TCH_ERR("Allocate touch input device failed.");
        ret = -ENODEV;
        goto err_touch_input_dev_alloc_failed;
    }
    key_input = input_allocate_device();
    if (key_input == NULL)
    {
        TCH_ERR("Allocate key input device failed.");
        ret = -ENODEV;
        goto err_key_input_dev_alloc_failed;
    }

    touch_input->name  = "atmel_mxt165_touch";
    touch_input->phys  = "atmel_mxt165/input0";
    set_bit(EV_ABS, touch_input->evbit);
    set_bit(EV_SYN, touch_input->evbit);
#ifndef ATMEL_MXT165_MT
    set_bit(EV_KEY, touch_input->evbit);
    set_bit(BTN_TOUCH, touch_input->keybit);
    set_bit(BTN_2, touch_input->keybit);
    input_set_abs_params(touch_input, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_Y, TS_MIN_Y, atmel_mxt165->dynamic.TS_MAX_Y, 0, 0);	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00*
    input_set_abs_params(touch_input, ABS_HAT0X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_HAT0Y, TS_MIN_Y, atmel_mxt165->dynamic.TS_MAX_Y, 0, 0);	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00*
    input_set_abs_params(touch_input, ABS_PRESSURE, 0, 255, 0, 0);
#else
    input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_Y, TS_MIN_Y, atmel_mxt165->dynamic.TS_MAX_Y, 0, 0);	//Div2-D5-Peripheral-FG-TouchAddNewPhase-00*
#endif

    key_input->name  = "atmel_mxt165_key";
    key_input->phys  = "atmel_mxt165/input1";
    set_bit(EV_KEY, key_input->evbit);
    set_bit(KEY_HOME, key_input->keybit);
    set_bit(KEY_MENU, key_input->keybit);
    set_bit(KEY_BACK, key_input->keybit);
    set_bit(KEY_SEARCH, key_input->keybit);

    // Register Input Device
    atmel_mxt165->touch_input = touch_input;
    if (input_register_device(atmel_mxt165->touch_input))
    {
        TCH_ERR("Register touch input device failed.");
        ret = -ENODEV;
        goto err_touch_input_register_device_failed;
    }
    atmel_mxt165->key_input = key_input;
    if (input_register_device(atmel_mxt165->key_input))
    {
        TCH_ERR("Register key input device failed.");
        ret = -ENODEV;
        goto err_key_input_register_device_failed;
    }

    // Init Work Queue and Register IRQ
    INIT_WORK(&atmel_mxt165->work_queue, atmel_mxt165_isr_workqueue);
    atmel_mxt165->client->irq = MSM_GPIO_TO_INT(GPIO_TP_INT_N);
    if (request_irq(atmel_mxt165->client->irq, atmel_mxt165_isr, IRQF_TRIGGER_FALLING, client->dev.driver->name, atmel_mxt165))
    {
        TCH_ERR("Request IRQ failed.");
        ret = -EBUSY;
        goto err_request_irq_failed;
    }

    // Register early_suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
    atmel_mxt165->early_suspended.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
    atmel_mxt165->early_suspended.suspend = atmel_mxt165_early_suspend;
    atmel_mxt165->early_suspended.resume = atmel_mxt165_late_resume;
    register_early_suspend(&atmel_mxt165->early_suspended);
#endif

    memset(atmel_mxt165->points, 0, sizeof(struct point_info));
    atmel_mxt165->fake_proximity = FALSE;
    atmel_mxt165->suspend_state = FALSE;
    atmel_mxt165->T7[0] = 32;
    atmel_mxt165->T7[1] = 12;
    atmel_mxt165->T7[2] = 50;
    if (gpio_get_value(GPIO_TP_INT_N) == LOW)
        atmel_mxt165_isr_workqueue(&atmel_mxt165->work_queue);

    TCH_DBG(DB_LEVEL0, "Done.");
    return 0;

    free_irq(atmel_mxt165->client->irq, atmel_mxt165);
err_request_irq_failed:
    cancel_work_sync(&atmel_mxt165->work_queue);
    kfree(&atmel_mxt165->work_queue);
    input_unregister_device(atmel_mxt165->key_input);
err_key_input_register_device_failed:
    input_unregister_device(atmel_mxt165->touch_input);
err_touch_input_register_device_failed:
    input_free_device(key_input);
err_key_input_dev_alloc_failed:
    input_free_device(touch_input);
err_touch_input_dev_alloc_failed:
err_check_touch_IC_failed:
    dev_set_drvdata(&client->dev, 0);
    vreg_disable(device_vreg);
err_power_failed:
    kfree(info_block);
err_alloc_info_block_failed:
    kfree(atmel_mxt165);
err_alloc_atmel_mxt165_failed:
err_check_functionality_failed:
    TCH_ERR("Failed.");
    return ret;
}

static int atmel_mxt165_remove(struct i2c_client * client)
{
    free_irq(atmel_mxt165->client->irq, atmel_mxt165);

    cancel_work_sync(&atmel_mxt165->work_queue);
    kfree(&atmel_mxt165->work_queue);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&atmel_mxt165->early_suspended);
#endif

    input_unregister_device(atmel_mxt165->touch_input);
    input_unregister_device(atmel_mxt165->key_input);

    kfree(atmel_mxt165->touch_input);
    kfree(atmel_mxt165->key_input);
    input_free_device(atmel_mxt165->touch_input);
    input_free_device(atmel_mxt165->key_input);

    kfree(atmel_mxt165);
    kfree(info_block->info_id);
    kfree(info_block);

    dev_set_drvdata(&client->dev, 0);
    TCH_DBG(DB_LEVEL1, "Done.");
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_mxt165_early_suspend(struct early_suspend *h)
{
    struct atmel_mxt165_info *ts = container_of(h, struct atmel_mxt165_info, early_suspended);
    uint16_t power_cfg_address = 0;
    uint8_t data[3];
    uint8_t rc = 0;

    //cancel_work_sync(&ts->work_queue);
    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    read_mem(power_cfg_address, 3, (void *) ts->T7);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;

    //MXT_GetConfigStatus();	// Clear all data, avoid intrrupt no resume.

    rc = write_mem(power_cfg_address, 3, (void *) data);
    if (rc != 0)
        TCH_ERR("Driver can't enter deep sleep mode [%d].", rc);
    else
        TCH_MSG("Enter deep sleep mode.");

    disable_irq(ts->client->irq);
    ts->suspend_state = TRUE;
    //gpio_tlmm_config(GPIO_CFG(GPIO_TP_INT_N, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TP_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(GPIO_TP_RST_N, HIGH);

    TCH_MSG("GPIO_TP_RST_N = %d", gpio_get_value(GPIO_TP_RST_N));
    TCH_MSG("GPIO_TP_INT_N = %d", gpio_get_value(GPIO_TP_INT_N));
    TCH_DBG(DB_LEVEL1, "Done.");
}

void atmel_mxt165_late_resume(struct early_suspend *h)
{
    struct atmel_mxt165_info *ts = container_of(h, struct atmel_mxt165_info, early_suspended);
    uint16_t power_cfg_address = 0;
    uint8_t rc = 0;

    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    //gpio_tlmm_config(GPIO_CFG(GPIO_TP_INT_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TP_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(GPIO_TP_RST_N, HIGH);
    ts->suspend_state = FALSE;
    enable_irq(ts->client->irq);

    rc = write_mem(power_cfg_address, 3, (void *) ts->T7);
    if (rc != 0)
        TCH_ERR("Driver can't return from deep sleep mode [%d].", rc);
    else
        TCH_MSG("Return from sleep mode.");

    //atmel_mxt165_reset();
    MXT_GetConfigStatus();
    //MXT_InitConfig();
    MXT_Calibrate();

    TCH_MSG("GPIO_TP_RST_N = %d", gpio_get_value(GPIO_TP_RST_N));
    TCH_MSG("GPIO_TP_INT_N = %d", gpio_get_value(GPIO_TP_INT_N));
    TCH_DBG(DB_LEVEL1, "Done.");
}
#endif

#ifdef CONFIG_PM
static int atmel_mxt165_suspend(struct device *dev)
{
    //struct st1332_data *ts = dev_get_drvdata(dev);
    TCH_DBG(DB_LEVEL1, "Done.");
    return 0;
}

static int atmel_mxt165_resume(struct device *dev)
{
    //struct st1332_data *ts = dev_get_drvdata(dev);
    TCH_DBG(DB_LEVEL1, "Done.");
    return 0;
}

static struct dev_pm_ops atmel_mxt165_pm_ops = {
    .suspend = atmel_mxt165_suspend,
    .resume  = atmel_mxt165_resume,
};
#else
static int atmel_mxt165_suspend(struct i2c_client *client, pm_message_t state)
{
    //struct st1332_data *ts = i2c_get_clientdata(client);
    TCH_DBG(DB_LEVEL1, "Done.");
    return 0;
}

static int atmel_mxt165_resume(struct i2c_client *client)
{
    //struct st1332_data *ts = i2c_get_clientdata(client);
    TCH_DBG(DB_LEVEL1, "Done.");
    return 0;
}
#endif

static const struct i2c_device_id atmel_mxt165_id[] = {
    { TOUCH_DEVICE_NAME, TOUCH_DEVICE_I2C_ADDRESS },
    { }
};
MODULE_DEVICE_TABLE(i2c, atmel_mxt165_id);

static struct i2c_driver atmel_mxt165_i2c_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = TOUCH_DEVICE_NAME,
#ifdef CONFIG_PM
        .pm     = &atmel_mxt165_pm_ops,
#endif
    },
    .id_table   = atmel_mxt165_id,
    .probe      = atmel_mxt165_probe,
    .remove     = atmel_mxt165_remove,
#ifndef CONFIG_PM
    .suspend    = atmel_mxt165_suspend,
    .resume     = atmel_mxt165_resume,
#endif
};

static int __init atmel_mxt165_init( void )
{
    TCH_DBG(DB_LEVEL1, "Done.");
    return i2c_add_driver(&atmel_mxt165_i2c_driver);
}

static void __exit atmel_mxt165_exit( void )
{
    TCH_DBG(DB_LEVEL1, "Done.");
    i2c_del_driver(&atmel_mxt165_i2c_driver);
}

module_init(atmel_mxt165_init);
module_exit(atmel_mxt165_exit);

MODULE_AUTHOR("FIH Div2 Dep5 Peripheral Team FromkerGu <fromkergu@fih-foxconn.com>");
MODULE_DESCRIPTION("ATMEL MXT165 Touch Screen Driver");
MODULE_VERSION("1.6");
MODULE_LICENSE("GPL");
