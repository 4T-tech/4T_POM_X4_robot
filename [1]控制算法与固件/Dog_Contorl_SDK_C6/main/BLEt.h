#ifndef __BLETH_H__
#define __BLETH_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// In gatts_table_creat_demo.h
typedef enum {
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,
    HRS_IDX_NB, // 这个值会自动变为 4
} esp_gatts_attr_db_enum_t;

void BLE_Init(void);


#endif