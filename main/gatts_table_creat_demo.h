/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC_1,
    IDX_CHAR_A_1,
    IDX_CHAR_VAL_A_1,

    IDX_CHAR_B_1,
    IDX_CHAR_VAL_B_1,

    IDX_CHAR_C_1,
    IDX_CHAR_VAL_C_1,

    IDX_CHAR_D_1,
    IDX_CHAR_VAL_D_1,

    IDX_CHAR_E_1,
    IDX_CHAR_VAL_E_1,

    HRS_IDX_NB_1,
};

enum
{
    IDX_SVC_2,
    IDX_CHAR_A_2,
    IDX_CHAR_VAL_A_2,
    IDX_CHAR_CFG_A_2,
    IDX_CHAR_CFG_VAL_A_2,
    IDX_CHAR_CFG_USER_A_2,
    IDX_CHAR_CFG_USER_VAL_A_2,

    IDX_CHAR_B_2,
    IDX_CHAR_VAL_B_2,
    IDX_CHAR_CFG_B_2,
    IDX_CHAR_CFG_VAL_B_2,
    IDX_CHAR_CFG_USER_B_2,
    IDX_CHAR_CFG_USER_VAL_B_2,

    IDX_CHAR_C_2,
    IDX_CHAR_VAL_C_2,
    IDX_CHAR_CFG_C_2,
    IDX_CHAR_CFG_VAL_C_2,
    IDX_CHAR_CFG_USER_C_2,
    IDX_CHAR_CFG_USER_VAL_C_2,

    HRS_IDX_NB_2,
};

enum
{
    IDX_SVC_3,
    IDX_CHAR_A_3,
    IDX_CHAR_VAL_A_3,

    IDX_CHAR_B_3,
    IDX_CHAR_VAL_B_3,

    IDX_CHAR_C_3,
    IDX_CHAR_VAL_C_3,

    IDX_CHAR_D_3,
    IDX_CHAR_VAL_D_3,

    IDX_CHAR_E_3,
    IDX_CHAR_VAL_E_3,

    IDX_CHAR_F_3,
    IDX_CHAR_VAL_F_3,
    IDX_CHAR_CFG_F_3,
    IDX_CHAR_CFG_VAL_F_3,

    HRS_IDX_NB_3,
};