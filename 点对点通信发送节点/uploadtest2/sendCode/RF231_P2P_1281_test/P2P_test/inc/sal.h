/**
 * @file sal.h
 *
 * @brief Declarations for low-level security API
 *
 * This file contains declarations for the low-level security
 * API.
 *
 * $Id: sal.h 12288 2008-11-27 07:38:16Z sschneid $
 *
 */
/**
 *  @author
 *      Atmel Corporation: http://www.atmel.com
 *      Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> LICENSE.txt
 */

/* Prevent double inclusion */
#ifndef SAL_H
#define SAL_H

/* === Includes =========================================================== */

#include "hal.h"


/* === Macros ============================================================= */

#define AES_BLOCKSIZE               (16)    /* Size of AES blocks */
#define AES_KEYSIZE                 (16)    /* Size of AES key */

/* === Types ============================================================== */


/* === Externals ========================================================== */


/* === Prototypes ========================================================= */

#ifdef __cplusplus
extern "C" {
#endif

void sal_init(void);

void sal_aes_read(uint8_t *data);

void sal_aes_restart(void);

bool sal_aes_setup(uint8_t *key,uint8_t enc_mode,uint8_t dir);

void sal_aes_wrrd(uint8_t *idata, uint8_t *odata);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SAL_H */
/* EOF */
