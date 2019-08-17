/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "irq.h"
#include "i2cslave.h"

#if 1||MICROPY_HW_ENABLE_HW_I2CSLAVE

#define IRQ_PRI_I2C NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 13, 0)

typedef struct _machine_i2cslave_data_t {
    uint8_t first_rx;
    uint8_t addr;
    uint8_t len; // -1
    uint8_t *mem;
} machine_i2cslave_data_t;

STATIC machine_i2cslave_data_t i2cslave_data[4];

int i2c_slave_process_addr_match(i2c_slave_t *i2c, int rw) {
    size_t i2c_id = ((uintptr_t)i2c - I2C1_BASE) / (I2C2_BASE - I2C1_BASE);
    //printf("I2CSlave(%u).addr_match(rw=%d)\n", i2c_id, rw);
    i2cslave_data[i2c_id].first_rx = 1;
    return 0;
}
int i2c_slave_process_rx_byte(i2c_slave_t *i2c, uint8_t val) {
    size_t i2c_id = ((uintptr_t)i2c - I2C1_BASE) / (I2C2_BASE - I2C1_BASE);
    //printf("I2CSlave(%u).rx_byte(%u)\n", i2c_id, val);
    machine_i2cslave_data_t *data = &i2cslave_data[i2c_id];
    if (data->first_rx == 1) {
        data->first_rx = 0;
        data->addr = val % (data->len + 1);
    } else {
        data->mem[data->addr++] = val;
        if (data->addr >= data->len + 1) {
            data->addr = 0;
        }
    }
    return 0;
}
void i2c_slave_process_rx_end(i2c_slave_t *i2c) {
    //printf("RXE\n");
}
uint8_t i2c_slave_process_tx_byte(i2c_slave_t *i2c) {
    //printf("TXB\n");
    size_t i2c_id = ((uintptr_t)i2c - I2C1_BASE) / (I2C2_BASE - I2C1_BASE);
    machine_i2cslave_data_t *data = &i2cslave_data[i2c_id];
    uint8_t val = data->mem[data->addr++];
    if (data->addr >= data->len + 1) {
        data->addr = 0;
    }
    return val;
}
void i2c_slave_process_tx_end(i2c_slave_t *i2c) {
    //printf("TXE\n");
}

const mp_obj_type_t machine_i2cslave_type;

typedef struct _machine_i2cslave_obj_t {
    mp_obj_base_t base;
    I2C_TypeDef *i2c;
    int irqn;
    mp_hal_pin_obj_t scl;
    mp_hal_pin_obj_t sda;
} machine_i2cslave_obj_t;

STATIC const machine_i2cslave_obj_t machine_i2cslave_obj[] = {
    #if defined(MICROPY_HW_I2C1_SCL)
    {{&machine_i2cslave_type}, I2C1, I2C1_EV_IRQn, MICROPY_HW_I2C1_SCL, MICROPY_HW_I2C1_SDA},
    #else
    {{NULL}, NULL, 0, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_I2C2_SCL)
    {{&machine_i2cslave_type}, I2C2, I2C2_EV_IRQn, MICROPY_HW_I2C2_SCL, MICROPY_HW_I2C2_SDA},
    #else
    {{NULL}, NULL, 0, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_I2C3_SCL)
    {{&machine_i2cslave_type}, I2C3, I2C3_EV_IRQn, MICROPY_HW_I2C3_SCL, MICROPY_HW_I2C3_SDA},
    #else
    {{NULL}, NULL, 0, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_I2C4_SCL)
    {{&machine_i2cslave_type}, I2C4, I2C4_EV_IRQn, MICROPY_HW_I2C4_SCL, MICROPY_HW_I2C4_SDA},
    #else
    {{NULL}, NULL, 0, NULL, NULL},
    #endif
};

STATIC void machine_i2cslave_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2cslave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2CSlaveMem(%u, addr=%u)",
        self - &machine_i2cslave_obj[0] + 1,
        (self->i2c->OAR1 >> 1) & 0x7f);
}

mp_obj_t machine_i2cslave_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    // parse args
    enum { ARG_id, ARG_addr, ARG_mem, ARG_scl, ARG_sda, ARG_freq, ARG_timeout };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_mem, MP_ARG_REQUIRED | MP_ARG_OBJ },
        /*
        { MP_QSTR_scl, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sda, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_freq, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 400000} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1000} },
        */
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Work out i2c bus
    int i2c_id = 0;
    if (MP_OBJ_IS_STR(args[ARG_id].u_obj)) {
        const char *port = mp_obj_str_get_str(args[ARG_id].u_obj);
        if (0) {
        #ifdef MICROPY_HW_I2C1_NAME
        } else if (strcmp(port, MICROPY_HW_I2C1_NAME) == 0) {
            i2c_id = 1;
        #endif
        #ifdef MICROPY_HW_I2C2_NAME
        } else if (strcmp(port, MICROPY_HW_I2C2_NAME) == 0) {
            i2c_id = 2;
        #endif
        #ifdef MICROPY_HW_I2C3_NAME
        } else if (strcmp(port, MICROPY_HW_I2C3_NAME) == 0) {
            i2c_id = 3;
        #endif
        } else {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
                "I2C(%s) doesn't exist", port));
        }
    } else {
        i2c_id = mp_obj_get_int(args[ARG_id].u_obj);
        if (i2c_id < 1 || i2c_id > MP_ARRAY_SIZE(machine_i2cslave_obj)
            || machine_i2cslave_obj[i2c_id - 1].base.type == NULL) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
                "I2C(%d) doesn't exist", i2c_id));
        }
    }

    // Initialise data
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_mem].u_obj, &bufinfo, MP_BUFFER_RW);
    if (bufinfo.len == 0 || bufinfo.len > 256) {
        mp_raise_ValueError("mem must be between 1 and 256 bytes");
    }
    MP_STATE_PORT(pyb_i2cslave_mem)[i2c_id - 1] = args[ARG_mem].u_obj;
    machine_i2cslave_data_t *data = &i2cslave_data[i2c_id - 1];
    data->addr = 0;
    data->len = bufinfo.len - 1;
    data->mem = bufinfo.buf;

    // get static peripheral object
    machine_i2cslave_obj_t *self = (machine_i2cslave_obj_t*)&machine_i2cslave_obj[i2c_id - 1];

    // Initialise the I2C slave peripheral
    mp_hal_pin_config_alt(self->scl, MP_HAL_PIN_MODE_ALT_OPEN_DRAIN, MP_HAL_PIN_PULL_NONE, AF_FN_I2C, i2c_id);
    mp_hal_pin_config_alt(self->sda, MP_HAL_PIN_MODE_ALT_OPEN_DRAIN, MP_HAL_PIN_PULL_NONE, AF_FN_I2C, i2c_id);
    i2c_slave_init(self->i2c, self->irqn, IRQ_PRI_I2C, args[ARG_addr].u_int);

    return MP_OBJ_FROM_PTR(self);
}

const mp_obj_type_t machine_i2cslave_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2CSlaveMem,
    .print = machine_i2cslave_print,
    .make_new = machine_i2cslave_make_new,
    //.locals_dict = (mp_obj_dict_t*)&mp_machine_i2cslave_locals_dict,
};

#endif // MICROPY_HW_ENABLE_HW_I2CSLAVE
