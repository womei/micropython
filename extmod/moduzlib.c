/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Paul Sokolovsky
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

#include "py/nlr.h"
#include "py/runtime.h"

#if MICROPY_PY_UZLIB

#include "uzlib/tinf.h"

#if 0 // print debugging info
#define DEBUG_printf DEBUG_printf
#else // don't print debugging info
#define DEBUG_printf(...) (void)0
#endif

/******************************************************************************/
// decompress function

STATIC int mod_uzlib_grow_buf(TINF_DATA *d, unsigned alloc_req) {
    if (alloc_req < 256) {
        alloc_req = 256;
    }
    DEBUG_printf("uzlib: Resizing buffer to " UINT_FMT " bytes\n", d->destSize + alloc_req);
    d->destStart = m_renew(byte, d->destStart, d->destSize, d->destSize + alloc_req);
    d->destSize += alloc_req;
    return 0;
}

STATIC mp_obj_t mod_uzlib_decompress(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    mp_obj_t data = args[0];
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data, &bufinfo, MP_BUFFER_READ);

    TINF_DATA *decomp = m_new_obj(TINF_DATA);
    DEBUG_printf("sizeof(TINF_DATA)=" UINT_FMT "\n", sizeof(*decomp));

    decomp->destSize = (bufinfo.len + 15) & ~15;
    decomp->destStart = m_new(byte, decomp->destSize);
    DEBUG_printf("uzlib: Initial out buffer: " UINT_FMT " bytes\n", decomp->destSize);
    decomp->destGrow = mod_uzlib_grow_buf;
    decomp->source0_cur = bufinfo.buf;
    decomp->source0_top = (const uint8_t*)bufinfo.buf + bufinfo.len;

    int st;
    if (n_args > 1 && MP_OBJ_SMALL_INT_VALUE(args[1]) < 0) {
        st = tinf_uncompress_dyn(decomp);
    } else {
        st = tinf_zlib_uncompress_dyn(decomp, bufinfo.len);
    }
    if (st != 0) {
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_ValueError, MP_OBJ_NEW_SMALL_INT(st)));
    }

    mp_uint_t final_sz = decomp->dest - decomp->destStart;
    DEBUG_printf("uzlib: Resizing from " UINT_FMT " to final size: " UINT_FMT " bytes\n", decomp->destSize, final_sz);
    decomp->destStart = (byte*)m_renew(byte, decomp->destStart, decomp->destSize, final_sz);
    mp_obj_t res = mp_obj_new_bytearray_by_ref(final_sz, decomp->destStart);
    m_del_obj(TINF_DATA, decomp);
    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_uzlib_decompress_obj, 1, 3, mod_uzlib_decompress);

/******************************************************************************/
// decompress object

typedef struct _mp_obj_decompress_t {
    mp_obj_base_t base;
    vstr_t vstr;
    TINF_DATA_STREAM decomp;
} mp_obj_decompress_t;

STATIC int stream_out(TINF_DATA_STREAM *d, const unsigned char *buf, unsigned int len) {
    mp_obj_decompress_t *self = (mp_obj_decompress_t*)d->user_data;
    //printf("out %p %d (on top of %d)\n", buf, len, (int)self->vstr.len);
    vstr_add_strn(&self->vstr, (const char*)buf, len);
    return 0;
}

STATIC mp_obj_t decompressobj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_obj_decompress_t *o = m_new_obj(mp_obj_decompress_t);
    o->base.type = type;
    if (n_args == 1 && mp_obj_get_int(args[0]) < 0) {
        // raw stream
    } else {
        // stream with zlib header; not implemented
        assert(0);
    }
    vstr_init(&o->vstr, 128);
    tinf_stream_uncompress_init(&o->decomp);
    o->decomp.stream_out = stream_out;
    o->decomp.user_data = o;
    //printf("state size = %d\n", (int)sizeof(mp_obj_decompress_t));
    return MP_OBJ_FROM_PTR(o);
}

STATIC mp_obj_t decompressobj_decompress(mp_obj_t self_in, mp_obj_t data_in) {
    mp_obj_decompress_t *self = (mp_obj_decompress_t*)self_in;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
    vstr_reset(&self->vstr);
    int res = tinf_stream_uncompress_part(&self->decomp, bufinfo.buf, bufinfo.len, false);
    if (res != TINF_OK && res != TINF_STREAM_CONTINUE) {
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_ValueError, MP_OBJ_NEW_SMALL_INT(res)));
    }
    return mp_obj_new_bytes((const byte*)self->vstr.buf, self->vstr.len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(decompressobj_decompress_obj, decompressobj_decompress);

STATIC mp_obj_t decompressobj_flush(mp_obj_t self_in) {
    mp_obj_decompress_t *self = (mp_obj_decompress_t*)self_in;
    vstr_reset(&self->vstr);
    int res = tinf_stream_uncompress_part(&self->decomp, NULL, 0, true);
    if (res != TINF_OK) {
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_ValueError, MP_OBJ_NEW_SMALL_INT(res)));
    }
    return mp_obj_new_bytes((const byte*)self->vstr.buf, self->vstr.len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(decompressobj_flush_obj, decompressobj_flush);

STATIC const mp_rom_map_elem_t decompressobj_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_decompress), MP_ROM_PTR(&decompressobj_decompress_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&decompressobj_flush_obj) },
};

STATIC MP_DEFINE_CONST_DICT(decompressobj_locals_dict, decompressobj_locals_dict_table);

STATIC const mp_obj_type_t decompressobj_type = {
    { &mp_type_type },
    .name = MP_QSTR_decompress, // CPython calls it "Decompress", but we save on a qstr
    .make_new = decompressobj_make_new,
    .locals_dict = (void*)&decompressobj_locals_dict,
};

/******************************************************************************/
// uzlib module

STATIC const mp_rom_map_elem_t mp_module_uzlib_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_uzlib) },
    { MP_ROM_QSTR(MP_QSTR_decompress), MP_ROM_PTR(&mod_uzlib_decompress_obj) },
    { MP_ROM_QSTR(MP_QSTR_decompressobj), MP_ROM_PTR(&decompressobj_type) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_uzlib_globals, mp_module_uzlib_globals_table);

const mp_obj_module_t mp_module_uzlib = {
    .base = { &mp_type_module },
    .name = MP_QSTR_uzlib,
    .globals = (mp_obj_dict_t*)&mp_module_uzlib_globals,
};

// Source files #include'd here to make sure they're compiled in
// only if module is enabled by config setting.

#include "uzlib/tinflate.c"
#include "uzlib/tinfzlib.c"
#include "uzlib/adler32.c"

#endif // MICROPY_PY_UZLIB
