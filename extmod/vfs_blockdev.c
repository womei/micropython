/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2019 Damien P. George
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

#include "py/mpconfig.h"
#if MICROPY_VFS && MICROPY_VFS_FAT

#include <stdint.h>
#include <stdio.h>

#include "py/mphal.h"

#include "py/runtime.h"
#include "py/binary.h"
#include "py/objarray.h"
#include "extmod/vfs_fat.h"

int mp_vfs_blockdev_read(mp_vfs_blockdev_t *self, size_t block, size_t len, uint8_t *buf) {
    if (self->flags & MP_BLOCKDEV_FLAG_NATIVE) {
        int (*f)(void*, size_t, size_t, uint8_t*) = (void*)(uintptr_t)self->readblocks[2];
        return f(self, block, len, buf);
    } else {
        mp_obj_array_t ar = {{&mp_type_bytearray}, BYTEARRAY_TYPECODE, 0, len, buf};
        self->readblocks[2] = MP_OBJ_NEW_SMALL_INT(block);
        self->readblocks[3] = MP_OBJ_FROM_PTR(&ar);
        mp_call_method_n_kw(2, 0, self->readblocks);
        // TODO handle error return
        return 0;
    }
}

int mp_vfs_blockdev_write(mp_vfs_blockdev_t *self, size_t block, size_t len, const uint8_t *buf) {
    if (self->writeblocks[0] == MP_OBJ_NULL) {
        // read-only block device
        return -MP_ERO;
    }

    if (self->flags & MP_BLOCKDEV_FLAG_NATIVE) {
        int (*f)(void*, size_t, size_t, const uint8_t*) = (void*)(uintptr_t)self->writeblocks[2];
        return f(self, block, len, buf);
    } else {
        mp_obj_array_t ar = {{&mp_type_bytearray}, BYTEARRAY_TYPECODE, 0, len, buf};
        self->writeblocks[2] = MP_OBJ_NEW_SMALL_INT(block);
        self->writeblocks[3] = MP_OBJ_FROM_PTR(&ar);
        mp_call_method_n_kw(2, 0, self->writeblocks);
        // TODO handle error return
        return 0;
    }
}

int mp_vfs_blockdev_ioctl(mp_vfs_blockdev_t *self, uintptr_t cmd, void *data) {
    // First part: call the relevant method of the underlying block device
    mp_obj_t ret = mp_const_none;
    if (self->flags & MP_BLOCKDEV_FLAG_HAVE_IOCTL) {
        // new protocol with ioctl
        static const uint8_t op_map[8] = {
            [CTRL_SYNC] = BP_IOCTL_SYNC,
            [GET_SECTOR_COUNT] = BP_IOCTL_SEC_COUNT,
            [GET_SECTOR_SIZE] = BP_IOCTL_SEC_SIZE,
            [IOCTL_INIT] = BP_IOCTL_INIT,
        };
        uint8_t bp_op = op_map[cmd & 7];
        if (bp_op != 0) {
            self->u.ioctl[2] = MP_OBJ_NEW_SMALL_INT(bp_op);
            self->u.ioctl[3] = MP_OBJ_NEW_SMALL_INT(0); // unused
            ret = mp_call_method_n_kw(2, 0, self->u.ioctl);
        }
    } else {
        // old protocol with sync and count
        switch (cmd) {
            case CTRL_SYNC:
                if (self->u.old.sync[0] != MP_OBJ_NULL) {
                    mp_call_method_n_kw(0, 0, self->u.old.sync);
                }
                break;

            case GET_SECTOR_COUNT:
                ret = mp_call_method_n_kw(0, 0, self->u.old.count);
                break;

            case GET_SECTOR_SIZE:
                // old protocol has fixed sector size of 512 bytes
                break;

            case IOCTL_INIT:
                // old protocol doesn't have init
                break;
        }
    }

    // Second part: convert the result for return
    switch (cmd) {
        case CTRL_SYNC:
            return RES_OK;

        case GET_SECTOR_COUNT: {
            *((DWORD*)buff) = mp_obj_get_int(ret);
            return RES_OK;
        }

        case GET_SECTOR_SIZE: {
            if (ret == mp_const_none) {
                // Default sector size
                *((WORD*)buff) = 512;
            } else {
                *((WORD*)buff) = mp_obj_get_int(ret);
            }
            #if FF_MAX_SS != FF_MIN_SS
            // need to store ssize because we use it in disk_read/disk_write
            vfs->fatfs.ssize = *((WORD*)buff);
            #endif
            return RES_OK;
        }

        case GET_BLOCK_SIZE:
            *((DWORD*)buff) = 1; // erase block size in units of sector size
            return RES_OK;

        case IOCTL_INIT:
        case IOCTL_STATUS: {
            DSTATUS stat;
            if (ret != mp_const_none && MP_OBJ_SMALL_INT_VALUE(ret) != 0) {
                // error initialising
                stat = STA_NOINIT;
            } else if (self->writeblocks[0] == MP_OBJ_NULL) {
                stat = STA_PROTECT;
            } else {
                stat = 0;
            }
            *((DSTATUS*)buff) = stat;
            return RES_OK;
        }

        default:
            return RES_PARERR;
    }
}

#endif // MICROPY_VFS && MICROPY_VFS_FAT
