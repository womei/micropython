/*
 * uzlib  -  tiny deflate/inflate library (deflate, gzip, zlib)
 *
 * Copyright (c) 2003 by Joergen Ibsen / Jibz
 * All Rights Reserved
 * http://www.ibsensoftware.com/
 *
 * Copyright (c) 2014 by Paul Sokolovsky
 */

#ifndef TINF_H_INCLUDED
#define TINF_H_INCLUDED

#include <stdint.h>

/* calling convention */
#ifndef TINFCC
 #ifdef __WATCOMC__
  #define TINFCC __cdecl
 #else
  #define TINFCC
 #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TINF_OK             0
#define TINF_STREAM_CONTINUE 1
#define TINF_DATA_ERROR    (-3)
#define TINF_DEST_OVERFLOW (-4)

/* data structures */

typedef struct {
   unsigned short table[16];  /* table of code length counts */
   unsigned short trans[288]; /* code -> symbol translation table */
} TINF_TREE;

struct TINF_DATA;
typedef struct TINF_DATA {
   // consume source0 first, then source1
   const unsigned char *source0_cur;
   const unsigned char *source0_top;
   const unsigned char *source1_cur;
   const unsigned char *source1_top;

   unsigned int tag;
   unsigned int bitcount;

    /* Buffer start */
    unsigned char *destStart;
    /* Buffer total size */
    unsigned int destSize;
    /* Current pointer in buffer */
    unsigned char *dest;
    /* Remaining bytes in buffer */
    unsigned int destRemaining;
    /* Argument is the allocation size which didn't fit into buffer. Note that
       exact mimumum size to grow buffer by is lastAlloc - destRemaining. But
       growing by this exact size is ineficient, as the next allocation will
       fail again. */
    int (*destGrow)(struct TINF_DATA *data, unsigned int lastAlloc);

   TINF_TREE ltree; /* dynamic length/symbol tree */
   TINF_TREE dtree; /* dynamic distance tree */
} TINF_DATA;

#define TINF_MAX_BYTES_DYN_TREE (924)
#define TINF_WIN_SIZE (32768)

struct TINF_DATA_STREAM;
typedef struct TINF_DATA_STREAM {
    TINF_DATA base;

    int state_X;
    int state_bfinal;
    int source_finished;
    unsigned char source_buf[TINF_MAX_BYTES_DYN_TREE];

    unsigned int win_head, win_tail;
    unsigned char win_buf[TINF_WIN_SIZE];

    int (*stream_out)(struct TINF_DATA_STREAM *data, const unsigned char *buf, unsigned int len);

    void *user_data;
} TINF_DATA_STREAM;

/* low-level API */

/* Step 1: Allocate TINF_DATA structure */
/* Step 2: Set destStart, destSize, and destGrow fields */
/* Step 3: Set source field */
/* Step 4: Call tinf_uncompress_dyn() */
/* Step 5: In response to destGrow callback, update destStart and destSize fields */
/* Step 6: When tinf_uncompress_dyn() returns, buf.dest points to a byte past last uncompressed byte */

int TINFCC tinf_uncompress_dyn(TINF_DATA *d);
int TINFCC tinf_zlib_uncompress_dyn(TINF_DATA *d, unsigned int sourceLen);

/* high-level API */

void TINFCC tinf_init(void);

int TINFCC tinf_uncompress(void *dest, unsigned int *destLen,
                           const void *source, unsigned int sourceLen);

int TINFCC tinf_gzip_uncompress(void *dest, unsigned int *destLen,
                                const void *source, unsigned int sourceLen);

int TINFCC tinf_zlib_uncompress(void *dest, unsigned int *destLen,
                                const void *source, unsigned int sourceLen);

unsigned int TINFCC tinf_adler32(const void *data, unsigned int length);

unsigned int TINFCC tinf_crc32(const void *data, unsigned int length);

/* compression API */

void TINFCC tinf_compress(void *data, const uint8_t *src, unsigned slen);

/* stream decompress API */

int tinf_stream_uncompress_init(TINF_DATA_STREAM *d);
int tinf_stream_uncompress_part(TINF_DATA_STREAM *d, const void *src, unsigned int len, int last);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINF_H_INCLUDED */
