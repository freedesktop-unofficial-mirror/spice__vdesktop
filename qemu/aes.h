/* Simple helpers to libgcrypt AES support
 *
 * Author: Eduardo Habkost <ehabkost@redhat.com>
 *
 * Copyright (c) 2009, Red Hat, Inc.
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef _QEMU_AES_H
#define _QEMU_AES_H

#include <gcrypt.h>

typedef struct _AES_CBC_CIPHER {
	gcry_cipher_hd_t cipher;
} AES_CBC_CIPHER;

static inline int
AES_CBC_init(AES_CBC_CIPHER *h)
{
    if (!gcry_check_version (GCRYPT_VERSION))
        return -1;

    if (gcry_cipher_open(&h->cipher, GCRY_CIPHER_AES, GCRY_CIPHER_MODE_CBC, 0))
        return -1;
    return 0;
}

static inline int
AES_CBC_set_key(const void *key, size_t keybits, AES_CBC_CIPHER *h)
{
    /* We get the key length in bits, not bytes, here */
    if ((keybits % 8) != 0)
        return -1;

    if (gcry_cipher_setkey(h->cipher, key, keybits/8))
        return -1;

    return 0;
}

static inline int
AES_CBC_encrypt(const void *inbuf, void *outbuf, size_t bufbytes,
                const AES_CBC_CIPHER *h, const unsigned char *iv, size_t ivbytes, int enc)
{
    if (gcry_cipher_reset(h->cipher))
        return -1;

    if (gcry_cipher_setiv(h->cipher, iv, ivbytes))
        return -1;

    if (enc) {
        if (gcry_cipher_encrypt(h->cipher, outbuf, bufbytes, inbuf, bufbytes))
            return -1;
    } else {
        if (gcry_cipher_decrypt(h->cipher, outbuf, bufbytes, inbuf, bufbytes))
            return -1;
    }

    return 0;
}

static inline void
AES_CBC_deinit(AES_CBC_CIPHER *h)
{
    gcry_cipher_close(h->cipher);
}


#endif /* _QEMU_AES_H */
