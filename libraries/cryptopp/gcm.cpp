// gcm.cpp - originally written and placed in the public domain by Wei Dai.
//           ARM and Aarch64 added by Jeffrey Walton. The ARM carryless
//           multiply routines are less efficient because they shadow x86.
//           The precomputed key table integration makes it tricky to use the
//           more efficient ARMv8 implementation of the multiply and reduce.

// use "cl /EP /P /DCRYPTOPP_GENERATE_X64_MASM gcm.cpp" to generate MASM code

#include "pch.h"
#include "config.h"

#ifndef CRYPTOPP_IMPORTS
#ifndef CRYPTOPP_GENERATE_X64_MASM

// Visual Studio .Net 2003 compiler crash
#if defined(_MSC_VER) && (_MSC_VER < 1400)
# pragma optimize("", off)
#endif

#include "gcm.h"
#include "cpu.h"

#if defined(CRYPTOPP_DISABLE_GCM_ASM)
# undef CRYPTOPP_X86_ASM_AVAILABLE
# undef CRYPTOPP_X32_ASM_AVAILABLE
# undef CRYPTOPP_X64_ASM_AVAILABLE
# undef CRYPTOPP_SSE2_ASM_AVAILABLE
#endif

NAMESPACE_BEGIN(CryptoPP)

#if (CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64)
// Different assemblers accept different mnemonics: 'movd eax, xmm0' vs
//   'movd rax, xmm0' vs 'mov eax, xmm0' vs 'mov rax, xmm0'
#if defined(CRYPTOPP_DISABLE_MIXED_ASM)
// 'movd eax, xmm0' only. REG_WORD() macro not used. Clang path.
# define USE_MOVD_REG32 1
#elif defined(__GNUC__) || defined(_MSC_VER)
// 'movd eax, xmm0' or 'movd rax, xmm0'. REG_WORD() macro supplies REG32 or REG64.
# define USE_MOVD_REG32_OR_REG64 1
#else
// 'mov eax, xmm0' or 'mov rax, xmm0'. REG_WORD() macro supplies REG32 or REG64.
# define USE_MOV_REG32_OR_REG64 1
#endif
#endif  // CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64

// Clang intrinsic casts, http://bugs.llvm.org/show_bug.cgi?id=20670
#define M128_CAST(x) ((__m128i *)(void *)(x))
#define CONST_M128_CAST(x) ((const __m128i *)(const void *)(x))

word16 GCM_Base::s_reductionTable[256];
volatile bool GCM_Base::s_reductionTableInitialized = false;

void GCM_Base::GCTR::IncrementCounterBy256()
{
    IncrementCounterByOne(m_counterArray+BlockSize()-4, 3);
}

static inline void Xor16(byte *a, const byte *b, const byte *c)
{
    CRYPTOPP_ASSERT(IsAlignedOn(a,GetAlignmentOf<word64>()));
    CRYPTOPP_ASSERT(IsAlignedOn(b,GetAlignmentOf<word64>()));
    CRYPTOPP_ASSERT(IsAlignedOn(c,GetAlignmentOf<word64>()));
    ((word64 *)(void *)a)[0] = ((word64 *)(void *)b)[0] ^ ((word64 *)(void *)c)[0];
    ((word64 *)(void *)a)[1] = ((word64 *)(void *)b)[1] ^ ((word64 *)(void *)c)[1];
}

#if CRYPTOPP_SSE2_INTRIN_AVAILABLE || CRYPTOPP_SSE2_ASM_AVAILABLE
// SunCC 5.10-5.11 compiler crash. Move GCM_Xor16_SSE2 out-of-line, and place in
// a source file with a SSE architecture switch. Also see GH #226 and GH #284.
extern void GCM_Xor16_SSE2(byte *a, const byte *b, const byte *c);
#endif  // SSE2

#if CRYPTOPP_ARM_NEON_AVAILABLE
extern void GCM_Xor16_NEON(byte *a, const byte *b, const byte *c);
#endif

#if CRYPTOPP_POWER8_AVAILABLE
extern void GCM_Xor16_POWER8(byte *a, const byte *b, const byte *c);
#endif

#if CRYPTOPP_CLMUL_AVAILABLE
extern void GCM_SetKeyWithoutResync_CLMUL(const byte *hashKey, byte *mulTable, unsigned int tableSize);
extern size_t GCM_AuthenticateBlocks_CLMUL(const byte *data, size_t len, const byte *mtable, byte *hbuffer);
const unsigned int s_cltableSizeInBlocks = 8;
extern void GCM_ReverseHashBufferIfNeeded_CLMUL(byte *hashBuffer);
#endif  // CRYPTOPP_CLMUL_AVAILABLE

#if CRYPTOPP_ARM_PMULL_AVAILABLE
extern void GCM_SetKeyWithoutResync_PMULL(const byte *hashKey, byte *mulTable, unsigned int tableSize);
extern size_t GCM_AuthenticateBlocks_PMULL(const byte *data, size_t len, const byte *mtable, byte *hbuffer);
const unsigned int s_cltableSizeInBlocks = 8;
extern void GCM_ReverseHashBufferIfNeeded_PMULL(byte *hashBuffer);
#endif  // CRYPTOPP_ARM_PMULL_AVAILABLE

#if CRYPTOPP_POWER8_VMULL_AVAILABLE
extern void GCM_SetKeyWithoutResync_VMULL(const byte *hashKey, byte *mulTable, unsigned int tableSize);
extern size_t GCM_AuthenticateBlocks_VMULL(const byte *data, size_t len, const byte *mtable, byte *hbuffer);
const unsigned int s_cltableSizeInBlocks = 8;
extern void GCM_ReverseHashBufferIfNeeded_VMULL(byte *hashBuffer);
#endif  // CRYPTOPP_POWER8_VMULL_AVAILABLE

void GCM_Base::SetKeyWithoutResync(const byte *userKey, size_t keylength, const NameValuePairs &params)
{
    BlockCipher &blockCipher = AccessBlockCipher();
    blockCipher.SetKey(userKey, keylength, params);

    // GCM is only defined for 16-byte block ciphers at the moment.
    // However, variable blocksize support means we have to defer
    // blocksize checks to runtime after the key is set. Also see
    // https://github.com/weidai11/cryptopp/issues/408.
    const unsigned int blockSize = blockCipher.BlockSize();
    CRYPTOPP_ASSERT(blockSize == REQUIRED_BLOCKSIZE);
    if (blockCipher.BlockSize() != REQUIRED_BLOCKSIZE)
        throw InvalidArgument(AlgorithmName() + ": block size of underlying block cipher is not 16");

    int tableSize, i, j, k;

#if CRYPTOPP_CLMUL_AVAILABLE
    if (HasCLMUL())
    {
        // Avoid "parameter not used" error and suppress Coverity finding
        (void)params.GetIntValue(Name::TableSize(), tableSize);
        tableSize = s_cltableSizeInBlocks * blockSize;
        CRYPTOPP_ASSERT(tableSize > static_cast<int>(blockSize));
    }
    else
#elif CRYPTOPP_ARM_PMULL_AVAILABLE
    if (HasPMULL())
    {
        // Avoid "parameter not used" error and suppress Coverity finding
        (void)params.GetIntValue(Name::TableSize(), tableSize);
        tableSize = s_cltableSizeInBlocks * blockSize;
        CRYPTOPP_ASSERT(tableSize > static_cast<int>(blockSize));
    }
    else
#elif CRYPTOPP_POWER8_VMULL_AVAILABLE
    if (HasPMULL())
    {
        // Avoid "parameter not used" error and suppress Coverity finding
        (void)params.GetIntValue(Name::TableSize(), tableSize);
        tableSize = s_cltableSizeInBlocks * blockSize;
        CRYPTOPP_ASSERT(tableSize > static_cast<int>(blockSize));
    }
    else
#endif
    {
        if (params.GetIntValue(Name::TableSize(), tableSize))
            tableSize = (tableSize >= 64*1024) ? 64*1024 : 2*1024;
        else
            tableSize = (GetTablesOption() == GCM_64K_Tables) ? 64*1024 : 2*1024;

        //#if defined(_MSC_VER) && (_MSC_VER < 1400)
        // VC 2003 workaround: compiler generates bad code for 64K tables
        //tableSize = 2*1024;
        //#endif
    }

    m_buffer.resize(3*blockSize + tableSize);
    byte *mulTable = MulTable();
    byte *hashKey = HashKey();
    memset(hashKey, 0, REQUIRED_BLOCKSIZE);
    blockCipher.ProcessBlock(hashKey);

#if CRYPTOPP_CLMUL_AVAILABLE
    if (HasCLMUL())
    {
        GCM_SetKeyWithoutResync_CLMUL(hashKey, mulTable, tableSize);
        return;
    }
#elif CRYPTOPP_ARM_PMULL_AVAILABLE
    if (HasPMULL())
    {
        GCM_SetKeyWithoutResync_PMULL(hashKey, mulTable, tableSize);
        return;
    }
#elif CRYPTOPP_POWER8_VMULL_AVAILABLE
    if (HasPMULL())
    {
        GCM_SetKeyWithoutResync_VMULL(hashKey, mulTable, tableSize);
        return;
    }
#endif

    word64 V0, V1;
    typedef BlockGetAndPut<word64, BigEndian> Block;
    Block::Get(hashKey)(V0)(V1);

    if (tableSize == 64*1024)
    {
        for (i=0; i<128; i++)
        {
            k = i%8;
            Block::Put(NULLPTR, mulTable+(i/8)*256*16+(size_t(1)<<(11-k)))(V0)(V1);

            int x = (int)V1 & 1;
            V1 = (V1>>1) | (V0<<63);
            V0 = (V0>>1) ^ (x ? W64LIT(0xe1) << 56 : 0);
        }

        for (i=0; i<16; i++)
        {
            memset(mulTable+i*256*16, 0, 16);
#if CRYPTOPP_SSE2_INTRIN_AVAILABLE || CRYPTOPP_SSE2_ASM_AVAILABLE
            if (HasSSE2())
                for (j=2; j<=0x80; j*=2)
                    for (k=1; k<j; k++)
                        GCM_Xor16_SSE2(mulTable+i*256*16+(j+k)*16, mulTable+i*256*16+j*16, mulTable+i*256*16+k*16);
            else
#elif CRYPTOPP_ARM_NEON_AVAILABLE
            if (HasNEON())
                for (j=2; j<=0x80; j*=2)
                    for (k=1; k<j; k++)
                        GCM_Xor16_NEON(mulTable+i*256*16+(j+k)*16, mulTable+i*256*16+j*16, mulTable+i*256*16+k*16);
            else
#elif CRYPTOPP_POWER8_AVAILABLE
            if (HasPower8())
                for (j=2; j<=0x80; j*=2)
                    for (k=1; k<j; k++)
                        GCM_Xor16_POWER8(mulTable+i*256*16+(j+k)*16, mulTable+i*256*16+j*16, mulTable+i*256*16+k*16);
            else
#endif
                for (j=2; j<=0x80; j*=2)
                    for (k=1; k<j; k++)
                        Xor16(mulTable+i*256*16+(j+k)*16, mulTable+i*256*16+j*16, mulTable+i*256*16+k*16);
        }
    }
    else
    {
        if (!s_reductionTableInitialized)
        {
            s_reductionTable[0] = 0;
            word16 x = 0x01c2;
            s_reductionTable[1] = ByteReverse(x);
            for (unsigned int ii=2; ii<=0x80; ii*=2)
            {
                x <<= 1;
                s_reductionTable[ii] = ByteReverse(x);
                for (unsigned int jj=1; jj<ii; jj++)
                    s_reductionTable[ii+jj] = s_reductionTable[ii] ^ s_reductionTable[jj];
            }
            s_reductionTableInitialized = true;
        }

        for (i=0; i<128-24; i++)
        {
            k = i%32;
            if (k < 4)
                Block::Put(NULLPTR, mulTable+1024+(i/32)*256+(size_t(1)<<(7-k)))(V0)(V1);
            else if (k < 8)
                Block::Put(NULLPTR, mulTable+(i/32)*256+(size_t(1)<<(11-k)))(V0)(V1);

            int x = (int)V1 & 1;
            V1 = (V1>>1) | (V0<<63);
            V0 = (V0>>1) ^ (x ? W64LIT(0xe1) << 56 : 0);
        }

        for (i=0; i<4; i++)
        {
            memset(mulTable+i*256, 0, 16);
            memset(mulTable+1024+i*256, 0, 16);
#if CRYPTOPP_SSE2_INTRIN_AVAILABLE || CRYPTOPP_SSE2_ASM_AVAILABLE
            if (HasSSE2())
                for (j=2; j<=8; j*=2)
                    for (k=1; k<j; k++)
                    {
                        GCM_Xor16_SSE2(mulTable+i*256+(j+k)*16, mulTable+i*256+j*16, mulTable+i*256+k*16);
                        GCM_Xor16_SSE2(mulTable+1024+i*256+(j+k)*16, mulTable+1024+i*256+j*16, mulTable+1024+i*256+k*16);
                    }
            else
#elif CRYPTOPP_ARM_NEON_AVAILABLE
            if (HasNEON())
                for (j=2; j<=8; j*=2)
                    for (k=1; k<j; k++)
                    {
                        GCM_Xor16_NEON(mulTable+i*256+(j+k)*16, mulTable+i*256+j*16, mulTable+i*256+k*16);
                        GCM_Xor16_NEON(mulTable+1024+i*256+(j+k)*16, mulTable+1024+i*256+j*16, mulTable+1024+i*256+k*16);
                    }
            else
#elif CRYPTOPP_POWER8_AVAILABLE
            if (HasPower8())
                for (j=2; j<=8; j*=2)
                    for (k=1; k<j; k++)
                    {
                        GCM_Xor16_POWER8(mulTable+i*256+(j+k)*16, mulTable+i*256+j*16, mulTable+i*256+k*16);
                        GCM_Xor16_POWER8(mulTable+1024+i*256+(j+k)*16, mulTable+1024+i*256+j*16, mulTable+1024+i*256+k*16);
                    }
            else
#endif
                for (j=2; j<=8; j*=2)
                    for (k=1; k<j; k++)
                    {
                        Xor16(mulTable+i*256+(j+k)*16, mulTable+i*256+j*16, mulTable+i*256+k*16);
                        Xor16(mulTable+1024+i*256+(j+k)*16, mulTable+1024+i*256+j*16, mulTable+1024+i*256+k*16);
                    }
        }
    }
}

inline void GCM_Base::ReverseHashBufferIfNeeded()
{
#if CRYPTOPP_CLMUL_AVAILABLE
    if (HasCLMUL())
    {
        GCM_ReverseHashBufferIfNeeded_CLMUL(HashBuffer());
    }
#elif CRYPTOPP_ARM_PMULL_AVAILABLE
    if (HasPMULL())
    {
        GCM_ReverseHashBufferIfNeeded_PMULL(HashBuffer());
    }
#elif CRYPTOPP_POWER8_VMULL_AVAILABLE
    if (HasPMULL())
    {
        GCM_ReverseHashBufferIfNeeded_VMULL(HashBuffer());
    }
#endif
}

void GCM_Base::Resync(const byte *iv, size_t len)
{
    BlockCipher &cipher = AccessBlockCipher();
    byte *hashBuffer = HashBuffer();

    if (len == 12)
    {
        memcpy(hashBuffer, iv, len);
        memset(hashBuffer+len, 0, 3);
        hashBuffer[len+3] = 1;
    }
    else
    {
        size_t origLen = len;
        memset(hashBuffer, 0, HASH_BLOCKSIZE);

        if (len >= HASH_BLOCKSIZE)
        {
            len = GCM_Base::AuthenticateBlocks(iv, len);
            iv += (origLen - len);
        }

        if (len > 0)
        {
            memcpy(m_buffer, iv, len);
            memset(m_buffer+len, 0, HASH_BLOCKSIZE-len);
            GCM_Base::AuthenticateBlocks(m_buffer, HASH_BLOCKSIZE);
        }

        PutBlock<word64, BigEndian, true>(NULLPTR, m_buffer)(0)(origLen*8);
        GCM_Base::AuthenticateBlocks(m_buffer, HASH_BLOCKSIZE);

        ReverseHashBufferIfNeeded();
    }

    if (m_state >= State_IVSet)
        m_ctr.Resynchronize(hashBuffer, REQUIRED_BLOCKSIZE);
    else
        m_ctr.SetCipherWithIV(cipher, hashBuffer);

    m_ctr.Seek(HASH_BLOCKSIZE);

    memset(hashBuffer, 0, HASH_BLOCKSIZE);
}

unsigned int GCM_Base::OptimalDataAlignment() const
{
    return
#if CRYPTOPP_SSE2_ASM_AVAILABLE || defined(CRYPTOPP_X64_MASM_AVAILABLE)
        HasSSE2() ? 16 :
#elif CRYPTOPP_ARM_NEON_AVAILABLE
        HasNEON() ? 4 :
#elif CRYPTOPP_POWER8_AVAILABLE
        HasPower8() ? 16 :
#endif
        GetBlockCipher().OptimalDataAlignment();
}

#if CRYPTOPP_MSC_VERSION
# pragma warning(disable: 4731)    // frame pointer register 'ebp' modified by inline assembly code
#endif

#endif    // Not CRYPTOPP_GENERATE_X64_MASM

#ifdef CRYPTOPP_X64_MASM_AVAILABLE
extern "C" {
void GCM_AuthenticateBlocks_2K_SSE2(const byte *data, size_t blocks, word64 *hashBuffer, const word16 *reductionTable);
void GCM_AuthenticateBlocks_64K_SSE2(const byte *data, size_t blocks, word64 *hashBuffer);
}
#endif

#ifndef CRYPTOPP_GENERATE_X64_MASM

size_t GCM_Base::AuthenticateBlocks(const byte *data, size_t len)
{
#if CRYPTOPP_CLMUL_AVAILABLE
    if (HasCLMUL())
    {
        return GCM_AuthenticateBlocks_CLMUL(data, len, MulTable(), HashBuffer());
    }
#elif CRYPTOPP_ARM_PMULL_AVAILABLE
    if (HasPMULL())
    {
        return GCM_AuthenticateBlocks_PMULL(data, len, MulTable(), HashBuffer());
    }
#elif CRYPTOPP_POWER8_VMULL_AVAILABLE
    if (HasPMULL())
    {
        return GCM_AuthenticateBlocks_VMULL(data, len, MulTable(), HashBuffer());
    }
#endif

    typedef BlockGetAndPut<word64, NativeByteOrder> Block;
    word64 *hashBuffer = (word64 *)(void *)HashBuffer();
    CRYPTOPP_ASSERT(IsAlignedOn(hashBuffer,GetAlignmentOf<word64>()));

    switch (2*(m_buffer.size()>=64*1024)
#if CRYPTOPP_SSE2_ASM_AVAILABLE || defined(CRYPTOPP_X64_MASM_AVAILABLE)
        + HasSSE2()
//#elif CRYPTOPP_ARM_NEON_AVAILABLE
//      + HasNEON()
#endif
        )
    {
    case 0:        // non-SSE2 and 2K tables
        {
        byte *mulTable = MulTable();
        word64 x0 = hashBuffer[0], x1 = hashBuffer[1];

        do
        {
            word64 y0, y1, a0, a1, b0, b1, c0, c1, d0, d1;
            Block::Get(data)(y0)(y1);
            x0 ^= y0;
            x1 ^= y1;

            data += HASH_BLOCKSIZE;
            len -= HASH_BLOCKSIZE;

            #define READ_TABLE_WORD64_COMMON(a, b, c, d)    *(word64 *)(void *)(mulTable+(a*1024)+(b*256)+c+d*8)

            #if (CRYPTOPP_LITTLE_ENDIAN)
                #if CRYPTOPP_BOOL_SLOW_WORD64
                    word32 z0 = (word32)x0;
                    word32 z1 = (word32)(x0>>32);
                    word32 z2 = (word32)x1;
                    word32 z3 = (word32)(x1>>32);
                    #define READ_TABLE_WORD64(a, b, c, d, e)    READ_TABLE_WORD64_COMMON((d%2), c, (d?(z##c>>((d?d-1:0)*4))&0xf0:(z##c&0xf)<<4), e)
                #else
                    #define READ_TABLE_WORD64(a, b, c, d, e)    READ_TABLE_WORD64_COMMON((d%2), c, ((d+8*b)?(x##a>>(((d+8*b)?(d+8*b)-1:1)*4))&0xf0:(x##a&0xf)<<4), e)
                #endif
                #define GF_MOST_SIG_8BITS(a) (a##1 >> 7*8)
                #define GF_SHIFT_8(a) a##1 = (a##1 << 8) ^ (a##0 >> 7*8); a##0 <<= 8;
            #else
                #define READ_TABLE_WORD64(a, b, c, d, e)    READ_TABLE_WORD64_COMMON((1-d%2), c, ((15-d-8*b)?(x##a>>(((15-d-8*b)?(15-d-8*b)-1:0)*4))&0xf0:(x##a&0xf)<<4), e)
                #define GF_MOST_SIG_8BITS(a) (a##1 & 0xff)
                #define GF_SHIFT_8(a) a##1 = (a##1 >> 8) ^ (a##0 << 7*8); a##0 >>= 8;
            #endif

            #define GF_MUL_32BY128(op, a, b, c)                                            \
                a0 op READ_TABLE_WORD64(a, b, c, 0, 0) ^ READ_TABLE_WORD64(a, b, c, 1, 0); \
                a1 op READ_TABLE_WORD64(a, b, c, 0, 1) ^ READ_TABLE_WORD64(a, b, c, 1, 1); \
                b0 op READ_TABLE_WORD64(a, b, c, 2, 0) ^ READ_TABLE_WORD64(a, b, c, 3, 0); \
                b1 op READ_TABLE_WORD64(a, b, c, 2, 1) ^ READ_TABLE_WORD64(a, b, c, 3, 1); \
                c0 op READ_TABLE_WORD64(a, b, c, 4, 0) ^ READ_TABLE_WORD64(a, b, c, 5, 0); \
                c1 op READ_TABLE_WORD64(a, b, c, 4, 1) ^ READ_TABLE_WORD64(a, b, c, 5, 1); \
                d0 op READ_TABLE_WORD64(a, b, c, 6, 0) ^ READ_TABLE_WORD64(a, b, c, 7, 0); \
                d1 op READ_TABLE_WORD64(a, b, c, 6, 1) ^ READ_TABLE_WORD64(a, b, c, 7, 1); \

            GF_MUL_32BY128(=, 0, 0, 0)
            GF_MUL_32BY128(^=, 0, 1, 1)
            GF_MUL_32BY128(^=, 1, 0, 2)
            GF_MUL_32BY128(^=, 1, 1, 3)

            word32 r = (word32)s_reductionTable[GF_MOST_SIG_8BITS(d)] << 16;
            GF_SHIFT_8(d)
            c0 ^= d0; c1 ^= d1;
            r ^= (word32)s_reductionTable[GF_MOST_SIG_8BITS(c)] << 8;
            GF_SHIFT_8(c)
            b0 ^= c0; b1 ^= c1;
            r ^= s_reductionTable[GF_MOST_SIG_8BITS(b)];
            GF_SHIFT_8(b)
            a0 ^= b0; a1 ^= b1;
            a0 ^= ConditionalByteReverse<word64>(LITTLE_ENDIAN_ORDER, r);
            x0 = a0; x1 = a1;
        }
        while (len >= HASH_BLOCKSIZE);

        hashBuffer[0] = x0; hashBuffer[1] = x1;
        return len;
        }

    case 2:        // non-SSE2 and 64K tables
        {
        byte *mulTable = MulTable();
        word64 x0 = hashBuffer[0], x1 = hashBuffer[1];

        do
        {
            word64 y0, y1, a0, a1;
            Block::Get(data)(y0)(y1);
            x0 ^= y0;
            x1 ^= y1;

            data += HASH_BLOCKSIZE;
            len -= HASH_BLOCKSIZE;

            #undef READ_TABLE_WORD64_COMMON
            #undef READ_TABLE_WORD64

            #define READ_TABLE_WORD64_COMMON(a, c, d)    *(word64 *)(void *)(mulTable+(a)*256*16+(c)+(d)*8)

            #if (CRYPTOPP_LITTLE_ENDIAN)
                #if CRYPTOPP_BOOL_SLOW_WORD64
                    word32 z0 = (word32)x0;
                    word32 z1 = (word32)(x0>>32);
                    word32 z2 = (word32)x1;
                    word32 z3 = (word32)(x1>>32);
                    #define READ_TABLE_WORD64(b, c, d, e)    READ_TABLE_WORD64_COMMON(c*4+d, (d?(z##c>>((d?d:1)*8-4))&0xff0:(z##c&0xff)<<4), e)
                #else
                    #define READ_TABLE_WORD64(b, c, d, e)    READ_TABLE_WORD64_COMMON(c*4+d, ((d+4*(c%2))?(x##b>>(((d+4*(c%2))?(d+4*(c%2)):1)*8-4))&0xff0:(x##b&0xff)<<4), e)
                #endif
            #else
                #define READ_TABLE_WORD64(b, c, d, e)    READ_TABLE_WORD64_COMMON(c*4+d, ((7-d-4*(c%2))?(x##b>>(((7-d-4*(c%2))?(7-d-4*(c%2)):1)*8-4))&0xff0:(x##b&0xff)<<4), e)
            #endif

            #define GF_MUL_8BY128(op, b, c, d)        \
                a0 op READ_TABLE_WORD64(b, c, d, 0);\
                a1 op READ_TABLE_WORD64(b, c, d, 1);\

            GF_MUL_8BY128(=, 0, 0, 0)
            GF_MUL_8BY128(^=, 0, 0, 1)
            GF_MUL_8BY128(^=, 0, 0, 2)
            GF_MUL_8BY128(^=, 0, 0, 3)
            GF_MUL_8BY128(^=, 0, 1, 0)
            GF_MUL_8BY128(^=, 0, 1, 1)
            GF_MUL_8BY128(^=, 0, 1, 2)
            GF_MUL_8BY128(^=, 0, 1, 3)
            GF_MUL_8BY128(^=, 1, 2, 0)
            GF_MUL_8BY128(^=, 1, 2, 1)
            GF_MUL_8BY128(^=, 1, 2, 2)
            GF_MUL_8BY128(^=, 1, 2, 3)
            GF_MUL_8BY128(^=, 1, 3, 0)
            GF_MUL_8BY128(^=, 1, 3, 1)
            GF_MUL_8BY128(^=, 1, 3, 2)
            GF_MUL_8BY128(^=, 1, 3, 3)

            x0 = a0; x1 = a1;
        }
        while (len >= HASH_BLOCKSIZE);

        hashBuffer[0] = x0; hashBuffer[1] = x1;
        return len;
        }
#endif    // #ifndef CRYPTOPP_GENERATE_X64_MASM

#ifdef CRYPTOPP_X64_MASM_AVAILABLE
    case 1:        // SSE2 and 2K tables
        GCM_AuthenticateBlocks_2K_SSE2(data, len/16, hashBuffer, s_reductionTable);
        return len % 16;
    case 3:        // SSE2 and 64K tables
        GCM_AuthenticateBlocks_64K_SSE2(data, len/16, hashBuffer);
        return len % 16;
#endif

#if CRYPTOPP_SSE2_ASM_AVAILABLE
    case 1:        // SSE2 and 2K tables
        {
        #ifdef __GNUC__
            __asm__ __volatile__
            (
            INTEL_NOPREFIX
        #elif defined(CRYPTOPP_GENERATE_X64_MASM)
            ALIGN   8
            GCM_AuthenticateBlocks_2K_SSE2    PROC FRAME
            rex_push_reg rsi
            push_reg rdi
            push_reg rbx
            .endprolog
            mov rsi, r8
            mov r11, r9
        #else
            AS2(    mov        WORD_REG(cx), data        )
            AS2(    mov        WORD_REG(dx), len         )
            AS2(    mov        WORD_REG(si), hashBuffer  )
            AS2(    shr        WORD_REG(dx), 4           )
        #endif

        #if CRYPTOPP_BOOL_X32
            AS1(push    rbx)
            AS1(push    rbp)
        #else
            AS_PUSH_IF86(    bx)
            AS_PUSH_IF86(    bp)
        #endif

        #ifdef __GNUC__
            AS2(    mov      AS_REG_7, WORD_REG(di))
        #elif CRYPTOPP_BOOL_X86
            AS2(    lea      AS_REG_7, s_reductionTable)
        #endif

        AS2(    movdqa   xmm0, [WORD_REG(si)]            )

        #define MUL_TABLE_0 WORD_REG(si) + 32
        #define MUL_TABLE_1 WORD_REG(si) + 32 + 1024
        #define RED_TABLE AS_REG_7

        ASL(0)
        AS2(    movdqu   xmm4, [WORD_REG(cx)]            )
        AS2(    pxor     xmm0, xmm4                      )

        AS2(    movd     ebx, xmm0                       )
        AS2(    mov      eax, AS_HEX(f0f0f0f0)           )
        AS2(    and      eax, ebx                        )
        AS2(    shl      ebx, 4                          )
        AS2(    and      ebx, AS_HEX(f0f0f0f0)           )
        AS2(    movzx    edi, ah                         )
        AS2(    movdqa   xmm5, XMMWORD_PTR [MUL_TABLE_1 + WORD_REG(di)]    )
        AS2(    movzx    edi, al                         )
        AS2(    movdqa   xmm4, XMMWORD_PTR [MUL_TABLE_1 + WORD_REG(di)]    )
        AS2(    shr      eax, 16                         )
        AS2(    movzx    edi, ah                         )
        AS2(    movdqa   xmm3, XMMWORD_PTR [MUL_TABLE_1 + WORD_REG(di)]    )
        AS2(    movzx    edi, al                         )
        AS2(    movdqa   xmm2, XMMWORD_PTR [MUL_TABLE_1 + WORD_REG(di)]    )

        #define SSE2_MUL_32BITS(i)                                                       \
            AS2(    psrldq  xmm0, 4                                                     )\
            AS2(    movd    eax, xmm0                                                   )\
            AS2(    and     eax, AS_HEX(f0f0f0f0)                                       )\
            AS2(    movzx   edi, bh                                                     )\
            AS2(    pxor    xmm5, XMMWORD_PTR [MUL_TABLE_0 + (i-1)*256 + WORD_REG(di)]  )\
            AS2(    movzx   edi, bl                                                     )\
            AS2(    pxor    xmm4, XMMWORD_PTR [MUL_TABLE_0 + (i-1)*256 + WORD_REG(di)]  )\
            AS2(    shr     ebx, 16                                                     )\
            AS2(    movzx   edi, bh                                                     )\
            AS2(    pxor    xmm3, XMMWORD_PTR [MUL_TABLE_0 + (i-1)*256 + WORD_REG(di)]  )\
            AS2(    movzx   edi, bl                                                     )\
            AS2(    pxor    xmm2, XMMWORD_PTR [MUL_TABLE_0 + (i-1)*256 + WORD_REG(di)]  )\
            AS2(    movd    ebx, xmm0                                                   )\
            AS2(    shl     ebx, 4                                                      )\
            AS2(    and     ebx, AS_HEX(f0f0f0f0)                                       )\
            AS2(    movzx   edi, ah                                                     )\
            AS2(    pxor    xmm5, XMMWORD_PTR [MUL_TABLE_1 + i*256 + WORD_REG(di)]      )\
            AS2(    movzx   edi, al                                                     )\
            AS2(    pxor    xmm4, XMMWORD_PTR [MUL_TABLE_1 + i*256 + WORD_REG(di)]      )\
            AS2(    shr     eax, 16                                                     )\
            AS2(    movzx   edi, ah                                                     )\
            AS2(    pxor    xmm3, XMMWORD_PTR [MUL_TABLE_1 + i*256 + WORD_REG(di)]      )\
            AS2(    movzx   edi, al                                                     )\
            AS2(    pxor    xmm2, XMMWORD_PTR [MUL_TABLE_1 + i*256 + WORD_REG(di)]      )\

        SSE2_MUL_32BITS(1)
        SSE2_MUL_32BITS(2)
        SSE2_MUL_32BITS(3)

        AS2(    movzx   edi, bh                    )
        AS2(    pxor    xmm5, XMMWORD_PTR [MUL_TABLE_0 + 3*256 + WORD_REG(di)]    )
        AS2(    movzx   edi, bl                    )
        AS2(    pxor    xmm4, XMMWORD_PTR [MUL_TABLE_0 + 3*256 + WORD_REG(di)]    )
        AS2(    shr     ebx, 16                    )
        AS2(    movzx   edi, bh                    )
        AS2(    pxor    xmm3, XMMWORD_PTR [MUL_TABLE_0 + 3*256 + WORD_REG(di)]    )
        AS2(    movzx   edi, bl                    )
        AS2(    pxor    xmm2, XMMWORD_PTR [MUL_TABLE_0 + 3*256 + WORD_REG(di)]    )

        AS2(    movdqa  xmm0, xmm3                 )
        AS2(    pslldq  xmm3, 1                    )
        AS2(    pxor    xmm2, xmm3                 )
        AS2(    movdqa  xmm1, xmm2                 )
        AS2(    pslldq  xmm2, 1                    )
        AS2(    pxor    xmm5, xmm2                 )

        AS2(    psrldq  xmm0, 15                   )
#if USE_MOVD_REG32
        AS2(    movd    edi, xmm0                  )
#elif USE_MOV_REG32_OR_REG64
        AS2(    mov     WORD_REG(di), xmm0         )
#else    // GNU Assembler
        AS2(    movd    WORD_REG(di), xmm0         )
#endif
        AS2(    movzx   eax, WORD PTR [RED_TABLE + WORD_REG(di)*2]  )
        AS2(    shl     eax, 8                     )

        AS2(    movdqa  xmm0, xmm5                 )
        AS2(    pslldq  xmm5, 1                    )
        AS2(    pxor    xmm4, xmm5                 )

        AS2(    psrldq  xmm1, 15                   )
#if USE_MOVD_REG32
        AS2(    movd    edi, xmm1                  )
#elif USE_MOV_REG32_OR_REG64
        AS2(    mov     WORD_REG(di), xmm1         )
#else
        AS2(    movd    WORD_REG(di), xmm1         )
#endif
        AS2(    xor     ax, WORD PTR [RED_TABLE + WORD_REG(di)*2]  )
        AS2(    shl     eax, 8                     )

        AS2(    psrldq  xmm0, 15                   )
#if USE_MOVD_REG32
        AS2(    movd    edi, xmm0                  )
#elif USE_MOV_REG32_OR_REG64
        AS2(    mov     WORD_REG(di), xmm0         )
#else
        AS2(    movd    WORD_REG(di), xmm0         )
#endif
        AS2(    xor     ax, WORD PTR [RED_TABLE + WORD_REG(di)*2]  )

        AS2(    movd    xmm0, eax                  )
        AS2(    pxor    xmm0, xmm4                 )

        AS2(    add     WORD_REG(cx), 16           )
        AS2(    sub     WORD_REG(dx), 1            )
        // ATT_NOPREFIX
        ASJ(    jnz,    0, b                       )
        INTEL_NOPREFIX
        AS2(    movdqa  [WORD_REG(si)], xmm0       )

        #if CRYPTOPP_BOOL_X32
            AS1(pop        rbp)
            AS1(pop        rbx)
        #else
            AS_POP_IF86(    bp)
            AS_POP_IF86(    bx)
        #endif

        #ifdef __GNUC__
                ATT_PREFIX
                    :
                    : "c" (data), "d" (len/16), "S" (hashBuffer), "D" (s_reductionTable)
                    : "memory", "cc", "%eax"
            #if CRYPTOPP_BOOL_X64
                    , "%ebx", "%r11"
            #endif
                );
        #elif defined(CRYPTOPP_GENERATE_X64_MASM)
            pop rbx
            pop rdi
            pop rsi
            ret
            GCM_AuthenticateBlocks_2K_SSE2 ENDP
        #endif

        return len%16;
        }
    case 3:        // SSE2 and 64K tables
        {
        #ifdef __GNUC__
            __asm__ __volatile__
            (
            INTEL_NOPREFIX
        #elif defined(CRYPTOPP_GENERATE_X64_MASM)
            ALIGN   8
            GCM_AuthenticateBlocks_64K_SSE2    PROC FRAME
            rex_push_reg rsi
            push_reg rdi
            .endprolog
            mov rsi, r8
        #else
            AS2(    mov        WORD_REG(cx), data       )
            AS2(    mov        WORD_REG(dx), len        )
            AS2(    mov        WORD_REG(si), hashBuffer )
            AS2(    shr        WORD_REG(dx), 4          )
        #endif

        AS2(    movdqa    xmm0, [WORD_REG(si)]          )

        #undef MUL_TABLE
        #define MUL_TABLE(i,j) WORD_REG(si) + 32 + (i*4+j)*256*16

        ASL(1)
        AS2(    movdqu    xmm1, [WORD_REG(cx)]          )
        AS2(    pxor    xmm1, xmm0                      )
        AS2(    pxor    xmm0, xmm0                      )

        #undef SSE2_MUL_32BITS
        #define SSE2_MUL_32BITS(i)                                   \
            AS2(    movd    eax, xmm1                               )\
            AS2(    psrldq    xmm1, 4                               )\
            AS2(    movzx    edi, al                                )\
            AS2(    add        WORD_REG(di), WORD_REG(di)           )\
            AS2(    pxor    xmm0, [MUL_TABLE(i,0) + WORD_REG(di)*8] )\
            AS2(    movzx    edi, ah                                )\
            AS2(    add        WORD_REG(di), WORD_REG(di)           )\
            AS2(    pxor    xmm0, [MUL_TABLE(i,1) + WORD_REG(di)*8] )\
            AS2(    shr        eax, 16                              )\
            AS2(    movzx    edi, al                                )\
            AS2(    add        WORD_REG(di), WORD_REG(di)           )\
            AS2(    pxor    xmm0, [MUL_TABLE(i,2) + WORD_REG(di)*8] )\
            AS2(    movzx    edi, ah                                )\
            AS2(    add        WORD_REG(di), WORD_REG(di)           )\
            AS2(    pxor    xmm0, [MUL_TABLE(i,3) + WORD_REG(di)*8] )\

        SSE2_MUL_32BITS(0)
        SSE2_MUL_32BITS(1)
        SSE2_MUL_32BITS(2)
        SSE2_MUL_32BITS(3)

        AS2(    add     WORD_REG(cx), 16      )
        AS2(    sub     WORD_REG(dx), 1       )
        // ATT_NOPREFIX
        ASJ(    jnz,    1, b                  )
        INTEL_NOPREFIX
        AS2(    movdqa  [WORD_REG(si)], xmm0  )

        #ifdef __GNUC__
                ATT_PREFIX
                    :
                    : "c" (data), "d" (len/16), "S" (hashBuffer)
                    : "memory", "cc", "%edi", "%eax"
                );
        #elif defined(CRYPTOPP_GENERATE_X64_MASM)
            pop rdi
            pop rsi
            ret
            GCM_AuthenticateBlocks_64K_SSE2 ENDP
        #endif

        return len%16;
        }
#endif
#ifndef CRYPTOPP_GENERATE_X64_MASM
    }

    return len%16;
}

void GCM_Base::AuthenticateLastHeaderBlock()
{
    if (m_bufferedDataLength > 0)
    {
        memset(m_buffer+m_bufferedDataLength, 0, HASH_BLOCKSIZE-m_bufferedDataLength);
        m_bufferedDataLength = 0;
        GCM_Base::AuthenticateBlocks(m_buffer, HASH_BLOCKSIZE);
    }
}

void GCM_Base::AuthenticateLastConfidentialBlock()
{
    GCM_Base::AuthenticateLastHeaderBlock();
    PutBlock<word64, BigEndian, true>(NULLPTR, m_buffer)(m_totalHeaderLength*8)(m_totalMessageLength*8);
    GCM_Base::AuthenticateBlocks(m_buffer, HASH_BLOCKSIZE);
}

void GCM_Base::AuthenticateLastFooterBlock(byte *mac, size_t macSize)
{
    m_ctr.Seek(0);
    ReverseHashBufferIfNeeded();
    m_ctr.ProcessData(mac, HashBuffer(), macSize);
}

NAMESPACE_END

#endif    // Not CRYPTOPP_GENERATE_X64_MASM
#endif
