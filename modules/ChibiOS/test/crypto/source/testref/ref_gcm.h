#ifndef TEST_REF_GCM_H_
#define TEST_REF_GCM_H_



#define K3_LEN      16
#define P3_LEN      16
#define IV3_LEN     (12+4)
#define AAD3_LEN    16
#define C3_LEN      16
#define T3_LEN      16

extern const uint8_t K3[K3_LEN];
extern const uint8_t IV3[IV3_LEN];
extern const uint8_t P3[P3_LEN];
extern const uint8_t A3[AAD3_LEN];
extern const uint8_t C3[C3_LEN];
extern const uint8_t T3[T3_LEN];

///////
// http://csrc.nist.gov/groups/STM/cavp/gcmtestvectors.zip gcmEncryptExtIV128.rsp
// [Keylen = 128]
// [IVlen = 96]
// [PTlen = 256]
// [AADlen = 128]
// [Taglen = 128]
// Count = 0
// K = 298efa1ccf29cf62ae6824bfc19557fc
// IV = 6f58a93fe1d207fae4ed2f6d
// P = cc38bccd6bc536ad919b1395f5d63801f99f8068d65ca5ac63872daf16b93901
// AAD = 021fafd238463973ffe80256e5b1c6b1
// C = dfce4e9cd291103d7fe4e63351d9e79d3dfd391e3267104658212da96521b7db
// T = 542465ef599316f73a7a560509a2d9f2
///////
#define K4_LEN      16
#define P4_LEN      32
#define IV4_LEN     (12+4)
#define AAD4_LEN    16
#define C4_LEN      32
#define T4_LEN      16

extern const uint8_t K4[K4_LEN] ;
extern const uint8_t IV4[IV4_LEN];
extern const uint8_t P4[P4_LEN] ;
extern const uint8_t A4[AAD4_LEN] ;
extern const uint8_t C4[C4_LEN] ;
extern const uint8_t T4[T4_LEN] ;

///////
// http://csrc.nist.gov/groups/STM/cavp/gcmtestvectors.zip gcmEncryptExtIV128.rsp
// [Keylen = 128]
// [IVlen = 96]
// [PTlen = 256]
// [AADlen = 128]
// [Taglen = 128]
// Count = 0
// K = 298efa1ccf29cf62ae6824bfc19557fc
// IV = 6f58a93fe1d207fae4ed2f6d
// P = cc38bccd6bc536ad919b1395f5d63801f99f8068d65ca5ac63872daf16b93901
// AAD = 021fafd238463973ffe80256e5b1c6b1
// C = dfce4e9cd291103d7fe4e63351d9e79d3dfd391e3267104658212da96521b7db
// T = 542465ef599316f73a7a560509a2d9f2
///////
#define K5_LEN      16
#define P5_LEN      32
#define IV5_LEN     (12+4)
#define AAD5_LEN    16
#define C5_LEN      32
#define T5_LEN      16

extern const uint8_t K5[K5_LEN] ;
extern const uint8_t IV5[IV5_LEN];
extern const uint8_t P5[P5_LEN] ;
extern const uint8_t A5[AAD5_LEN] ;
extern const uint8_t C5[C5_LEN] ;
extern const uint8_t T5[T5_LEN] ;





#endif //TEST_REF_GCM_H_
