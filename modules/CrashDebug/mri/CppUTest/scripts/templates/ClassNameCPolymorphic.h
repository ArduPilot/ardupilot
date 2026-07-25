#ifndef D_ClassName_H
#define D_ClassName_H

/**********************************************************
 *
 * ClassName is responsible for ...
 *
 **********************************************************/

typedef struct ClassName ClassNamePiml;

ClassName* ClassName_Create(void);
void ClassName_Destroy(ClassName*);
void ClassName_VirtualFunction_impl(ClassName*);

#endif  /* D_FakeClassName_H */
