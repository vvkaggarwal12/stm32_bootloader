#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>

typedef struct __CIR_BUFFER {
  uint8_t *Buffer; 
  uint32_t size;
  uint32_t readPos;
  uint32_t writePos;
  uint32_t overflow;
} CIR_BUFFER;

int32_t pushToCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size );
int32_t CirBufInit (CIR_BUFFER *circular_buf, uint32_t size,uint8_t *ptr);
int32_t peekIntoCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size);
int32_t popFromCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size);
int32_t getCirBufAvailableSize ( CIR_BUFFER *circular_buf );
int32_t flushCirBuf ( CIR_BUFFER *circular_buf );
int32_t getCirBufSize(CIR_BUFFER *circular_buf);

#endif
