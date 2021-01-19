#include "circular_buf.h"
#include "stddef.h"
#include "string.h"

int32_t CirBufInit(CIR_BUFFER *circular_buf, uint32_t size, uint8_t *ptr) {
	uint32_t retval = 0;
	if (circular_buf != NULL) {
		circular_buf->Buffer = ptr;
		if (circular_buf->Buffer != NULL) {
			circular_buf->size = size;
			circular_buf->readPos = 0;
			circular_buf->writePos = 0;
			circular_buf->overflow = 0;
		} else {
			retval = -1;
		}
	} else {
		retval = -1;
	}
	return retval;
}

int32_t getCirBufAvailableSize(CIR_BUFFER *circular_buf) {
	uint32_t retval;
	if (circular_buf != NULL) {
		retval = circular_buf->size - getCirBufSize(circular_buf);
	} else {
		retval = -1;
	}
	return retval;
}

int32_t getCirBufSize(CIR_BUFFER *circular_buf) {
	uint32_t retval = 0;
	if (circular_buf != NULL) {
		if ((circular_buf->readPos <= circular_buf->writePos)
				&& circular_buf->overflow == 0) {
			retval = (circular_buf->writePos) - circular_buf->readPos;
		} else if ((circular_buf->writePos == circular_buf->readPos)
				&& circular_buf->overflow == 1) {
			retval = circular_buf->size;
		} else {
			retval = (circular_buf->size - circular_buf->readPos)
					+ circular_buf->writePos;
		}
	}

	return retval;
}

int32_t pushToCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size) {
	int32_t retval = 0;
	uint32_t i;
	uint32_t BufSize = 0;

	if (circular_buf != NULL) {
		BufSize = getCirBufAvailableSize(circular_buf);
		if (BufSize >= size) {
			for (i = 0; i < size; i++) {
				circular_buf->Buffer[circular_buf->writePos] = buffer[i];
				if (circular_buf->writePos + 1 == circular_buf->size) {
					circular_buf->overflow = 1;
					circular_buf->writePos = 0;
				} else {
					circular_buf->writePos++;
				}
			}
			retval = size;
		} else if (BufSize < size) {
			retval = BufSize;
		}
	} else {
		retval = -1;
	}

	return retval;
}

int32_t peekIntoCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size) {
	uint32_t retval = 0;
	int32_t i;
	uint32_t readPosition = circular_buf->readPos;

	if (circular_buf != NULL) {
		i = getCirBufSize(circular_buf);
		if ((i >= 0) && ((uint32_t) i >= size)) {
			for (i = 0; (uint32_t) i < size; i++) {
				buffer[i] = circular_buf->Buffer[readPosition];
				readPosition++;
				if (readPosition == circular_buf->size) {
					readPosition = 0;
				}
			}
			retval = size;
		} else {
			retval = -1;
		}
	} else {
		retval = -1;
	}
	return retval;
}

int32_t popFromCirBuf(CIR_BUFFER *circular_buf, uint8_t *buffer, uint32_t size) {
	int32_t retval = 0;
	uint32_t i;
	uint32_t BufSize;

	if (circular_buf != NULL) {
		BufSize = getCirBufSize(circular_buf);
		if (BufSize >= size) {
			for (i = 0; i < size; i++) {
				buffer[i] = circular_buf->Buffer[circular_buf->readPos];
				if (circular_buf->readPos + 1 == circular_buf->size) {
					circular_buf->readPos = 0;
					circular_buf->overflow = 0;
				} else {
					circular_buf->readPos++;
				}
			}
			retval = size;
		} else if (BufSize < size) {
			retval = BufSize;
		}
	} else {
		retval = -1;
	}

	return retval;
}

int32_t flushCirBuf(CIR_BUFFER *circular_buf) {
	uint32_t retval = 0;
	if (circular_buf != NULL) {
		circular_buf->writePos = 0;
		circular_buf->readPos = 0;
		circular_buf->overflow = 0;
		memset(circular_buf->Buffer, 0x0, circular_buf->size);
	} else {
		retval = -1;
	}
	return retval;
}

