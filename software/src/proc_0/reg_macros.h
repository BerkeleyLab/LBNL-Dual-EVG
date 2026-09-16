/*
 * Register macros
 */

#ifndef _REG_MACROS_H_
#define _REG_MACROS_H_

#include <stdint.h>
#include <xil_io.h>

/*
 * Read/Write registers
 */

#ifndef __REG_MACROS_DEFINED__
#define __REG_MACROS_DEFINED__
#define REG_GEN_MASK(offset, size) (((1<<(size))-1) << (offset))
#define REG_GEN_WRITE(value, offset, size) (((value) & ((1<<(size))-1)) << (offset))
#define REG_GEN_READ(reg, offset, size) (((reg) >> (offset)) & ((1<<(size))-1))
#define REG_SIGN_EXTEND(value, bits) (((value) & (1<<bits) ? ~((1<<(bits))-1): 0 ) | (value))
#endif

#endif /* _REG_MACROS_H_ */
