#include <linux/types.h>

extern int __builtin_memcmp(const void *,const void *,__kernel_size_t);
extern int memcmp(const void *,const void *,__kernel_size_t);

extern int memcmp(const void *from,const void *to,__kernel_size_t size)
{
	__builtin_memcmp(from,to,size);
}
