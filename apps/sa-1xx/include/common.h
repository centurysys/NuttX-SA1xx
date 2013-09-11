#ifndef __SA1XX_COMMON_H
#define __SA1XX_COMMON_H

#include <stdio.h>
#include <debug.h>
#include <sched.h>

#ifndef DBG
#ifdef CONFIG_DEBUG

#define DBG(fmt, args...) do {sched_lock(); lowsyslog(fmt, ## args); sched_unlock();} while (0)
#else  /* CONFIG_DEBUG */
#define DBG(fmt, args...) do {} while (0)
#endif /* CONFIG_DEBUG */
#endif /* DBG */

#ifndef info
#define info(fmt, arg...) do {sched_lock(); lowsyslog(fmt, ##arg); sched_unlock();} while (0)
#endif

#ifndef err
#define err(fmt, arg...) do {sched_lock(); lowsyslog("[ERR] " fmt, ##arg); sched_unlock();} while (0)
#endif

#ifndef warn
#define warn(fmt, arg...) do {sched_lock(); lowsyslog("[WARN] " fmt, ##arg); sched_unlock();} while (0)
#endif

#define ALIGN(x,a)		__ALIGN_MASK(x,(typeof(x))(a)-1)
#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))
#define PTR_ALIGN(p, a)		((typeof(p))ALIGN((unsigned long)(p), (a)))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define hb(x) ((x & 0xff00) >> 8)
#define lb(x)  (x & 0x00ff)

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#endif
