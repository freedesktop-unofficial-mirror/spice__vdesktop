
/*
 * Compatibility header for building as an external module.
 */

/*
 * Avoid picking up the kernel's kvm.h in case we have a newer one.
 */

#include <linux/compiler.h>
#include <linux/version.h>
#include "include/linux/kvm.h"
#include "include/linux/kvm_para.h"
#include <linux/cpu.h>
#include <asm/processor.h>
#include <linux/hrtimer.h>
#include <asm/bitops.h>

/*
 * 2.6.16 does not have GFP_NOWAIT
 */

#include <linux/gfp.h>

#ifndef GFP_NOWAIT
#define GFP_NOWAIT (GFP_ATOMIC & ~__GFP_HIGH)
#endif


/*
 * kvm profiling support needs 2.6.20
 */
#include <linux/profile.h>

#ifndef KVM_PROFILING
#define KVM_PROFILING 1234
#define prof_on       4321
#endif

/*
 * smp_call_function_single() is not exported below 2.6.20
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)

#include <linux/spinlock.h>
#include <linux/smp.h>

static spinlock_t scfs_lock = SPIN_LOCK_UNLOCKED;
static int scfs_cpu;
static void (*scfs_func)(void *info);

static void scfs_thunk(void *info)
{
	if (raw_smp_processor_id() == scfs_cpu)
		scfs_func(info);
}

static inline int smp_call_function_single1(int cpu, void (*func)(void *info),
					   void *info, int nonatomic, int wait)
{
	int r, this_cpu;

	this_cpu = get_cpu();
	if (cpu == this_cpu) {
		r = 0;
		local_irq_disable();
		func(info);
		local_irq_enable();
	} else {
		spin_lock(&scfs_lock);
		scfs_cpu = cpu;
		scfs_func = func;
		r = smp_call_function(scfs_thunk, info, nonatomic, wait);
		spin_unlock(&scfs_lock);
	}
	put_cpu();
	return r;
}

#define smp_call_function_single smp_call_function_single1

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
/*
 * pre 2.6.23 doesn't handle smp_call_function_single on current cpu
 */

#include <linux/smp.h>

static inline int smp_call_function_single2(int cpu, void (*func)(void *info),
					    void *info, int nonatomic, int wait)
{
	int this_cpu, r;

	this_cpu = get_cpu();
	if (cpu == this_cpu) {
		r = 0;
		local_irq_disable();
		func(info);
		local_irq_enable();
	} else
		r = smp_call_function_single(cpu, func, info, nonatomic, wait);
	put_cpu();
	return r;
}

#define smp_call_function_single smp_call_function_single2

#endif

/*
 * The cpu hotplug stubs are broken if !CONFIG_CPU_HOTPLUG
 */

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
#define DEFINE_MUTEX(a) DECLARE_MUTEX(a)
#define mutex_lock_interruptible(a) down_interruptible(a)
#define mutex_unlock(a) up(a)
#define mutex_lock(a) down(a)
#define mutex_init(a) init_MUTEX(a)
#define mutex_trylock(a) down_trylock(a)
#define mutex semaphore
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
#ifndef kzalloc
#define kzalloc(size,flags)			\
({						\
	void *__ret = kmalloc(size, flags);	\
	if (__ret)
		memset(__ret, 0, size);
	__ret;
})
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)
#ifndef kmem_cache_zalloc
#define kmem_cache_zalloc(cache,flags)			  \
({							  \
	void *__ret = kmem_cache_alloc(cache, flags);	  \
	if (__ret)                                        \
		memset(__ret, 0, kmem_cache_size(cache)); \
	__ret;                                            \
})
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)

#ifndef CONFIG_HOTPLUG_CPU
#define register_cpu_notifier(nb) (0)
#endif

#endif

#include <linux/miscdevice.h>
#ifndef KVM_MINOR
#define KVM_MINOR 232
#endif

#include <linux/notifier.h>
#ifndef CPU_TASKS_FROZEN

#define CPU_TASKS_FROZEN       0x0010
#define CPU_ONLINE_FROZEN      (CPU_ONLINE | CPU_TASKS_FROZEN)
#define CPU_UP_PREPARE_FROZEN  (CPU_UP_PREPARE | CPU_TASKS_FROZEN)
#define CPU_UP_CANCELED_FROZEN (CPU_UP_CANCELED | CPU_TASKS_FROZEN)
#define CPU_DOWN_PREPARE_FROZEN        (CPU_DOWN_PREPARE | CPU_TASKS_FROZEN)
#define CPU_DOWN_FAILED_FROZEN (CPU_DOWN_FAILED | CPU_TASKS_FROZEN)
#define CPU_DEAD_FROZEN                (CPU_DEAD | CPU_TASKS_FROZEN)

#endif

#ifndef CPU_DYING
#define CPU_DYING 0x000A
#define CPU_DYING_FROZEN (CPU_DYING | CPU_TASKS_FROZEN)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)

#ifndef _EFER_SCE
#define _EFER_SCE		0  /* SYSCALL/SYSRET */
#endif

#ifndef EFER_SCE
#define EFER_SCE		(1<<_EFER_SCE)
#endif

#endif

/*
 * For set_64bit(), which is in a new file asm/cmpxchg.h in newer kernels.
 */
#include <asm/system.h>

#define anon_inode_getfd kvm_anon_inode_getfd
int kvm_init_anon_inodes(void);
void kvm_exit_anon_inodes(void);

#include <linux/smp.h>

#define on_cpu kvm_on_cpu

/*
 * Call a function on one processor, which might be the currently executing
 * processor.
 */
static inline int on_cpu(int cpu, void (*func) (void *info), void *info,
			 int retry, int wait)
{
	int ret;
	int this_cpu;

	this_cpu = get_cpu();
	if (this_cpu == cpu) {
		local_irq_disable();
		func(info);
		local_irq_enable();
		ret = 0;
	} else
		ret = smp_call_function_single(cpu, func, info, retry, wait);
	put_cpu();
	return ret;
}

#ifndef X86_CR0_PE
#define X86_CR0_PE 0x00000001
#endif

#ifndef X86_CR0_MP
#define X86_CR0_MP 0x00000002
#endif

#ifndef X86_CR0_EM
#define X86_CR0_EM 0x00000004
#endif

#ifndef X86_CR0_TS
#define X86_CR0_TS 0x00000008
#endif

#ifndef X86_CR0_ET
#define X86_CR0_ET 0x00000010
#endif

#ifndef X86_CR0_NE
#define X86_CR0_NE 0x00000020
#endif

#ifndef X86_CR0_WP
#define X86_CR0_WP 0x00010000
#endif

#ifndef X86_CR0_AM
#define X86_CR0_AM 0x00040000
#endif

#ifndef X86_CR0_NW
#define X86_CR0_NW 0x20000000
#endif

#ifndef X86_CR0_CD
#define X86_CR0_CD 0x40000000
#endif

#ifndef X86_CR0_PG
#define X86_CR0_PG 0x80000000
#endif

#ifndef X86_CR3_PWT
#define X86_CR3_PWT 0x00000008
#endif

#ifndef X86_CR3_PCD
#define X86_CR3_PCD 0x00000010
#endif

#ifndef X86_CR4_VMXE
#define X86_CR4_VMXE 0x00002000
#endif

#undef X86_CR8_TPR
#define X86_CR8_TPR 0x0f

/*
 * 2.6.23 removed the cache destructor
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#  define KMEM_CACHE_CREATE_CTOR_DTOR NULL, NULL
#else
#  define KMEM_CACHE_CREATE_CTOR_DTOR NULL
#endif

/*
 * 2.6.22 does not define set_64bit() under nonpae
 */
#ifdef CONFIG_X86_32

#include <asm/cmpxchg.h>

static inline void __kvm_set_64bit(u64 *ptr, u64 val)
{
	unsigned int low = val;
	unsigned int high = val >> 32;

	__asm__ __volatile__ (
		"\n1:\t"
		"movl (%0), %%eax\n\t"
		"movl 4(%0), %%edx\n\t"
		"lock cmpxchg8b (%0)\n\t"
		"jnz 1b"
		: /* no outputs */
		:	"D"(ptr),
			"b"(low),
			"c"(high)
		:	"ax","dx","memory");
}

#undef  set_64bit
#define set_64bit __kvm_set_64bit

#endif

#ifndef CONFIG_PREEMPT_NOTIFIERS

struct preempt_notifier;

struct preempt_ops {
	void (*sched_in)(struct preempt_notifier *notifier, int cpu);
	void (*sched_out)(struct preempt_notifier *notifier,
			  struct task_struct *next);
};

struct preempt_notifier {
	struct list_head link;
	struct task_struct *tsk;
	struct preempt_ops *ops;
};

void preempt_notifier_register(struct preempt_notifier *notifier);
void preempt_notifier_unregister(struct preempt_notifier *notifier);

static inline void preempt_notifier_init(struct preempt_notifier *notifier,
				     struct preempt_ops *ops)
{
	notifier->ops = ops;
}

void start_special_insn(void);
void end_special_insn(void);
void in_special_section(void);
void special_reload_dr7(void);

void preempt_notifier_sys_init(void);
void preempt_notifier_sys_exit(void);

#else

static inline void start_special_insn(void) {}
static inline void end_special_insn(void) {}
static inline void in_special_section(void) {}
static inline void special_reload_dr7(void) {}

static inline void preempt_notifier_sys_init(void) {}
static inline void preempt_notifier_sys_exit(void) {}

#endif

/* HRTIMER_MODE_ABS started life with a different name */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define HRTIMER_MODE_ABS HRTIMER_ABS
#endif

/* div64_64 is fairly new */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)

#ifdef CONFIG_64BIT

static inline uint64_t div64_64(uint64_t dividend, uint64_t divisor)
{
	return dividend / divisor;
}

#else

/* 64bit divisor, dividend and result. dynamic precision */
static inline uint64_t div64_64(uint64_t dividend, uint64_t divisor)
{
	uint32_t high, d;

	high = divisor >> 32;
	if (high) {
		unsigned int shift = fls(high);

		d = divisor >> shift;
		dividend >>= shift;
	} else
		d = divisor;

	do_div(dividend, d);

	return dividend;
}

#endif

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

typedef _Bool bool;

#endif
