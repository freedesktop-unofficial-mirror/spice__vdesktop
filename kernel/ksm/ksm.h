#ifndef __LINUX_KSM_H
#define __LINUX_KSM_H

/*
 * Userspace interface for /dev/ksm - kvm shared memory
 */

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <sys/types.h>
#include <sys/ioctl.h>
#endif

#include <asm/types.h>

#define KSM_API_VERSION 1

#define ksm_control_flags_run 1

/* for KSM_REGISTER_MEMORY_REGION */
struct ksm_memory_region {
	__u32 npages; /* number of pages to share */
	__u32 pad;
	__u64 addr; /* the begining of the virtual address */
};

struct ksm_user_scan {
	__u32 pages_to_scan;
	__u32 flags; /* control flags */
};

struct ksm_kthread_info {
	__u32 sleep; /* number of microsecoends to sleep */
	__u32 pages_to_scan; /* number of pages to scan */
	__u32 flags; /* control flags */
};

#define KSMIO 0xAB

/* ioctls for /dev/ksm */

#define KSM_GET_API_VERSION              _IO(KSMIO,   0x00)
/*
 * KSM_CREATE_SHARED_MEMORY_AREA - create the shared memory reagion fd
 */
#define KSM_CREATE_SHARED_MEMORY_AREA    _IO(KSMIO,   0x01) /* return SMA fd */
/*
 * KSM_CREATE_SCAN - create the scanner fd
 */
#define KSM_CREATE_SCAN                  _IO(KSMIO,   0x02) /* return SCAN fd */
/*
 * KSM_START_STOP_KTHREAD - control the kernel thread scanning speed
 * (can stop the kernel thread from working by setting running = 0)
 */
#define KSM_START_STOP_KTHREAD		 _IOW(KSMIO,  0x03,\
					      struct ksm_kthread_info)
/*
 * KSM_GET_INFO_KTHREAD - return information about the kernel thread
 * scanning speed.
 */
#define KSM_GET_INFO_KTHREAD		 _IOW(KSMIO,  0x04,\
					      struct ksm_kthread_info)


/* ioctls for SMA fds */

/*
 * KSM_REGISTER_MEMORY_REGION - register virtual address memory area to be
 * scanned by kvm.
 */
#define KSM_REGISTER_MEMORY_REGION       _IOW(KSMIO,  0x20,\
					      struct ksm_memory_region)
/*
 * KSM_REMOVE_MEMORY_REGION - remove virtual address memory area from ksm.
 */
#define KSM_REMOVE_MEMORY_REGION         _IO(KSMIO,   0x21)

/* ioctls for SCAN fds */
#define KSM_SCAN                         _IOW(KSMIO,  0x40,\
					      struct ksm_user_scan)

#endif
