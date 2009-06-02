#ifndef WP_NOTIFIER_H
#define WP_NOTIFIER_H

#include <linux/mm.h>

#ifndef CONFIG_MMU_NOTIFIER
int init_wp_notifier(void);
void exit_wp_notifier(void);
#else
static inline int init_wp_notifier(void) {return 0;}
static inline void exit_wp_notifier(void) {return;}
#endif

void kvm_wp_notifier(struct mm_struct *mm,
                    unsigned long address);

#endif
