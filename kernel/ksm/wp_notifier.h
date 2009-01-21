#ifndef WP_NOTIFIER_H
#define WP_NOTIFIER_H

#include <linux/mm.h>

int init_wp_notifier(void);
void exit_wp_notifier(void);

void kvm_wp_notifier(struct mm_struct *mm,
                    unsigned long address);

#endif
