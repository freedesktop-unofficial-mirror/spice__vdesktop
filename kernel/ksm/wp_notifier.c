#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#include "wp_notifier.h"

static int pre_do_wp_page(struct kprobe *p,
                         struct pt_regs *regs)
{
       struct mm_struct *mm;
       unsigned long address;

       /*
        * kprobes runs with irq disabled and preempt disabled but we
        * need irq enabled to flush the smp tlb with IPIs while
        * tearing down sptes.
        */
       local_irq_enable();

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
       mm = (struct mm_struct *) regs->rdi;
       address = (unsigned long) regs->rdx;
#else
       mm = (struct mm_struct *) regs->di;
       address = (unsigned long) regs->dx;
#endif
       kvm_wp_notifier(mm, address);

       local_irq_disable();

       return 0;
}

static struct kprobe not_kprobe;

int init_wp_notifier(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) && defined(CONFIG_KALLSYMS) && !defined(RHEL_RELEASE_CODE)
       not_kprobe.addr = (kprobe_opcode_t *)kallsyms_lookup_name("do_wp_page");
       if (!not_kprobe.addr) {
               printk(KERN_WARNING "do_wp_page not found");
               return 1;
       }
#else
       not_kprobe.symbol_name = "do_wp_page";
#endif
       not_kprobe.pre_handler = pre_do_wp_page;

       if (register_kprobe(&not_kprobe)) {
               printk(KERN_WARNING "cant register kprobe for do_wp_page");
               return 1;
       }

       return 0;
}

void exit_wp_notifier(void)
{
       unregister_kprobe(&not_kprobe);
}
