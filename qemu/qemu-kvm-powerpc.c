
#include "config.h"
#include "config-host.h"

extern int kvm_allowed;
extern int kvm_irqchip;

#ifdef USE_KVM

#include <string.h>
#include "hw/hw.h"
#include "sysemu.h"
#include "cpu.h"
#include "helper_regs.h"

#include "qemu-kvm.h"
#include <libkvm.h>
#include <pthread.h>
#include <sys/utsname.h>

extern kvm_context_t kvm_context;
extern __thread CPUState *vcpu_env;

void cpu_reset(CPUState *env)
{
	cpu_ppc_reset(env);
}


int kvm_arch_qemu_create_context(void)
{
	return 0;
}

void kvm_arch_load_regs(CPUState *env)
{
    struct kvm_regs regs;
    int rc,i;

    rc = kvm_get_regs(kvm_context, env->cpu_index, &regs);
    if (rc == -1)
        perror("kvm_get_regs FAILED");

    /* cr is untouched in qemu and not existant in CPUState fr ppr */
    /* hflags is a morphed to MSR on ppc, no need to sync that down to kvm */

    regs.pc = env->nip;

    regs.ctr = env->ctr;
    regs.lr  = env->lr;
    regs.xer = ppc_load_xer(env);
    regs.msr = env->msr;

    regs.srr0 = env->spr[SPR_SRR0];
    regs.srr1 = env->spr[SPR_SRR1];

    regs.sprg0 = env->spr[SPR_SPRG0];
    regs.sprg1 = env->spr[SPR_SPRG1];
    regs.sprg2 = env->spr[SPR_SPRG2];
    regs.sprg3 = env->spr[SPR_SPRG3];
    regs.sprg4 = env->spr[SPR_SPRG4];
    regs.sprg5 = env->spr[SPR_SPRG5];
    regs.sprg6 = env->spr[SPR_SPRG6];
    regs.sprg7 = env->spr[SPR_SPRG7];

    for (i = 0;i < 32; i++){
        regs.gpr[i] = env->gpr[i];
        regs.fpr[i] = env->fpr[i];
    }

    rc = kvm_set_regs(kvm_context, env->cpu_index, &regs);
    if (rc == -1)
        perror("kvm_set_regs FAILED");
}


void kvm_arch_save_regs(CPUState *env)
{
    struct kvm_regs regs;
    uint32_t i, rc;

    rc = kvm_get_regs(kvm_context, env->cpu_index, &regs);
    if (rc == -1)
        perror("kvm_get_regs FAILED");

    env->ctr =regs.ctr;
    env->lr = regs.lr;
    ppc_store_xer(env,regs.xer);
    env->msr = regs.msr;
    /* calculate hflags based on the current msr using the ppc qemu helper */
    hreg_compute_hflags(env);

    env->nip = regs.pc;

    env->spr[SPR_SRR0] = regs.srr0;
    env->spr[SPR_SRR1] = regs.srr1;

    env->spr[SPR_SPRG0] = regs.sprg0;
    env->spr[SPR_SPRG1] = regs.sprg1;
    env->spr[SPR_SPRG2] = regs.sprg2;
    env->spr[SPR_SPRG3] = regs.sprg3;
    env->spr[SPR_SPRG4] = regs.sprg4;
    env->spr[SPR_SPRG5] = regs.sprg5;
    env->spr[SPR_SPRG6] = regs.sprg6;
    env->spr[SPR_SPRG7] = regs.sprg7;

    for (i = 0;i < 32; i++){
        env->gpr[i] = regs.gpr[i];
        env->fpr[i] = regs.fpr[i];
    }

}

int kvm_arch_qemu_init_env(CPUState *cenv)
{
    return 0;
}

int kvm_arch_halt(void *opaque, int vcpu)
{
    CPUState *env = cpu_single_env;

    if (!(env->interrupt_request & CPU_INTERRUPT_HARD)
	&& (msr_ee))
    {
            env->halted = 1;
	    env->exception_index = EXCP_HLT;
    }
    return 1;
}

void kvm_arch_pre_kvm_run(void *opaque, int vcpu)
{
	return;
}

void kvm_arch_post_kvm_run(void *opaque, int vcpu)
{
    CPUState *env = qemu_kvm_cpu_env(vcpu);
    cpu_single_env = env;
    env->ready_for_interrupt_injection = \
	kvm_is_ready_for_interrupt_injection(kvm_context, vcpu);
}

int kvm_arch_has_work(CPUState *env)
{
    if ((env->interrupt_request & (CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXIT)) &&
	(msr_ee))
	return 1;
    return 0;
}

int kvm_arch_try_push_interrupts(void *opaque)
{
    CPUState *env = cpu_single_env;
    int r;
    unsigned irq;

    if (env->ready_for_interrupt_injection &&
        (env->interrupt_request & CPU_INTERRUPT_HARD))
       {
            env->interrupt_request &= ~CPU_INTERRUPT_HARD;

            /* For now KVM disregards the 'irq' argument. However, in the
             * future KVM could cache it in-kernel to avoid a heavyweight exit
             * when reading the UIC.
             */
            irq = -1U;

            r = kvm_inject_irq(kvm_context, env->cpu_index, irq);
            if (r < 0)
                printf("cpu %d fail inject %x\n", env->cpu_index, irq);
    }

    return (env->interrupt_request & CPU_INTERRUPT_HARD) != 0;
}

void kvm_arch_update_regs_for_sipi(CPUState *env)
{
    printf("%s: no kvm-powerpc multi processor support yet!\n", __func__);
}

/* map dcr access to existing qemu dcr emulation */
int handle_powerpc_dcr_read(int vcpu, uint32_t dcrn, uint32_t *data)
{
    CPUState *env = qemu_kvm_cpu_env(vcpu);
    ppc_dcr_read(env->dcr_env, dcrn, data);
    return 0; /* XXX ignore failed DCR ops */
}

int handle_powerpc_dcr_write(int vcpu, uint32_t dcrn, uint32_t data)
{
    CPUState *env = qemu_kvm_cpu_env(vcpu);
    ppc_dcr_write(env->dcr_env, dcrn, data);
    return 0; /* XXX ignore failed DCR ops */
}

#endif
