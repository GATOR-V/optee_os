/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022-2023 NXP
 */

#ifndef __KERNEL_ARCH_SCALL_H
#define __KERNEL_ARCH_SCALL_H

#include <kernel/thread.h>
#include <riscv.h>
#include <types_ext.h>

static inline void scall_get_max_args(struct thread_scall_regs *regs,
				      size_t *scn, size_t *max_args)
{
	*scn = regs->t0;
	*max_args = regs->t1;
}

static inline void scall_set_retval(struct thread_scall_regs *regs,
				    uint32_t ret_val)
{
	/*
	 * a0 is restored verbatim to the user TA on return. The RISC-V
	 * psABI keeps a 32-bit value in a register sign-extended to XLEN,
	 * and TA code compares the full register against sign-extended
	 * constants (e.g. TEE_ERROR_* such as 0xffff0008). Sign-extend the
	 * result so those comparisons match; a plain uint32_t assignment
	 * would zero-extend and break them.
	 */
	regs->a0 = (unsigned long)(long)(int32_t)ret_val;
}

static inline void scall_set_sys_return_regs(struct thread_scall_regs *regs,
					     bool panic, uint32_t panic_code)
{
	regs->a1 = panic;
	regs->a2 = panic_code;
}

#endif /*__KERNEL_ARCH_SCALL_H*/
