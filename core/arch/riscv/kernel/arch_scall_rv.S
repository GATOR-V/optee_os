/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022-2023 NXP
 */
#include "tee_syscall_numbers.h"
#include "trace_levels.h"
#include <asm.S>
#include <generated/asm-defines.h>
#include <kernel/thread.h>
#include <riscv.h>
#include <riscv_macros.S>
#include <tee_api_defines.h>

/*
 * uint32_t scall_do_call(struct thread_scall_regs *regs, syscall_t func);
 *
 * Called from scall_handle_user_ta()
 */
FUNC scall_do_call , :
	addi	sp, sp, -16

	/* Save scall regs to t0 */
	mv	t0, a0

	/* Save func to t1 */
	mv	t1, a1

	/* Push return address to stack */
	store_xregs sp, 0, 1

	/* Load arguments to function */
	load_xregs a0, THREAD_SCALL_REG_A0, 10, 17

	/* Call the syscall function */
	jalr	t1

	/* Pop return address from stack */
	load_xregs	sp, 0, 1

	addi	sp, sp, 16
	/*
	 * scall_do_call() is declared to return uint32_t. The RISC-V psABI
	 * requires a 32-bit return value to be sign-extended to XLEN, and both
	 * TA and core code compare the full a0 register against sign-extended
	 * TEE_Result constants (e.g. TEE_ERROR_CANCEL, 0xffff0002). Some
	 * syscalls return a0 through paths that do not honour that rule, most
	 * notably syscall_sys_return()->scall_sys_return_helper() (the value a
	 * TA passes to utee_return()) and syscall_invoke_ta_command() (the
	 * result unwound from a nested TA). scall_handle_user_ta() then stores
	 * this value via scall_set_retval(), whose sign-extension the compiler
	 * elides because it assumes our uint32_t return is already extended.
	 * Force the sign-extension here so it holds for every syscall.
	 */
	sext.w	a0, a0
	ret
END_FUNC scall_do_call

/*
 * void syscall_sys_return(uint32_t ret);
 */
FUNC syscall_sys_return , :
	li	a1, 0  /* panic = false */
	li	a2, 0  /* panic_code = 0 */
	mv	a3, t0 /* pointer to struct thread_scall_regs */
	j	scall_sys_return_helper
END_FUNC syscall_sys_return

/*
 * void syscall_panic(uint32_t code);
 */
FUNC syscall_panic , :
	li	a1, 1  /* panic = true */
	mv	a2, a0  /* code */
	li	a0, TEE_ERROR_TARGET_DEAD
	mv	a3, t0 /* pointer to struct thread_scall_regs */
	j	scall_sys_return_helper
END_FUNC syscall_panic
