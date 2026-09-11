/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2026 NXP
 */
#ifndef __KERNEL_RPMI_TEE_H
#define __KERNEL_RPMI_TEE_H

#include <compiler.h>
#include <mm/mobj.h>
#include <stdint.h>

/*
 * TEE side of the RPMI TEE service group (RPMI v2.0), see rpmi_tee.c.
 * The REE side lives in the Linux RPMI TEE framework driver and OP-TEE
 * driver, the framework in OpenSBI (fdt_mpxy_rpmi_tee.c).
 */

/*
 * Get the MOBJ of a memory parcel shared by the REE, accepting the
 * parcel with the framework on first use. @cookie is the parcel handle
 * (NONCE << 32 | MEM_PARCEL_ID) used as global ID in the OP-TEE message
 * protocol, @internal_offs the offset of the memory in its first page.
 */
struct mobj *rpmi_tee_mobj_get_by_cookie(uint64_t cookie,
					 unsigned int internal_offs);

/* Release a memory parcel with the framework after the MOBJ is gone */
TEE_Result rpmi_tee_parcel_release(uint64_t cookie);

/*
 * Return to the REE through the framework, called by
 * thread_return_to_udomain() with the TEEABI_OPTEED_RETURN_* arguments.
 * Never returns: the next TEE_CALL is dispatched from here.
 */
void __noreturn thread_rpmi_tee_return(unsigned long a0, unsigned long a1,
				       unsigned long a2, unsigned long a3,
				       unsigned long a4, unsigned long a5);

#endif /*__KERNEL_RPMI_TEE_H*/
