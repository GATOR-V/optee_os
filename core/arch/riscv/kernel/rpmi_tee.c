// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2026 NXP
 *
 * TEE side of the RPMI TEE service group.
 *
 * OP-TEE is a TEE endpoint of the RPMI TEE framework implemented by the
 * M-mode firmware. The REE invokes the OP-TEE service with TEE_CALL
 * (see tee/optee_rpmi.h) and shares memory with memory parcels. OP-TEE
 * uses the same MPXY channel from its own harts:
 *
 * - TEE_EXIT returns control to the REE. As a framework specific
 *   extension its request data carries the service response of the
 *   TEE_CALL being completed, and its response carries the next TEE_CALL
 *   request delivered to this hart. Every hart of OP-TEE loops on it.
 *
 * - TEE_MEMORY_PARCEL_ACCEPT and TEE_MEMORY_PARCEL_RELEASE map and
 *   unmap the memory parcels created by the REE.
 *
 * Calls are dispatched to the regular OP-TEE ABI handlers: yielding
 * calls are OPTEE_ABI_CALL_WITH_REGD_ARG calls whose registered shared
 * memory is the memory parcel, and every exit of the ABI goes through
 * thread_return_to_udomain() which lands in thread_rpmi_tee_return().
 */

#include <assert.h>
#include <compiler.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/rpmi_tee.h>
#include <kernel/tee_misc.h>
#include <kernel/thread.h>
#include <kernel/thread_private_arch.h>
#include <mm/core_memprot.h>
#include <mm/mobj.h>
#include <sbi_mpxy.h>
#include <sbi_mpxy_rpmi.h>
#include <string.h>
#include <tee/entry_fast.h>
#include <tee/optee_abi.h>
#include <tee/optee_rpmi.h>
#include <tee/teeabi_opteed_macros.h>
#include <tee/teeabi_opteed.h>
#include <tee_api_defines.h>
#include <trace.h>
#include <util.h>

/* RPMI TEE service group */
#define RPMI_SRVGRP_TEE				0x0010
#define RPMI_TEE_SRV_EXIT			0x04
#define RPMI_TEE_SRV_MEMORY_PARCEL_ACCEPT	0x0a
#define RPMI_TEE_SRV_MEMORY_PARCEL_RELEASE	0x0b

#define RPMI_SUCCESS				0

#define RPMI_TEE_MEM_ACCESS_WRITE		BIT32(30)
#define RPMI_TEE_MEM_ACCESS_READ		BIT32(29)
#define RPMI_TEE_MEM_ACCESS_RW			(RPMI_TEE_MEM_ACCESS_READ | \
						 RPMI_TEE_MEM_ACCESS_WRITE)

#define RPMI_TEE_BLOCK_LOW_PFN_SHIFT		12
#define RPMI_TEE_BLOCK_LOW_COUNT_MASK		GENMASK_32(11, 0)

/* Largest parcel accepted, in 4kB pages */
#define RPMI_TEE_PARCEL_MAX_PAGES		1024

/* TEE_CALL request as delivered by the framework */
struct rpmi_tee_call {
	uint32_t sender_id;
	uint32_t target_id;
	uint8_t service[16];
	uint32_t data_len;
	uint8_t data[];
} __packed;

/* TEE_EXIT, framework specific request data */
struct rpmi_tee_exit_req {
	uint32_t rsp_len;
	struct optee_rpmi_msg rsp;
};

/* TEE_EXIT response: status followed by the next TEE_CALL request */
struct rpmi_tee_exit_rsp {
	int32_t status;
	struct rpmi_tee_call call;
	struct optee_rpmi_msg data;
} __packed;

struct rpmi_tee_parcel_accept_req {
	uint32_t acceptor_id;
	uint32_t access;
	uint32_t mem_parcel_id;
	uint32_t nonce;
	uint32_t creator_id;
	uint32_t creator_access;
	uint32_t flags;
	uint32_t address_high;
	uint32_t address_low;
	uint32_t max_pages;
	uint32_t other_cnt;
};

struct rpmi_tee_parcel_accept_rsp {
	int32_t status;
	uint32_t flags;
	uint32_t page_cnt;
	uint32_t block_cnt;
	uint32_t blocks[];	/* block_high[M], block_low[M] */
};

struct rpmi_tee_parcel_release_req {
	uint32_t mem_parcel_id;
	uint32_t flags;
	uint32_t endpoint_cnt;
};

struct rpmi_tee_parcel_release_rsp {
	int32_t status;
};

static const uint8_t optee_rpmi_service_uuid[16] = OPTEE_RPMI_SERVICE_UUID;

static struct sbi_mpxy_rpmi_channel *rpmi_tee_chan;
static uint32_t rpmi_tee_self_id;
/* Per hart TEE_EXIT buffers, used on the temporary stack */
static struct rpmi_tee_exit_req exit_req[CFG_TEE_CORE_NB_CORE];
static uint8_t exit_rsp[CFG_TEE_CORE_NB_CORE][sizeof(struct rpmi_tee_exit_rsp) +
					     64];

static struct sbi_mpxy_rpmi_channel *rpmi_tee_get_channel(void)
{
	struct sbi_mpxy_rpmi_context *ctx = sbi_mpxy_rpmi_ctx;
	uint32_t n = 0;

	if (rpmi_tee_chan)
		return rpmi_tee_chan;

	if (!ctx)
		panic("RPMI channels not probed");

	for (n = 0; n < ctx->channel_count; n++) {
		if (ctx->channels[n].rpmi_attrs.servicegroup_id ==
		    RPMI_SRVGRP_TEE) {
			rpmi_tee_chan = &ctx->channels[n];
			return rpmi_tee_chan;
		}
	}

	panic("no RPMI TEE channel");
}

static int rpmi_tee_send(uint32_t service_id, void *req, size_t req_len,
			 void *rsp, size_t rsp_size, size_t *rsp_len)
{
	struct sbi_mpxy_rpmi_message msg = { };
	int rc = 0;

	sbi_mpxy_rpmi_init_send_with_response(&msg, service_id, req, req_len,
					      rsp, rsp_size);
	rc = sbi_mpxy_rpmi_send_data(rpmi_tee_get_channel(), &msg);
	if (rc)
		return rc;
	if (msg.error)
		return msg.error;
	if (msg.data.response_len < sizeof(int32_t))
		return -1;
	if (rsp_len)
		*rsp_len = msg.data.response_len;

	return *(int32_t *)rsp;
}

static struct mobj *rpmi_tee_parcel_accept(uint64_t cookie,
					   unsigned int internal_offs)
{
	struct rpmi_tee_parcel_accept_req req = {
		.acceptor_id = rpmi_tee_self_id,
		.access = RPMI_TEE_MEM_ACCESS_RW,
		.mem_parcel_id = (uint32_t)cookie,
		.nonce = (uint32_t)(cookie >> 32),
		.max_pages = RPMI_TEE_PARCEL_MAX_PAGES,
	};
	struct rpmi_tee_parcel_accept_rsp *rsp = NULL;
	size_t rsp_size = sizeof(*rsp) + 2 * RPMI_TEE_PARCEL_MAX_PAGES *
			  sizeof(uint32_t);
	struct mobj *mobj = NULL;
	paddr_t *pages = NULL;
	size_t num_pages = 0;
	size_t rsp_len = 0;
	uint32_t n = 0;
	int rc = 0;

	rsp = calloc(1, rsp_size);
	if (!rsp)
		return NULL;

	rc = rpmi_tee_send(RPMI_TEE_SRV_MEMORY_PARCEL_ACCEPT, &req, sizeof(req),
			   rsp, rsp_size, &rsp_len);
	if (rc) {
		EMSG("PARCEL_ACCEPT %#"PRIx64": %d", cookie, rc);
		goto out;
	}
	if (rsp_len < sizeof(*rsp) + 2 * rsp->block_cnt * sizeof(uint32_t) ||
	    rsp->page_cnt > RPMI_TEE_PARCEL_MAX_PAGES || !rsp->page_cnt) {
		EMSG("PARCEL_ACCEPT %#"PRIx64": bad block list", cookie);
		goto out;
	}

	pages = calloc(rsp->page_cnt, sizeof(*pages));
	if (!pages)
		goto out;

	for (n = 0; n < rsp->block_cnt; n++) {
		uint32_t high = rsp->blocks[n];
		uint32_t low = rsp->blocks[rsp->block_cnt + n];
		uint64_t pfn = ((uint64_t)high << 20) |
			       (low >> RPMI_TEE_BLOCK_LOW_PFN_SHIFT);
		uint32_t count = (low & RPMI_TEE_BLOCK_LOW_COUNT_MASK) + 1;

		while (count--) {
			if (num_pages >= rsp->page_cnt)
				goto out;
			pages[num_pages++] = pfn << SMALL_PAGE_SHIFT;
			pfn++;
		}
	}

	mobj = mobj_reg_shm_alloc(pages, num_pages, internal_offs, cookie);
	if (mobj)
		mobj_reg_shm_unguard(mobj);
out:
	free(pages);
	free(rsp);
	return mobj;
}

struct mobj *rpmi_tee_mobj_get_by_cookie(uint64_t cookie,
					 unsigned int internal_offs)
{
	struct mobj *mobj = mobj_reg_shm_get_by_cookie(cookie);

	if (mobj)
		return mobj;

	mobj = rpmi_tee_parcel_accept(cookie, internal_offs);
	if (!mobj)
		return NULL;

	/*
	 * mobj_reg_shm_alloc() returns a reference which belongs to the
	 * registry, get the caller's own reference like for an already
	 * registered parcel.
	 */
	return mobj_reg_shm_get_by_cookie(cookie);
}

TEE_Result rpmi_tee_parcel_release(uint64_t cookie)
{
	struct rpmi_tee_parcel_release_req req = {
		.mem_parcel_id = (uint32_t)cookie,
	};
	struct rpmi_tee_parcel_release_rsp rsp = { };
	TEE_Result res = TEE_SUCCESS;
	int rc = 0;

	res = mobj_reg_shm_release_by_cookie(cookie);
	if (res)
		return res;

	rc = rpmi_tee_send(RPMI_TEE_SRV_MEMORY_PARCEL_RELEASE, &req,
			   sizeof(req), &rsp, sizeof(rsp), NULL);
	if (rc) {
		EMSG("PARCEL_RELEASE %#"PRIx64": %d", cookie, rc);
		return TEE_ERROR_GENERIC;
	}

	return TEE_SUCCESS;
}

/* Translate the TEEABI_OPTEED_RETURN_* exit into a TEE_CALL response */
static bool make_exit_response(struct optee_rpmi_msg *rsp, unsigned long a0,
			       unsigned long a1, unsigned long a4)
{
	memset(rsp, 0, sizeof(*rsp));

	switch (a0) {
	case TEEABI_OPTEED_RETURN_ENTRY_DONE:
	case TEEABI_OPTEED_RETURN_ON_DONE:
		/* Boot completed, no call in progress */
		return false;
	case TEEABI_OPTEED_RETURN_CALL_DONE:
		break;
	default:
		panic("unexpected return to REE");
	}

	if (OPTEE_ABI_RETURN_IS_RPC(a1)) {
		rsp->w[4] = a4;
		switch (a1) {
		case OPTEE_ABI_RETURN_RPC_FOREIGN_INTR:
			rsp->w[1] = OPTEE_RPMI_YIELDING_CALL_RETURN_INTERRUPT;
			break;
		case OPTEE_ABI_RETURN_RPC_CMD:
			rsp->w[1] = OPTEE_RPMI_YIELDING_CALL_RETURN_RPC_CMD;
			break;
		default:
			/* Register based RPC allocations are not used */
			EMSG("Unsupported RPC %#lx", a1);
			rsp->w[0] = TEE_ERROR_NOT_SUPPORTED;
			rsp->w[1] = OPTEE_RPMI_YIELDING_CALL_RETURN_DONE;
		}
		return true;
	}

	switch (a1) {
	case OPTEE_ABI_RETURN_OK:
		rsp->w[0] = TEE_SUCCESS;
		break;
	case OPTEE_ABI_RETURN_ETHREAD_LIMIT:
		rsp->w[0] = TEE_ERROR_BUSY;
		break;
	default:
		rsp->w[0] = TEE_ERROR_BAD_PARAMETERS;
	}
	rsp->w[1] = OPTEE_RPMI_YIELDING_CALL_RETURN_DONE;

	return true;
}

/*
 * Per-hart pending dispatch, filled by rpmi_tee_next() and consumed by the
 * assembly trampoline thread_rpmi_tee_dispatch().
 */
#define RPMI_TEE_DISPATCH_STD	0	/* via vector_std_abi_entry */
#define RPMI_TEE_DISPATCH_FAST	1	/* via vector_fast_abi_entry */

/* Per-hart ABI args (a0..a3) for the pending vector entry */
static uint32_t dispatch_args[CFG_TEE_CORE_NB_CORE][4];

/* Base of this hart's dispatch args, for the asm trampoline */
uint32_t *rpmi_tee_dispatch_args(void)
{
	return dispatch_args[get_core_pos()];
}

/* Translate a fast (non-yielding) OP-TEE service call inline */
static void handle_fast_optee(struct optee_rpmi_msg *req,
			      struct optee_rpmi_msg *rsp)
{
	uint64_t cookie = 0;

	memset(rsp, 0, sizeof(*rsp));

	switch (req->w[0]) {
	case OPTEE_RPMI_GET_API_VERSION:
		rsp->w[0] = OPTEE_RPMI_VERSION_MAJOR;
		rsp->w[1] = OPTEE_RPMI_VERSION_MINOR;
		break;
	case OPTEE_RPMI_GET_OS_VERSION:
		rsp->w[0] = CFG_OPTEE_REVISION_MAJOR;
		rsp->w[1] = CFG_OPTEE_REVISION_MINOR;
		rsp->w[2] = TEE_IMPL_GIT_SHA1 >> 32;
		break;
	case OPTEE_RPMI_EXCHANGE_CAPABILITIES:
		rsp->w[0] = TEE_SUCCESS;
		rsp->w[1] = OPTEE_RPMI_SEC_CAP_ARG_OFFSET;
		if (IS_ENABLED(CFG_RPMB_FS) && IS_ENABLED(CFG_CORE_RPMB_PROBE))
			rsp->w[1] |= OPTEE_RPMI_SEC_CAP_RPMB_PROBE;
		rsp->w[2] = THREAD_RPC_MAX_NUM_PARAMS;
		break;
	case OPTEE_RPMI_UNREGISTER_SHM:
		cookie = reg_pair_to_64(req->w[2], req->w[1]);
		rsp->w[0] = rpmi_tee_parcel_release(cookie);
		break;
	default:
		rsp->w[0] = TEE_ERROR_NOT_SUPPORTED;
	}
}

/*
 * Send the result of the call this hart just finished as the TEE_EXIT
 * request data, then fetch the next TEE_CALL. Fast OP-TEE calls are
 * answered inline in a loop (they need no thread); the first yielding
 * call found is prepared in dispatch[] for thread_rpmi_tee_dispatch() to
 * run through the vector entries, which allocate a thread. Returns a
 * pointer to the pending dispatch.
 *
 * @a0/@a1/@a4 are the TEEABI_OPTEED_RETURN_* arguments of the ABI exit
 * that led here (see make_exit_response()).
 */
int rpmi_tee_next(unsigned long a0, unsigned long a1, unsigned long a4)
{
	size_t pos = get_core_pos();
	struct rpmi_tee_exit_req *req = &exit_req[pos];
	struct rpmi_tee_exit_rsp *rsp = (void *)exit_rsp[pos];
	uint32_t *d = dispatch_args[pos];
	struct optee_rpmi_msg call = { };
	struct mobj *mobj = NULL;
	size_t rsp_len = 0;
	int rc = 0;

	assert(pos < CFG_TEE_CORE_NB_CORE);
	rpmi_tee_get_channel();

	if (make_exit_response(&req->rsp, a0, a1, a4))
		req->rsp_len = sizeof(req->rsp);
	else
		req->rsp_len = 0;

	while (true) {
		rc = rpmi_tee_send(RPMI_TEE_SRV_EXIT, req,
				   sizeof(req->rsp_len) + req->rsp_len,
				   rsp, sizeof(exit_rsp[pos]), &rsp_len);
		if (rc)
			panic("TEE_EXIT failed");
		if (rsp_len < sizeof(*rsp) ||
		    rsp->call.data_len < sizeof(call)) {
			EMSG("Short TEE_CALL %zu/%"PRIu32, rsp_len,
			     rsp->call.data_len);
			req->rsp_len = 0;
			continue;
		}
		if (memcmp(rsp->call.service, optee_rpmi_service_uuid,
			   sizeof(optee_rpmi_service_uuid))) {
			memset(&req->rsp, 0, sizeof(req->rsp));
			req->rsp.w[0] = TEE_ERROR_NOT_SUPPORTED;
			req->rsp_len = sizeof(req->rsp);
			continue;
		}
		rpmi_tee_self_id = rsp->call.target_id;
		memcpy(&call, &rsp->data, sizeof(call));

		if (!(call.w[0] & BIT32(OPTEE_RPMI_YIELDING_CALL_BIT))) {
			/* Fast call: answer inline, loop for the next one */
			handle_fast_optee(&call, &req->rsp);
			req->rsp_len = sizeof(req->rsp);
			continue;
		}

		/* Yielding call: run it in a thread via the vector entry */
		switch (call.w[0]) {
		case OPTEE_RPMI_YIELDING_CALL_WITH_ARG:
			/*
			 * Accept the memory parcel holding the argument and
			 * register it as shared memory before entering the
			 * ABI, which only looks registered cookies up.
			 */
			mobj = rpmi_tee_mobj_get_by_cookie(reg_pair_to_64(call.w[2],
								    call.w[1]), 0);
			if (!mobj) {
				memset(&req->rsp, 0, sizeof(req->rsp));
				req->rsp.w[0] = TEE_ERROR_BAD_PARAMETERS;
				req->rsp.w[1] =
					OPTEE_RPMI_YIELDING_CALL_RETURN_DONE;
				req->rsp_len = sizeof(req->rsp);
				continue;
			}
			mobj_put(mobj);
			d[0] = OPTEE_ABI_CALL_WITH_REGD_ARG;
			d[1] = call.w[2];	/* cookie high */
			d[2] = call.w[1];	/* cookie low */
			d[3] = call.w[3];	/* offset */
			return RPMI_TEE_DISPATCH_STD;
		case OPTEE_RPMI_YIELDING_CALL_RESUME:
			d[0] = OPTEE_ABI_CALL_RETURN_FROM_RPC;
			d[1] = 0;
			d[2] = 0;
			d[3] = call.w[4];	/* resume info */
			return RPMI_TEE_DISPATCH_STD;
		default:
			memset(&req->rsp, 0, sizeof(req->rsp));
			req->rsp.w[0] = TEE_ERROR_NOT_SUPPORTED;
			req->rsp.w[1] = OPTEE_RPMI_YIELDING_CALL_RETURN_DONE;
			req->rsp_len = sizeof(req->rsp);
			continue;
		}
	}
}
