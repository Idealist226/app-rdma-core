/*
 * Copyright (c) 2005 Topspin Communications.  All rights reserved.
 * Copyright (c) 2006, 2007 Cisco Systems, Inc.  All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define _GNU_SOURCE
#include <config.h>

#include <endian.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <linux/ip.h>
#include <dirent.h>
#include <netinet/in.h>

#include <util/compiler.h>
#include <util/symver.h>
#include <infiniband/cmd_write.h>

#include "ibverbs.h"
#include <net/if.h>
#include <net/if_arp.h>
#include "neigh.h"
#include <fcntl.h>
#include <sys/mman.h>
#include "freeflow.h"
#include "verbs.h"

#undef ibv_query_port

int __attribute__((const)) ibv_rate_to_mult(enum ibv_rate rate)
{
	switch (rate) {
	case IBV_RATE_2_5_GBPS: return  1;
	case IBV_RATE_5_GBPS:   return  2;
	case IBV_RATE_10_GBPS:  return  4;
	case IBV_RATE_20_GBPS:  return  8;
	case IBV_RATE_30_GBPS:  return 12;
	case IBV_RATE_40_GBPS:  return 16;
	case IBV_RATE_60_GBPS:  return 24;
	case IBV_RATE_80_GBPS:  return 32;
	case IBV_RATE_120_GBPS: return 48;
	case IBV_RATE_28_GBPS:  return 11;
	case IBV_RATE_50_GBPS:  return 20;
	case IBV_RATE_400_GBPS: return 160;
	case IBV_RATE_600_GBPS: return 240;
	default:           return -1;
	}
}

enum ibv_rate __attribute__((const)) mult_to_ibv_rate(int mult)
{
	switch (mult) {
	case 1:  return IBV_RATE_2_5_GBPS;
	case 2:  return IBV_RATE_5_GBPS;
	case 4:  return IBV_RATE_10_GBPS;
	case 8:  return IBV_RATE_20_GBPS;
	case 12: return IBV_RATE_30_GBPS;
	case 16: return IBV_RATE_40_GBPS;
	case 24: return IBV_RATE_60_GBPS;
	case 32: return IBV_RATE_80_GBPS;
	case 48: return IBV_RATE_120_GBPS;
	case 11: return IBV_RATE_28_GBPS;
	case 20: return IBV_RATE_50_GBPS;
	case 160: return IBV_RATE_400_GBPS;
	case 240: return IBV_RATE_600_GBPS;
	default: return IBV_RATE_MAX;
	}
}

int  __attribute__((const)) ibv_rate_to_mbps(enum ibv_rate rate)
{
	switch (rate) {
	case IBV_RATE_2_5_GBPS: return 2500;
	case IBV_RATE_5_GBPS:   return 5000;
	case IBV_RATE_10_GBPS:  return 10000;
	case IBV_RATE_20_GBPS:  return 20000;
	case IBV_RATE_30_GBPS:  return 30000;
	case IBV_RATE_40_GBPS:  return 40000;
	case IBV_RATE_60_GBPS:  return 60000;
	case IBV_RATE_80_GBPS:  return 80000;
	case IBV_RATE_120_GBPS: return 120000;
	case IBV_RATE_14_GBPS:  return 14062;
	case IBV_RATE_56_GBPS:  return 56250;
	case IBV_RATE_112_GBPS: return 112500;
	case IBV_RATE_168_GBPS: return 168750;
	case IBV_RATE_25_GBPS:  return 25781;
	case IBV_RATE_100_GBPS: return 103125;
	case IBV_RATE_200_GBPS: return 206250;
	case IBV_RATE_300_GBPS: return 309375;
	case IBV_RATE_28_GBPS:  return 28125;
	case IBV_RATE_50_GBPS:  return 53125;
	case IBV_RATE_400_GBPS: return 425000;
	case IBV_RATE_600_GBPS: return 637500;
	default:               return -1;
	}
}

enum ibv_rate __attribute__((const)) mbps_to_ibv_rate(int mbps)
{
	switch (mbps) {
	case 2500:   return IBV_RATE_2_5_GBPS;
	case 5000:   return IBV_RATE_5_GBPS;
	case 10000:  return IBV_RATE_10_GBPS;
	case 20000:  return IBV_RATE_20_GBPS;
	case 30000:  return IBV_RATE_30_GBPS;
	case 40000:  return IBV_RATE_40_GBPS;
	case 60000:  return IBV_RATE_60_GBPS;
	case 80000:  return IBV_RATE_80_GBPS;
	case 120000: return IBV_RATE_120_GBPS;
	case 14062:  return IBV_RATE_14_GBPS;
	case 56250:  return IBV_RATE_56_GBPS;
	case 112500: return IBV_RATE_112_GBPS;
	case 168750: return IBV_RATE_168_GBPS;
	case 25781:  return IBV_RATE_25_GBPS;
	case 103125: return IBV_RATE_100_GBPS;
	case 206250: return IBV_RATE_200_GBPS;
	case 309375: return IBV_RATE_300_GBPS;
	case 28125:  return IBV_RATE_28_GBPS;
	case 53125:  return IBV_RATE_50_GBPS;
	case 425000: return IBV_RATE_400_GBPS;
	case 637500: return IBV_RATE_600_GBPS;
	default:     return IBV_RATE_MAX;
	}
}

LATEST_SYMVER_FUNC(ibv_query_device, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_context *context,
		   struct ibv_device_attr *device_attr)
{
	return get_ops(context)->query_device(context, device_attr);
}

int __lib_query_port(struct ibv_context *context, uint8_t port_num,
		     struct ibv_port_attr *port_attr, size_t port_attr_len)
{
	/* Don't expose this mess to the provider, provide a large enough
	 * temporary buffer if the user buffer is too small.
	 */
	if (port_attr_len < sizeof(struct ibv_port_attr)) {
		struct ibv_port_attr tmp_attr = {};
		int rc;

		rc = get_ops(context)->query_port(context, port_num,
						    &tmp_attr);
		if (rc)
			return rc;

		memcpy(port_attr, &tmp_attr, port_attr_len);
		return 0;
	}

	memset(port_attr, 0, port_attr_len);
	return get_ops(context)->query_port(context, port_num, port_attr);
}

struct _compat_ibv_port_attr {
	enum ibv_port_state state;
	enum ibv_mtu max_mtu;
	enum ibv_mtu active_mtu;
	int gid_tbl_len;
	uint32_t port_cap_flags;
	uint32_t max_msg_sz;
	uint32_t bad_pkey_cntr;
	uint32_t qkey_viol_cntr;
	uint16_t pkey_tbl_len;
	uint16_t lid;
	uint16_t sm_lid;
	uint8_t lmc;
	uint8_t max_vl_num;
	uint8_t sm_sl;
	uint8_t subnet_timeout;
	uint8_t init_type_reply;
	uint8_t active_width;
	uint8_t active_speed;
	uint8_t phys_state;
	uint8_t link_layer;
	uint8_t flags;
};

static void releaseShm(void *addr, size_t length, int fd)
{
	if (munmap(addr, length) == -1)
		printf("munmap: Error unmap %p\n", addr);
	if (close(fd) == -1)
		printf("munmap: Error close %d\n", fd);
	// if (shm_unlink(shm_name) == -1)
	// 	printf("shm_unlink: Error removing %s\n", shm_name);
}

LATEST_SYMVER_FUNC(ibv_query_port, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_context *context, uint8_t port_num,
		   struct _compat_ibv_port_attr *port_attr)
{
	if (PRINT_LOG) {
		printf("#### ibv_query_port ####\n");
		fflush(stdout);
	}

	struct IBV_QUERY_PORT_REQ req;
	req.port_num = port_num;

	struct IBV_QUERY_PORT_RSP rsp;
	int rsp_size;
	request_router(IBV_QUERY_PORT, &req, &rsp, &rsp_size);
	
	memcpy(port_attr, &rsp.port_attr, sizeof(struct ibv_port_attr));	

	if (PRINT_LOG) {
		printf("state=%d, max_mtu=%d, active_mtu=%d\n", port_attr->state, port_attr->max_mtu, port_attr->active_mtu);
		fflush(stdout);
	}
	
	return 0;
}

LATEST_SYMVER_FUNC(ibv_query_gid, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_context *context, uint8_t port_num,
		   int index, union ibv_gid *gid)
{
	struct verbs_device *verbs_device = verbs_get_device(context->device);
	char attr[41];
	uint16_t val;
	int i;

	if (ibv_read_ibdev_sysfs_file(attr, sizeof(attr), verbs_device->sysfs,
				      "ports/%d/gids/%d", port_num, index) < 0)
		return -1;

	for (i = 0; i < 8; ++i) {
		if (sscanf(attr + i * 5, "%hx", &val) != 1)
			return -1;
		gid->raw[i * 2    ] = val >> 8;
		gid->raw[i * 2 + 1] = val & 0xff;
	}

	return 0;
}

LATEST_SYMVER_FUNC(ibv_query_pkey, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_context *context, uint8_t port_num,
		   int index, __be16 *pkey)
{
	struct verbs_device *verbs_device = verbs_get_device(context->device);
	char attr[8];
	uint16_t val;

	if (ibv_read_ibdev_sysfs_file(attr, sizeof(attr), verbs_device->sysfs,
				      "ports/%d/pkeys/%d", port_num, index) < 0)
		return -1;

	if (sscanf(attr, "%hx", &val) != 1)
		return -1;

	*pkey = htobe16(val);
	return 0;
}

LATEST_SYMVER_FUNC(ibv_get_pkey_index, 1_5, "IBVERBS_1.5",
		   int,
		   struct ibv_context *context, uint8_t port_num, __be16 pkey)
{
	__be16 pkey_i;
	int i, ret;

	for (i = 0; ; i++) {
		ret = ibv_query_pkey(context, port_num, i, &pkey_i);
		if (ret < 0)
			return ret;
		if (pkey == pkey_i)
			return i;
	}
}

LATEST_SYMVER_FUNC(ibv_alloc_pd, 1_1, "IBVERBS_1.1",
		   struct ibv_pd *,
		   struct ibv_context *context)
{
	struct ibv_pd *pd = calloc(1, sizeof(struct ibv_pd));

	if (PRINT_LOG) {
		printf("#### ibv_alloc_pd ####\n");
		fflush(stdout);	
	}

	struct IBV_ALLOC_PD_RSP rsp;
	int rsp_size;
	request_router(IBV_ALLOC_PD, NULL, &rsp, &rsp_size);

	pd->handle  = rsp.pd_handle;
	pd->context = context;

	if (PRINT_LOG) {
		printf("PD handle = %d\n", pd->handle);
		fflush(stdout);
	}

	return pd;
}

LATEST_SYMVER_FUNC(ibv_dealloc_pd, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_pd *pd)
{
	if (PRINT_LOG) {
		printf("#### ibv_dealloc_pd ####\n");
		fflush(stdout);
	}

	struct IBV_DEALLOC_PD_REQ req;
	req.pd_handle = pd->handle;

	struct IBV_DEALLOC_PD_RSP rsp;
	int rsp_size;
	request_router(IBV_DEALLOC_PD, &req, &rsp, &rsp_size);

	free(pd);
	return 0;
}

#undef ibv_reg_mr
LATEST_SYMVER_FUNC(ibv_reg_mr, 1_1, "IBVERBS_1.1",
		   struct ibv_mr *,
		   struct ibv_pd *pd, void *addr,
		   size_t length, int access)
{
	struct ibv_mr *mr = calloc(1, sizeof(struct ibv_mr));
	struct mr_shm *p;
	
	if (PRINT_LOG) {
		printf("#### ibv_reg_mr ####\n");
		fflush(stdout);
	}

	struct IBV_REG_MR_REQ req_body;
	req_body.pd_handle = pd->handle;
	req_body.mem_size = length;
	req_body.access_flags = access;
	req_body.addr = addr;
	req_body.shm_name[0] = '\0';
	struct IBV_REG_MR_RSP rsp;
	int rsp_size;
	request_router(IBV_REG_MR, &req_body, &rsp, &rsp_size);

	mr->handle  = rsp.handle;
	mr->lkey    = rsp.lkey;
	mr->rkey    = rsp.rkey;
	strcpy(mr->shm_name, rsp.shm_name);

	// FreeFlow: mounting shared memory from router.
	mr->shm_fd = shm_open(mr->shm_name, O_CREAT | O_RDWR, 0666);
	if (ftruncate(mr->shm_fd, length)) {
		printf("[Error] Fail to mount shm %s\n", rsp.shm_name);
		fflush(stdout);
	}

	char *membuff = (char *)malloc(length);
	memcpy(membuff, addr, length);

	int is_align = (long)addr % (4 * 1024) == 0 ? 1 : 0;
	if (is_align)
		mr->shm_ptr = mmap(addr, length, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED | MAP_LOCKED, mr->shm_fd, 0); 
	else
		mr->shm_ptr = mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, mr->shm_fd, 0); 

	if (mr->shm_ptr == MAP_FAILED){
		printf("mmap failed in reg mr.\n");
		fflush(stdout);
	} else {
		memcpy(mr->shm_ptr, membuff, length);
		if (PRINT_LOG) {
			printf("mmap succeed in reg mr.\n");
		}

		p = (struct mr_shm*)mempool_insert(map_lkey_to_mrshm, mr->lkey);
		p->mr = addr;
		p->shm_ptr = mr->shm_ptr;

		if (PRINT_LOG) {
			printf("@@@@@@@@ lkey=%x, addr=%p, shm_prt=%p\n", mr->lkey, addr, mr->shm_ptr);
			fflush(stdout);
		}

		/*
		hashmap_put(map_lkey_to_shm_ptr, key, (void*)(mr->shm_ptr));
		fflush(stdout);
		*/	
	}

	mr->context = pd->context;
	mr->pd = pd;
	mr->addr = addr;
	mr->length = length;
	free(membuff);

	return mr;
}

#undef ibv_reg_mr_iova
struct ibv_mr *ibv_reg_mr_iova(struct ibv_pd *pd, void *addr, size_t length,
			       uint64_t iova, int access)
{
	struct ibv_mr *mr;

	if (ibv_dontfork_range(addr, length))
		return NULL;

	mr = get_ops(pd->context)->reg_mr(pd, addr, length, iova, access);
	if (mr) {
		mr->context = pd->context;
		mr->pd      = pd;
		mr->addr    = addr;
		mr->length  = length;
	} else
		ibv_dofork_range(addr, length);

	return mr;
}

struct ibv_mr *ibv_reg_mr_iova2(struct ibv_pd *pd, void *addr, size_t length,
				uint64_t iova, unsigned int access)
{
	struct verbs_device *device = verbs_get_device(pd->context->device);

	if (!(device->core_support & IB_UVERBS_CORE_SUPPORT_OPTIONAL_MR_ACCESS))
		access &= ~IBV_ACCESS_OPTIONAL_RANGE;

	return ibv_reg_mr_iova(pd, addr, length, iova, access);
}

LATEST_SYMVER_FUNC(ibv_rereg_mr, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_mr *mr, int flags,
		   struct ibv_pd *pd, void *addr,
		   size_t length, int access)
{
	int dofork_onfail = 0;
	int err;
	void *old_addr;
	size_t old_len;

	if (verbs_get_mr(mr)->mr_type != IBV_MR_TYPE_MR) {
		errno = EINVAL;
		return IBV_REREG_MR_ERR_INPUT;
	}

	if (flags & ~IBV_REREG_MR_FLAGS_SUPPORTED) {
		errno = EINVAL;
		return IBV_REREG_MR_ERR_INPUT;
	}

	if ((flags & IBV_REREG_MR_CHANGE_TRANSLATION) &&
	    (!length || !addr)) {
		errno = EINVAL;
		return IBV_REREG_MR_ERR_INPUT;
	}

	if (access && !(flags & IBV_REREG_MR_CHANGE_ACCESS)) {
		errno = EINVAL;
		return IBV_REREG_MR_ERR_INPUT;
	}

	if (flags & IBV_REREG_MR_CHANGE_TRANSLATION) {
		err = ibv_dontfork_range(addr, length);
		if (err)
			return IBV_REREG_MR_ERR_DONT_FORK_NEW;
		dofork_onfail = 1;
	}

	old_addr = mr->addr;
	old_len = mr->length;
	err = get_ops(mr->context)->rereg_mr(verbs_get_mr(mr),
					     flags, pd, addr,
					     length, access);
	if (!err) {
		if (flags & IBV_REREG_MR_CHANGE_PD)
			mr->pd = pd;
		if (flags & IBV_REREG_MR_CHANGE_TRANSLATION) {
			mr->addr    = addr;
			mr->length  = length;
			err = ibv_dofork_range(old_addr, old_len);
			if (err)
				return IBV_REREG_MR_ERR_DO_FORK_OLD;
		}
	} else {
		err = IBV_REREG_MR_ERR_CMD;
		if (dofork_onfail) {
			if (ibv_dofork_range(addr, length))
				err = IBV_REREG_MR_ERR_CMD_AND_DO_FORK_NEW;
		}
	}

	return err;
}

LATEST_SYMVER_FUNC(ibv_dereg_mr, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_mr *mr)
{
	if (PRINT_LOG) {
		printf("#### ibv_dereg_mr ####\n");
		fflush(stdout);	
	}

	struct IBV_DEREG_MR_REQ req_body;
	req_body.handle = mr->handle;
	
	struct IBV_DEREG_MR_RSP rsp;
	int rsp_size;
	request_router(IBV_DEREG_MR, &req_body, &rsp, &rsp_size);	

	shm_unlink(mr->shm_name);
	mempool_del(map_lkey_to_mrshm, mr->lkey);

	return rsp.ret;
}

struct ibv_comp_channel *ibv_create_comp_channel(struct ibv_context *context)
{
	struct ibv_create_comp_channel req;
	struct ib_uverbs_create_comp_channel_resp resp;
	struct ibv_comp_channel            *channel;

	channel = malloc(sizeof *channel);
	if (!channel)
		return NULL;

	req.core_payload = (struct ib_uverbs_create_comp_channel){};
	if (execute_cmd_write(context, IB_USER_VERBS_CMD_CREATE_COMP_CHANNEL,
			      &req, sizeof(req), &resp, sizeof(resp))) {
		free(channel);
		return NULL;
	}

	channel->context = context;
	channel->fd      = resp.fd;
	channel->refcnt  = 0;

	return channel;
}

int ibv_destroy_comp_channel(struct ibv_comp_channel *channel)
{
	struct ibv_context *context;
	int ret;

	context = channel->context;
	pthread_mutex_lock(&context->mutex);

	if (channel->refcnt) {
		ret = EBUSY;
		goto out;
	}

	close(channel->fd);
	free(channel);
	ret = 0;

out:
	pthread_mutex_unlock(&context->mutex);

	return ret;
}

LATEST_SYMVER_FUNC(ibv_create_cq, 1_1, "IBVERBS_1.1",
		   struct ibv_cq *,
		   struct ibv_context *context, int cqe, void *cq_context,
		   struct ibv_comp_channel *channel, int comp_vector)
{
	struct ibv_cq *cq = calloc(1, sizeof(struct ibv_cq));

	if (PRINT_LOG) {
		printf("#### ibv_create_cq ####\n");
		fflush(stdout);
	}

	struct IBV_CREATE_CQ_REQ req_body;
	req_body.channel_fd = channel ? comp_channel_map[channel->fd] : -1;
	req_body.comp_vector = comp_vector;
	req_body.cqe = cqe;
	struct IBV_CREATE_CQ_RSP rsp;
	int rsp_size;
	request_router(IBV_CREATE_CQ, &req_body, &rsp, &rsp_size);

	if (PRINT_LOG) {
		printf("CQ return from router: cqe=%d handle = %d\n", rsp.cqe, rsp.handle);
	}
	
	cq->handle  = rsp.handle;
	cq->cqe     = rsp.cqe;
	verbs_init_cq(cq, context, channel, cq_context);

	if (cqe > WR_QUEUE_SIZE) {
		printf("Freeflow WARNING: attempt to create a very large CQ. This may cause problems.\n");
	}
	map_cq_to_wr_queue[cq->handle] = (struct wr_queue*)malloc(sizeof(struct wr_queue));
	map_cq_to_wr_queue[cq->handle]->queue = (struct wr*)malloc(sizeof(struct wr) * WR_QUEUE_SIZE);
	map_cq_to_wr_queue[cq->handle]->head = 0;
	map_cq_to_wr_queue[cq->handle]->tail = 0;
	pthread_spin_init(&(map_cq_to_wr_queue[cq->handle]->head_lock), PTHREAD_PROCESS_PRIVATE);
	pthread_spin_init(&(map_cq_to_wr_queue[cq->handle]->tail_lock), PTHREAD_PROCESS_PRIVATE);

	return cq;
}

LATEST_SYMVER_FUNC(ibv_resize_cq, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_cq *cq, int cqe)
{
	return get_ops(cq->context)->resize_cq(cq, cqe);
}

LATEST_SYMVER_FUNC(ibv_destroy_cq, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_cq *cq)
{
	if (PRINT_LOG) {
		printf("#### ibv_destroy_cq ####\n");
		fflush(stdout);
	}

	struct IBV_DESTROY_CQ_REQ req_body;
	req_body.cq_handle = cq->handle;
	
	struct IBV_DESTROY_CQ_RSP rsp;
	int rsp_size;
	request_router(IBV_DESTROY_CQ, &req_body, &rsp, &rsp_size);

	struct ibv_comp_channel *channel = cq->channel;
	if (channel) {
		if (!rsp.ret) {
			pthread_mutex_lock(&channel->context->mutex);
			--channel->refcnt;
			pthread_mutex_unlock(&channel->context->mutex);
		}
	}

	return rsp.ret;
}

LATEST_SYMVER_FUNC(ibv_get_cq_event, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_comp_channel *channel,
		   struct ibv_cq **cq, void **cq_context)
{
	struct ib_uverbs_comp_event_desc ev;

	if (read(channel->fd, &ev, sizeof ev) != sizeof ev)
		return -1;

	*cq         = (struct ibv_cq *) (uintptr_t) ev.cq_handle;
	*cq_context = (*cq)->cq_context;

	get_ops((*cq)->context)->cq_event(*cq);

	return 0;
}

LATEST_SYMVER_FUNC(ibv_ack_cq_events, 1_1, "IBVERBS_1.1",
		   void,
		   struct ibv_cq *cq, unsigned int nevents)
{
	pthread_mutex_lock(&cq->mutex);
	cq->comp_events_completed += nevents;
	pthread_cond_signal(&cq->cond);
	pthread_mutex_unlock(&cq->mutex);
}

LATEST_SYMVER_FUNC(ibv_create_srq, 1_1, "IBVERBS_1.1",
		   struct ibv_srq *,
		   struct ibv_pd *pd,
		   struct ibv_srq_init_attr *srq_init_attr)
{
	struct ibv_srq *srq;

	srq = get_ops(pd->context)->create_srq(pd, srq_init_attr);
	if (srq) {
		srq->context          = pd->context;
		srq->srq_context      = srq_init_attr->srq_context;
		srq->pd               = pd;
		srq->events_completed = 0;
		pthread_mutex_init(&srq->mutex, NULL);
		pthread_cond_init(&srq->cond, NULL);
	}

	return srq;
}

LATEST_SYMVER_FUNC(ibv_modify_srq, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_srq *srq,
		   struct ibv_srq_attr *srq_attr,
		   int srq_attr_mask)
{
	return get_ops(srq->context)->modify_srq(srq, srq_attr, srq_attr_mask);
}

LATEST_SYMVER_FUNC(ibv_query_srq, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_srq *srq, struct ibv_srq_attr *srq_attr)
{
	return get_ops(srq->context)->query_srq(srq, srq_attr);
}

LATEST_SYMVER_FUNC(ibv_destroy_srq, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_srq *srq)
{
	return get_ops(srq->context)->destroy_srq(srq);
}

LATEST_SYMVER_FUNC(ibv_create_qp, 1_1, "IBVERBS_1.1",
		   struct ibv_qp *,
		   struct ibv_pd *pd,
		   struct ibv_qp_init_attr *qp_init_attr)
{
	struct ibv_qp *qp = calloc(1, sizeof(struct ibv_qp));

	if (PRINT_LOG) {
		printf("#### ibv_create_qp ####\n");
		fflush(stdout);
	}

	struct IBV_CREATE_QP_REQ req_body;
	req_body.pd_handle	= pd->handle;
	req_body.qp_type	= qp_init_attr->qp_type;
	req_body.sq_sig_all	= qp_init_attr->sq_sig_all;
	req_body.send_cq_handle	= qp_init_attr->send_cq->handle;
	req_body.recv_cq_handle	= qp_init_attr->recv_cq->handle;
	req_body.srq_handle	= qp_init_attr->srq ? qp_init_attr->srq->handle : 0;
	req_body.cap.max_recv_sge    = qp_init_attr->cap.max_recv_sge;
	req_body.cap.max_send_sge    = qp_init_attr->cap.max_send_sge;
	req_body.cap.max_recv_wr     = qp_init_attr->cap.max_recv_wr;
	req_body.cap.max_send_wr     = qp_init_attr->cap.max_send_wr;
	req_body.cap.max_inline_data = qp_init_attr->cap.max_inline_data;
	struct IBV_CREATE_QP_RSP rsp;
	int rsp_size;
	request_router(IBV_CREATE_QP, &req_body, &rsp, &rsp_size);

	qp->handle	= rsp.handle;
	qp->qp_num	= rsp.qp_num;
	qp_init_attr->cap.max_recv_sge	= rsp.cap.max_recv_sge;
	qp_init_attr->cap.max_send_sge	= rsp.cap.max_send_sge;
	qp_init_attr->cap.max_recv_wr	= rsp.cap.max_recv_wr;
	qp_init_attr->cap.max_send_wr	= rsp.cap.max_send_wr;
	qp_init_attr->cap.max_inline_data = rsp.cap.max_inline_data;

	qp->context			= pd->context;
	qp->qp_context		= qp_init_attr->qp_context;
	qp->pd				= pd;
	qp->send_cq			= qp_init_attr->send_cq;
	qp->recv_cq			= qp_init_attr->recv_cq;
	qp->srq				= qp_init_attr->srq;
	qp->qp_type			= qp_init_attr->qp_type;
	qp->state			= IBV_QPS_RESET;
	qp->events_completed = 0;
	pthread_mutex_init(&qp->mutex, NULL);
	pthread_cond_init(&qp->cond, NULL);

	// Patch SRQ
	// if (qp->srq) {
	// 	map_cq_to_srq[qp->recv_cq->handle] = qp->srq->handle;
	// }
	// else {
	// 	map_cq_to_srq[qp->recv_cq->handle] = MAP_SIZE + 1;
	// }

	if (PRINT_LOG) {
		printf("#### ibv_create_qp over ####\n");
		printf("init_attr.qp_type = %d\n", qp_init_attr->qp_type);
		printf("qp->handle = %d\n", qp->handle);
		printf("qp->qp_num = %d\n", qp->qp_num);
		fflush(stdout);
	}

	return qp;
}

struct ibv_qp_ex *ibv_qp_to_qp_ex(struct ibv_qp *qp)
{
	struct verbs_qp *vqp = (struct verbs_qp *)qp;

	if (vqp->comp_mask & VERBS_QP_EX)
		return &vqp->qp_ex;
	return NULL;
}

LATEST_SYMVER_FUNC(ibv_query_qp, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_qp *qp, struct ibv_qp_attr *attr,
		   int attr_mask,
		   struct ibv_qp_init_attr *init_attr)
{
	int ret;

	ret = get_ops(qp->context)->query_qp(qp, attr, attr_mask, init_attr);
	if (ret)
		return ret;

	if (attr_mask & IBV_QP_STATE)
		qp->state = attr->qp_state;

	return 0;
}

LATEST_SYMVER_FUNC(ibv_modify_qp, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_qp *qp, struct ibv_qp_attr *attr,
		   int attr_mask)
{
	struct IBV_MODIFY_QP_REQ req_body;
	memcpy(&req_body.attr, attr, sizeof(struct ibv_qp_attr));
	req_body.attr_mask = attr_mask;
	req_body.handle = qp->handle;

	struct IBV_MODIFY_QP_RSP rsp;
	int rsp_size;
	request_router(IBV_MODIFY_QP, &req_body, &rsp, &rsp_size);

	if (attr_mask & IBV_QP_STATE)
		qp->state = attr->qp_state;

	return rsp.ret;
}

LATEST_SYMVER_FUNC(ibv_destroy_qp, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_qp *qp)
{
	if (PRINT_LOG) {
		printf("#### ibv_destroy_qp ####\n");
		fflush(stdout);
	}

	struct IBV_DESTROY_QP_REQ req_body;
	req_body.qp_handle = qp->handle;
	
	struct IBV_DESTROY_QP_RSP rsp;
	int rsp_size;
	request_router(IBV_DESTROY_QP, &req_body, &rsp, &rsp_size);	

	// TODO: clean up mempool

	return rsp.ret;
}

LATEST_SYMVER_FUNC(ibv_create_ah, 1_1, "IBVERBS_1.1",
		   struct ibv_ah *,
		   struct ibv_pd *pd, struct ibv_ah_attr *attr)
{
	struct ibv_ah *ah = get_ops(pd->context)->create_ah(pd, attr);

	if (ah) {
		ah->context = pd->context;
		ah->pd      = pd;
	}

	return ah;
}

/* GID types as appear in sysfs, no change is expected as of ABI
 * compatibility.
 */
#define V1_TYPE "IB/RoCE v1"
#define V2_TYPE "RoCE v2"
int ibv_query_gid_type(struct ibv_context *context, uint8_t port_num,
		       unsigned int index, enum ibv_gid_type *type)
{
	struct verbs_device *verbs_device = verbs_get_device(context->device);
	char buff[11];

	/* Reset errno so that we can rely on its value upon any error flow in
	 * ibv_read_sysfs_file.
	 */
	errno = 0;
	if (ibv_read_ibdev_sysfs_file(buff, sizeof(buff), verbs_device->sysfs,
				      "ports/%d/gid_attrs/types/%d", port_num,
				      index) <= 0) {
		char *dir_path;
		DIR *dir;

		if (errno == EINVAL) {
			/* In IB, this file doesn't exist and the kernel sets
			 * errno to -EINVAL.
			 */
			*type = IBV_GID_TYPE_IB_ROCE_V1;
			return 0;
		}
		if (asprintf(&dir_path, "%s/%s/%d/%s/",
			     verbs_device->sysfs->ibdev_path, "ports", port_num,
			     "gid_attrs") < 0)
			return -1;
		dir = opendir(dir_path);
		free(dir_path);
		if (!dir) {
			if (errno == ENOENT)
				/* Assuming that if gid_attrs doesn't exist,
				 * we have an old kernel and all GIDs are
				 * IB/RoCE v1
				 */
				*type = IBV_GID_TYPE_IB_ROCE_V1;
			else
				return -1;
		} else {
			closedir(dir);
			errno = EFAULT;
			return -1;
		}
	} else {
		if (!strcmp(buff, V1_TYPE)) {
			*type = IBV_GID_TYPE_IB_ROCE_V1;
		} else if (!strcmp(buff, V2_TYPE)) {
			*type = IBV_GID_TYPE_ROCE_V2;
		} else {
			errno = ENOTSUP;
			return -1;
		}
	}

	return 0;
}

static int ibv_find_gid_index(struct ibv_context *context, uint8_t port_num,
			      union ibv_gid *gid, enum ibv_gid_type gid_type)
{
	enum ibv_gid_type sgid_type = 0;
	union ibv_gid sgid;
	int i = 0, ret;

	do {
		ret = ibv_query_gid(context, port_num, i, &sgid);
		if (!ret) {
			ret = ibv_query_gid_type(context, port_num, i,
						 &sgid_type);
		}
		i++;
	} while (!ret && (memcmp(&sgid, gid, sizeof(*gid)) ||
		 (gid_type != sgid_type)));

	return ret ? ret : i - 1;
}

static inline void map_ipv4_addr_to_ipv6(__be32 ipv4, struct in6_addr *ipv6)
{
	ipv6->s6_addr32[0] = 0;
	ipv6->s6_addr32[1] = 0;
	ipv6->s6_addr32[2] = htobe32(0x0000FFFF);
	ipv6->s6_addr32[3] = ipv4;
}

static inline __sum16 ipv4_calc_hdr_csum(uint16_t *data, unsigned int num_hwords)
{
	unsigned int i = 0;
	uint32_t sum = 0;

	for (i = 0; i < num_hwords; i++)
		sum += *(data++);

	sum = (sum & 0xffff) + (sum >> 16);

	return (__force __sum16)~sum;
}

static inline int get_grh_header_version(struct ibv_grh *grh)
{
	int ip6h_version = (be32toh(grh->version_tclass_flow) >> 28) & 0xf;
	struct iphdr *ip4h = (struct iphdr *)((void *)grh + 20);
	struct iphdr ip4h_checked;

	if (ip6h_version != 6) {
		if (ip4h->version == 4)
			return 4;
		errno = EPROTONOSUPPORT;
		return -1;
	}
	/* version may be 6 or 4 */
	if (ip4h->ihl != 5) /* IPv4 header length must be 5 for RoCE v2. */
		return 6;
	/*
	* Verify checksum.
	* We can't write on scattered buffers so we have to copy to temp
	* buffer.
	*/
	memcpy(&ip4h_checked, ip4h, sizeof(ip4h_checked));
	/* Need to set the checksum field (check) to 0 before re-calculating
	 * the checksum.
	 */
	ip4h_checked.check = 0;
	ip4h_checked.check = ipv4_calc_hdr_csum((uint16_t *)&ip4h_checked, 10);
	/* if IPv4 header checksum is OK, believe it */
	if (ip4h->check == ip4h_checked.check)
		return 4;
	return 6;
}

static inline void set_ah_attr_generic_fields(struct ibv_ah_attr *ah_attr,
					      struct ibv_wc *wc,
					      struct ibv_grh *grh,
					      uint8_t port_num)
{
	uint32_t flow_class;

	flow_class = be32toh(grh->version_tclass_flow);
	ah_attr->grh.flow_label = flow_class & 0xFFFFF;
	ah_attr->dlid = wc->slid;
	ah_attr->sl = wc->sl;
	ah_attr->src_path_bits = wc->dlid_path_bits;
	ah_attr->port_num = port_num;
}

static inline int set_ah_attr_by_ipv4(struct ibv_context *context,
				      struct ibv_ah_attr *ah_attr,
				      struct iphdr *ip4h, uint8_t port_num)
{
	union ibv_gid sgid;
	int ret;

	/* No point searching multicast GIDs in GID table */
	if (IN_CLASSD(be32toh(ip4h->daddr))) {
		errno = EINVAL;
		return -1;
	}

	map_ipv4_addr_to_ipv6(ip4h->daddr, (struct in6_addr *)&sgid);
	ret = ibv_find_gid_index(context, port_num, &sgid,
				 IBV_GID_TYPE_ROCE_V2);
	if (ret < 0)
		return ret;

	map_ipv4_addr_to_ipv6(ip4h->saddr,
			      (struct in6_addr *)&ah_attr->grh.dgid);
	ah_attr->grh.sgid_index = (uint8_t) ret;
	ah_attr->grh.hop_limit = ip4h->ttl;
	ah_attr->grh.traffic_class = ip4h->tos;

	return 0;
}

#define IB_NEXT_HDR    0x1b
static inline int set_ah_attr_by_ipv6(struct ibv_context *context,
				  struct ibv_ah_attr *ah_attr,
				  struct ibv_grh *grh, uint8_t port_num)
{
	uint32_t flow_class;
	uint32_t sgid_type;
	int ret;

	/* No point searching multicast GIDs in GID table */
	if (grh->dgid.raw[0] == 0xFF) {
		errno = EINVAL;
		return -1;
	}

	ah_attr->grh.dgid = grh->sgid;
	if (grh->next_hdr == IPPROTO_UDP) {
		sgid_type = IBV_GID_TYPE_ROCE_V2;
	} else if (grh->next_hdr == IB_NEXT_HDR) {
		sgid_type = IBV_GID_TYPE_IB_ROCE_V1;
	} else {
		errno = EPROTONOSUPPORT;
		return -1;
	}

	ret = ibv_find_gid_index(context, port_num, &grh->dgid,
				 sgid_type);
	if (ret < 0)
		return ret;

	ah_attr->grh.sgid_index = (uint8_t) ret;
	flow_class = be32toh(grh->version_tclass_flow);
	ah_attr->grh.hop_limit = grh->hop_limit;
	ah_attr->grh.traffic_class = (flow_class >> 20) & 0xFF;

	return 0;
}

int ibv_init_ah_from_wc(struct ibv_context *context, uint8_t port_num,
			struct ibv_wc *wc, struct ibv_grh *grh,
			struct ibv_ah_attr *ah_attr)
{
	int version;
	int ret = 0;

	memset(ah_attr, 0, sizeof *ah_attr);
	set_ah_attr_generic_fields(ah_attr, wc, grh, port_num);

	if (wc->wc_flags & IBV_WC_GRH) {
		ah_attr->is_global = 1;
		version = get_grh_header_version(grh);

		if (version == 4)
			ret = set_ah_attr_by_ipv4(context, ah_attr,
						  (struct iphdr *)((void *)grh + 20),
						  port_num);
		else if (version == 6)
			ret = set_ah_attr_by_ipv6(context, ah_attr, grh,
						  port_num);
		else
			ret = -1;
	}

	return ret;
}

struct ibv_ah *ibv_create_ah_from_wc(struct ibv_pd *pd, struct ibv_wc *wc,
				     struct ibv_grh *grh, uint8_t port_num)
{
	struct ibv_ah_attr ah_attr;
	int ret;

	ret = ibv_init_ah_from_wc(pd->context, port_num, wc, grh, &ah_attr);
	if (ret)
		return NULL;

	return ibv_create_ah(pd, &ah_attr);
}

LATEST_SYMVER_FUNC(ibv_destroy_ah, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_ah *ah)
{
	return get_ops(ah->context)->destroy_ah(ah);
}

LATEST_SYMVER_FUNC(ibv_attach_mcast, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_qp *qp, const union ibv_gid *gid, uint16_t lid)
{
	return get_ops(qp->context)->attach_mcast(qp, gid, lid);
}

LATEST_SYMVER_FUNC(ibv_detach_mcast, 1_1, "IBVERBS_1.1",
		   int,
		   struct ibv_qp *qp, const union ibv_gid *gid, uint16_t lid)
{
	return get_ops(qp->context)->detach_mcast(qp, gid, lid);
}

static inline int ipv6_addr_v4mapped(const struct in6_addr *a)
{
	return IN6_IS_ADDR_V4MAPPED(&a->s6_addr32) ||
		/* IPv4 encoded multicast addresses */
		(a->s6_addr32[0]  == htobe32(0xff0e0000) &&
		((a->s6_addr32[1] |
		 (a->s6_addr32[2] ^ htobe32(0x0000ffff))) == 0UL));
}

struct peer_address {
	void *address;
	uint32_t size;
};

static inline int create_peer_from_gid(int family, void *raw_gid,
				       struct peer_address *peer_address)
{
	switch (family) {
	case AF_INET:
		peer_address->address = raw_gid + 12;
		peer_address->size = 4;
		break;
	case AF_INET6:
		peer_address->address = raw_gid;
		peer_address->size = 16;
		break;
	default:
		return -1;
	}

	return 0;
}

#define NEIGH_GET_DEFAULT_TIMEOUT_MS 3000
int ibv_resolve_eth_l2_from_gid(struct ibv_context *context,
				struct ibv_ah_attr *attr,
				uint8_t eth_mac[ETHERNET_LL_SIZE],
				uint16_t *vid)
{
	int dst_family;
	int src_family;
	int oif;
	struct get_neigh_handler neigh_handler;
	union ibv_gid sgid;
	int ether_len;
	struct peer_address src;
	struct peer_address dst;
	uint16_t ret_vid;
	int ret = -EINVAL;
	int err;

	err = ibv_query_gid(context, attr->port_num,
			    attr->grh.sgid_index, &sgid);

	if (err)
		return err;

	err = neigh_init_resources(&neigh_handler,
				   NEIGH_GET_DEFAULT_TIMEOUT_MS);

	if (err)
		return err;

	dst_family = ipv6_addr_v4mapped((struct in6_addr *)attr->grh.dgid.raw) ?
			AF_INET : AF_INET6;
	src_family = ipv6_addr_v4mapped((struct in6_addr *)sgid.raw) ?
			AF_INET : AF_INET6;

	if (create_peer_from_gid(dst_family, attr->grh.dgid.raw, &dst))
		goto free_resources;

	if (create_peer_from_gid(src_family, &sgid.raw, &src))
		goto free_resources;

	if (neigh_set_dst(&neigh_handler, dst_family, dst.address,
			  dst.size))
		goto free_resources;

	if (neigh_set_src(&neigh_handler, src_family, src.address,
			  src.size))
		goto free_resources;

	oif = neigh_get_oif_from_src(&neigh_handler);

	if (oif > 0)
		neigh_set_oif(&neigh_handler, oif);
	else
		goto free_resources;

	ret = -EHOSTUNREACH;

	/* blocking call */
	if (process_get_neigh(&neigh_handler))
		goto free_resources;

	if (vid) {
		ret_vid = neigh_get_vlan_id_from_dev(&neigh_handler);

		if (ret_vid <= 0xfff)
			neigh_set_vlan_id(&neigh_handler, ret_vid);
	}

	/* We are using only Ethernet here */
	ether_len = neigh_get_ll(&neigh_handler,
				 eth_mac,
				 sizeof(uint8_t) * ETHERNET_LL_SIZE);

	if (ether_len <= 0)
		goto free_resources;

	if (vid)
		*vid = ret_vid;

	ret = 0;

free_resources:
	neigh_free_resources(&neigh_handler);

	return ret;
}

LATEST_SYMVER_FUNC(ibv_post_send, 1_1, "IBVERBS_1.1",
			int,
		   	struct ibv_qp *qp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr)
{
	struct ibv_send_wr	*i;
	struct ibv_send_wr	*n, *tmp = NULL;
	struct ibv_sge		*s;
	uint32_t			*ah;
	uint32_t			wr_count = 0, sge_count = 0, ah_count = 0;

	struct FfrRequestHeader		*header = calloc(1, sizeof(struct FfrRequestHeader));
	struct ib_uverbs_post_send	*cmd = calloc(1, 4*1024);
	struct IBV_POST_SEND_RSP	*rsp = calloc(1, sizeof(struct IBV_POST_SEND_RSP));
	

    if (PRINT_LOG) {
    	printf("#### ibv_post_send ####\n");
		fflush(stdout);
    }

	for (i = wr; i; i = i->next) {
		wr_count++;
		sge_count += i->num_sge;
	}

	if (qp->qp_type == IBV_QPT_UD) {
		ah_count = wr_count;
	}

	header->client_id = ffr_client_id;
	header->func = IBV_POST_SEND;
	header->body_size = sizeof *cmd + wr_count * sizeof *n + sge_count * sizeof *s + ah_count * sizeof(uint32_t);

	//IBV_INIT_CMD_RESP(cmd, req_body.wr_size, POST_SEND, &resp, sizeof resp);
	cmd->qp_handle = qp->handle;
	cmd->wr_count  = wr_count;
	cmd->sge_count = sge_count;

	n = (struct ibv_send_wr *) ((void *) cmd + sizeof *cmd);
	s = (struct ibv_sge *) (n + wr_count);
	ah = (uint32_t*) (s + sge_count);

	tmp = n;
	for (i = wr; i; i = i->next) {
		memcpy(tmp, i, sizeof(struct ibv_send_wr));
		if (qp->qp_type == IBV_QPT_UD) {
			*ah = i->wr.ud.ah->handle;
			ah = ah + 1;
		}

		if (tmp->num_sge) {
			/* freeflow copy data */
			memcpy(s, i->sg_list, tmp->num_sge * sizeof *s);
			// 对齐地址
			for (int j = 0; j < tmp->num_sge; j++) {
				struct mr_shm *p = (struct mr_shm*)mempool_get(map_lkey_to_mrshm, s[j].lkey);
				char *mr = p->mr;
				char *shm = p->shm_ptr;

				char *addr = (char*)(s[j].addr);
				if (shm != mr) {
					memcpy(shm + (addr - mr), addr, s[j].length);	
				}
				s[j].addr = addr - mr;

				if (PRINT_LOG) {
					printf("!!!!!!!!!! length=%u,addr=%lu,lkey=%u,mr_ptr=%lu,shm_ptr=%lu,original_addr=%lu\n", s[j].length, s[j].addr, s[j].lkey, (uint64_t)mr, (uint64_t)shm, (uint64_t)addr);
					fflush(stdout);
					printf("wr=%lu, sge=%lu, sizeof send_wr=%lu\n", (uint64_t)n, (uint64_t)s, sizeof(*n));
					fflush(stdout);
				}
			}
			s += tmp->num_sge;
		}
		tmp++;
	}

	struct IBV_POST_SEND_REQ req_body;
	req_body.wr_size = header->body_size;
	req_body.wr = (char*)cmd;
	int rsp_size;
	request_router(IBV_POST_SEND, &req_body, rsp, &rsp_size);
 
	wr_count = rsp->bad_wr;
	if (wr_count) {
		i = wr;
		while (--wr_count)
			i = i->next;
		*bad_wr = i;
	} else if (rsp->ret_errno) {
		*bad_wr = wr;
	}
	
	// wmb();

	return rsp->ret_errno;
}

LATEST_SYMVER_FUNC(ibv_post_recv, 1_1, "IBVERBS_1.1",
			int,
			struct ibv_qp *qp, struct ibv_recv_wr *wr,
			struct ibv_recv_wr **bad_wr)
{
	struct ibv_recv_wr		*i;
	struct ibv_recv_wr		*n = NULL, *tmp = NULL;
	struct ibv_sge			*s;
	uint32_t				wr_count = 0, sge_count = 0;
	int						wr_index;
	struct wr				*wr_p;
	struct sge_record		*sge_p;
	struct wr_queue			*wr_queue;

	struct FfrRequestHeader		*header = calloc(1, sizeof(struct FfrRequestHeader));
	struct ib_uverbs_post_send	*cmd = calloc(1, 4*1024);
	struct IBV_POST_SEND_RSP	*rsp = calloc(1, sizeof(struct IBV_POST_SEND_RSP));

	if (PRINT_LOG) {
		printf("#### ibv_post_receive ####\n");
		fflush(stdout);
	}

	for (i = wr; i; i = i->next) {
		wr_count++;
		sge_count += i->num_sge;
	}

	header->client_id = ffr_client_id;
	header->func = IBV_POST_RECV;
	header->body_size = sizeof *cmd + wr_count * sizeof *n + sge_count * sizeof *s;

	// IBV_INIT_CMD_RESP(cmd, req_body.wr_size, POST_RECV, &resp, sizeof resp);
	cmd->qp_handle = qp->handle;
	cmd->wr_count  = wr_count;
	cmd->sge_count = sge_count;

	n = (struct ibv_recv_wr *) ((void *) cmd + sizeof *cmd);
	s = (struct ibv_sge *) (n + wr_count);

	wr_queue = map_cq_to_wr_queue[qp->recv_cq->handle];

	tmp = n;
	for (i = wr; i; i = i->next) {
		memcpy(tmp, i, sizeof(struct ibv_recv_wr));

		// We are already in critical section
		//pthread_spin_lock(&(wr_queue->head_lock));
		wr_index = wr_queue->head;
		wr_queue->head++;
		if (wr_queue->head >= WR_QUEUE_SIZE) {
			wr_queue->head = 0;
		}
		//pthread_spin_unlock(&(wr_queue->head_lock));

		wr_p = wr_queue->queue + wr_index;
		wr_p->sge_num = tmp->num_sge;
		printf("ibv_post_recv: wr_p->sge_num = %d\n", tmp->num_sge);

		if (tmp->num_sge) {
			wr_p->sge_queue = malloc(sizeof(struct sge_record) * tmp->num_sge);
			sge_p = wr_p->sge_queue;

			/* freeflow keeps track of offsets */
			tmp->sg_list = s;
			memcpy(s, i->sg_list, tmp->num_sge * sizeof *s);

			for (int j = 0; j < tmp->num_sge; j++) {
				struct mr_shm *p = (struct mr_shm*)mempool_get(map_lkey_to_mrshm, s[j].lkey);
				char *mr = p->mr;
				
				char *addr = (char*)(s[j].addr);
				s[j].addr = addr - mr;

				sge_p->length = s[j].length;
				sge_p->mr_addr = addr;
				sge_p->shm_addr = p->shm_ptr + s[j].addr;
				sge_p++;
			}
			s += tmp->num_sge;
		}
		tmp++;
	}

	struct IBV_POST_RECV_REQ req_body;
	req_body.wr_size = header->body_size;
	req_body.wr = (char*)cmd;
	int rsp_size;
	request_router(IBV_POST_RECV, &req_body, rsp, &rsp_size);

	wr_count = rsp->bad_wr;
	if (wr_count) {
		i = wr;
		while (--wr_count)
			i = i->next;
		*bad_wr = i;
	} else if (rsp->ret_errno) {
		*bad_wr = wr;
	}



	return rsp->ret_errno;
}

LATEST_SYMVER_FUNC(ibv_poll_cq, 1_1, "IBVERBS_1.1",
			int,
			struct ibv_cq *cq, int num_entries, struct ibv_wc *wc)
{
	int					i, j, wr_index;
	struct sge_record	*s;
	struct wr_queue		*wr_queue;
	struct wr			*wr_p;

	struct FfrRequestHeader		*req_header = calloc(1, sizeof(struct FfrRequestHeader));
	struct IBV_POLL_CQ_REQ		*req = calloc(1, sizeof(struct IBV_POLL_CQ_REQ));
	struct FfrResponseHeader	*rsp_header = calloc(1, sizeof(struct FfrResponseHeader));
	struct ibv_wc				*wc_list = calloc(num_entries, sizeof(struct ibv_wc));

	// if (PRINT_LOG) {
	// 	printf("#### ibv_poll_cq ####\n");
	// 	fflush(stdout);
	// }

	req_header->client_id = ffr_client_id;
	req_header->func = IBV_POLL_CQ;
	req_header->body_size = sizeof(struct IBV_POLL_CQ_REQ);

	req->cq_handle = cq->handle;
	req->ne = num_entries;

	// patch SRQ
	// if (map_cq_to_srq[cq->handle] <= MAP_SIZE) {
	// 	pthread_spin_lock(&(map_srq_to_wr_queue[map_cq_to_srq[cq->handle]]->tail_lock));
	// }

	request_router(IBV_POLL_CQ, req, wc_list, &rsp_header->rsp_size);

	if (rsp_header->rsp_size % sizeof(struct ibv_wc) != 0) {
		printf("[Error] The rsp size is %d while the unit ibn_wc size is %d.", rsp_header->rsp_size, (int)sizeof(struct ibv_wc));
		fflush(stdout);
		return -1;
	}

	int count = rsp_header->rsp_size / sizeof(struct ibv_wc);

	// if (PRINT_LOG) {
	// 	printf("count = %d, rsp_size = %d, sizeof struct ibv_wc=%d\n", count, rsp_header->rsp_size, (int)sizeof(struct ibv_wc));
	// 	fflush(stdout);
	// }

	memcpy((char*)wc, (char const *)wc_list, rsp_header->rsp_size);

	/*
	for (i = 0; i < count; i++) {
		wc[i].wr_id = wc_list[i].wr_id;
		wc[i].status = wc_list[i].status;
		wc[i].opcode = wc_list[i].opcode;
		wc[i].vendor_err = wc_list[i].vendor_err;
		wc[i].byte_len = wc_list[i].byte_len;
		wc[i].imm_data = wc_list[i].imm_data;
		wc[i].qp_num = wc_list[i].qp_num;
		wc[i].src_qp = wc_list[i].src_qp;
		wc[i].wc_flags = wc_list[i].wc_flags;
		wc[i].pkey_index = wc_list[i].pkey_index;
		wc[i].slid = wc_list[i].slid;
		wc[i].sl = wc_list[i].sl;
		wc[i].dlid_path_bits = wc_list[i].dlid_path_bits;
	}
	*/

	if (PRINT_LOG)
	{
		for (i = 0; i < count; i++) {
			printf("=================================\n");
			printf("wc_list[%d].wr_id=%lu\n", i, (unsigned long)wc_list[i].wr_id);
			printf("wc_list[%d].status=%d\n", i, wc_list[i].status);
			printf("wc_list[%d].opcode=%d\n", i, wc_list[i].opcode);
			printf("wc_list[%d].vendor_err=%d\n", i, wc_list[i].vendor_err);
			printf("wc_list[%d].byte_len=%d\n", i, wc_list[i].byte_len);
			printf("wc_list[%d].imm_data=%u\n", i, wc_list[i].imm_data);
			printf("wc_list[%d].qp_num=%d\n", i, wc_list[i].qp_num);
			printf("wc_list[%d].src_qp=%d\n", i, wc_list[i].src_qp);
			printf("wc_list[%d].wc_flags=%d\n", i, wc_list[i].wc_flags);
			printf("wc_list[%d].pkey_index=%d\n", i, wc_list[i].pkey_index);
			printf("wc_list[%d].slid=%d\n", i, wc_list[i].slid);
			printf("wc_list[%d].sl=%d\n", i, wc_list[i].sl);
			printf("wc_list[%d].ldid_path_bits=%d\n", i, wc_list[i].dlid_path_bits);
			fflush(stdout);
		}
	}

	/* Copying shared memory here if it is a receve queue. */
	for (i = 0; i < count; i++) {
		if ((wc_list[i].opcode) & IBV_WC_RECV) {			
			// if (map_cq_to_srq[cq->handle] > MAP_SIZE) {
				wr_queue = map_cq_to_wr_queue[cq->handle];
			// }
			// else {
			// 	wr_queue = map_srq_to_wr_queue[map_cq_to_srq[cq->handle]];
			// }

			// we are already in critical section
			// pthread_spin_lock(&(wr_queue->tail_lock));
			wr_index = wr_queue->tail;
			wr_queue->tail++;
			if (wr_queue->tail >= WR_QUEUE_SIZE) {
				wr_queue->tail = 0;
			}
			// pthread_spin_unlock(&(wr_queue->tail_lock));

			wr_p = wr_queue->queue + wr_index;

			/* freeflow retrieve addresses */
			if (wr_p->sge_num) {
				s = (struct sge_record*)(wr_p->sge_queue);
				for (j = 0; j < wr_p->sge_num; j++) {
					if (s->mr_addr != s->shm_addr) {
						memcpy(s->mr_addr, s->shm_addr, s->length);	
					}
					s++;
				}
				free(wr_p->sge_queue);
			}
		}
	}

	// wmb();
	// patch SRQ
	// if (map_cq_to_srq[cq->handle] <= MAP_SIZE) {
	// 	pthread_spin_unlock(&(map_srq_to_wr_queue[map_cq_to_srq[cq->handle]]->tail_lock));
	// }

	return count;
}