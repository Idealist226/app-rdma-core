/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#ifndef FREEFLOW_H
#define FREEFLOW_H

#include "freeflow-types.h"
#include <infiniband/verbs.h>
#include <pthread.h>

extern void mem_flush(const void *p, int allocation_size);

extern void* mempool_create(void);
extern void* mempool_insert(void* mempool, uint32_t k);
extern void mempool_del(void* mempool, uint32_t k);
extern void* mempool_get(void* mempool, uint32_t k);
extern void mempool_destroy(void* mempool);

extern int ffr_client_id;


/**
 * FreeFlow Function
 * Connect to router via unix socket
 */
struct sock_with_lock
{
	int sock;
	pthread_mutex_t mutex;
};

void init_sock(void);
struct sock_with_lock* get_unix_sock(RDMA_FUNCTION_CALL req);
void connect_router(struct sock_with_lock *unix_sock);
void request_router(RDMA_FUNCTION_CALL req, void* req_body, void *rsp, int *rsp_size);

extern struct sock_with_lock control_sock[PARALLEL_SIZE];
extern struct sock_with_lock write_sock[PARALLEL_SIZE];
extern struct sock_with_lock read_sock[PARALLEL_SIZE];
extern struct sock_with_lock poll_sock[PARALLEL_SIZE];
extern struct sock_with_lock event_sock[PARALLEL_SIZE];

extern struct sock_with_lock comp_channel_sock_map[MAP_SIZE];
extern int comp_channel_map[MAP_SIZE];

extern int sock_initialized;

/******************************************
*     Map lkey to MR and SHM pointer      *
******************************************/
struct mr_shm {
	char *mr;
	char *shm_ptr;
};

extern void* map_lkey_to_mrshm;

/******************************************
*    Map a CQ or a SRQ to a queue of WR   *
******************************************/
struct sge_record {
	uint32_t length;
	char *mr_addr;
	char *shm_addr;
};

struct wr {
	uint32_t sge_num;
	struct sge_record* sge_queue;
};

struct wr_queue {
	struct wr* queue;
	uint32_t head, tail;
	pthread_spinlock_t head_lock, tail_lock;
};

extern struct wr_queue* map_cq_to_wr_queue[MAP_SIZE];
extern uint32_t map_cq_to_srq[MAP_SIZE];
extern struct wr_queue* map_srq_to_wr_queue[MAP_SIZE];


int recv_fd(struct sock_with_lock *unix_sock);

#endif /* FREEFLOW_H */

