/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <zephyr/kernel.h>

//#include <zephyr/sys/byteorder.h>
//#include <zephyr/sys/slist.h>
//#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"

#include "util/util.h"
//#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

//#include "ll.h"
//#include "ll_settings.h"

#include "lll.h"
#include "ll_feat.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
//#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

//#include "isoal.h"
//#include "ull_iso_types.h"
//#include "ull_conn_iso_types.h"
//#include "ull_conn_iso_internal.h"

#include "ull_conn_types.h"
#include "ull_internal.h"
//#include "ull_llcp.h"
//#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"
//#include "ull_conn_internal.h"
//
//#include <soc.h>
#include "hal/debug.h"

/* LLCP Local Procedure Subrating FSM states */
enum {
	LP_SR_STATE_IDLE = LLCP_STATE_IDLE,
        LP_SR_STATE_WAIT_TX_ACK_SUB_IND,
	LP_SR_STATE_WAIT_TX_SUB_IND,
        LP_SR_STATE_NOTIFY,
};

/* LLCP Local Procedure Subrating FSM events */
enum {
	/* Procedure run */
	LP_SR_EVT_RUN,

	/* Ack received */
	LP_SR_EVT_ACK,

	/* Notify received */
	LP_SR_EVT_NTF,

	LP_SR_EVT_SUBRATE_IND,

	LP_SR_EVT_REJECT,
};

/* LLCP Remote Procedure Subrating FSM states */
enum {
	RP_SR_STATE_IDLE = LLCP_STATE_IDLE,
};

/* LLCP Remote Procedure Subrating FSM events */
enum {
	/* Procedure run */
	RP_SR_EVT_RUN,
};

static void lp_sr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Get pre-allocated tx node */
	tx = ctx->node_ref.tx;
	ctx->node_ref.tx = NULL;

	if (!tx) {
		/* Allocate tx node if non pre-alloc'ed */
		tx = llcp_tx_alloc(conn, ctx);
		LL_ASSERT(tx);
	}
	ctx->node_ref.tx_ack = tx;

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ:
		llcp_pdu_encode_subrate_req_or_ind(ctx, pdu, true);
		break;
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		llcp_pdu_encode_subrate_req_or_ind(ctx, pdu, false);
		break;
	default:
		/* Unknown opcode */
		LL_ASSERT(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);
}

static void sr_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct node_rx_sr *pdu;

	/* Allocate ntf node */
	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	if (!ntf) {
		/* Allocate ntf node */
		ntf = llcp_ntf_alloc();
		LL_ASSERT(ntf);
	}

	ntf->hdr.type = NODE_RX_TYPE_SUBRATE_CHANGE;
	ntf->hdr.handle = conn->lll.handle;

	pdu = (struct node_rx_sr *)ntf->pdu;
	ll_rx_put_sched(ntf->hdr.link, ntf);
}

static void lp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_SR_STATE_IDLE;
}

static void lp_sr_send_subrating_req_or_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				            void *param)
{
	uint8_t opcode;
	if (ctx->proc == PROC_CONN_SUBRATE_REQUEST) {
		opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ;
		ctx->state = LP_SR_STATE_WAIT_TX_SUB_IND;
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
	}
	else {
		opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
		ctx->state = LP_SR_STATE_WAIT_TX_ACK_SUB_IND;
	}
	lp_sr_tx(conn, ctx, opcode);
}

static void lp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		switch (ctx->proc) {
		case PROC_CONN_SUBRATE_REQUEST:
		case PROC_CONN_SUBRATE_UPDATE:
			lp_sr_send_subrating_req_or_ind(conn, ctx, evt, param);
			break;
		default:
			/* Unknown procedure */
			LL_ASSERT(0);
			break;
		}
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_wait_tx_ack_sub_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_ACK:
		ctx->state = LP_SR_STATE_NOTIFY;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_wait_tx_sub_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_SUBRATE_IND:
	case LP_SR_EVT_REJECT:
		ctx->state = LP_SR_STATE_NOTIFY;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_notify(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_NTF:
		sr_ntf(conn, ctx);
		lp_sr_complete(conn, ctx);
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_SR_STATE_IDLE:
		lp_sr_st_idle(conn, ctx, evt, param);
		break;

	case LP_SR_STATE_WAIT_TX_ACK_SUB_IND:
		lp_sr_st_wait_tx_ack_sub_ind(conn, ctx, evt, param);
		break;

	case LP_SR_STATE_WAIT_TX_SUB_IND:
		lp_sr_st_wait_tx_sub_ind(conn, ctx, evt, param);
		break;

	case LP_SR_STATE_NOTIFY:
		lp_sr_st_notify(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT(0);
	}
}

void llcp_lp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_RUN, param);
}

void llcp_lp_sr_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_ACK, param);
}

void llcp_lp_sr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;
	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_SUBRATE_IND, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_REJECT, pdu);
		break;
	default:
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		llcp_lr_complete(conn);
		ctx->state = LP_SR_STATE_IDLE;
		break;
	}
}

void llcp_lp_sr_tx_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_NTF, NULL);
}
