/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "ll_feat.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_iso_internal.h"

#include "ull_conn_types.h"
#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"
#include "ull_conn_internal.h"

#include <soc.h>
#include "hal/debug.h"

/* LLCP Local Procedure PHY Update FSM states */
enum {
	LP_SR_STATE_IDLE = LLCP_STATE_IDLE,
#if 0
	LP_PU_STATE_WAIT_TX_PHY_REQ,
	LP_PU_STATE_WAIT_TX_ACK_PHY_REQ,
	LP_PU_STATE_WAIT_RX_PHY_RSP,
	LP_PU_STATE_WAIT_TX_PHY_UPDATE_IND,
	LP_PU_STATE_WAIT_TX_ACK_PHY_UPDATE_IND,
	LP_PU_STATE_WAIT_RX_PHY_UPDATE_IND,
	LP_PU_STATE_WAIT_NTF_AVAIL,
	LP_PU_STATE_WAIT_INSTANT,
	LP_PU_STATE_WAIT_INSTANT_ON_AIR,
#endif
};

/* LLCP Local Procedure PHY Update FSM events */
enum {
	/* Procedure run */
	LP_SR_EVT_RUN,
#if 0

	/* Response received */
	LP_PU_EVT_PHY_RSP,

	/* Indication received */
	LP_PU_EVT_PHY_UPDATE_IND,

	/* Ack received */
	LP_PU_EVT_ACK,

	/* Ready to notify host */
	LP_PU_EVT_NTF,

	/* Reject response received */
	LP_PU_EVT_REJECT,

	/* Unknown response received */
	LP_PU_EVT_UNKNOWN,
#endif
};

/* LLCP Remote Procedure PHY Update FSM states */
enum {
	RP_PU_STATE_IDLE = LLCP_STATE_IDLE,
	RP_PU_STATE_WAIT_RX_PHY_REQ,
	RP_PU_STATE_WAIT_TX_PHY_RSP,
	RP_PU_STATE_WAIT_TX_ACK_PHY_RSP,
	RP_PU_STATE_WAIT_TX_PHY_UPDATE_IND,
	RP_PU_STATE_WAIT_TX_ACK_PHY_UPDATE_IND,
	RP_PU_STATE_WAIT_RX_PHY_UPDATE_IND,
	RP_PU_STATE_WAIT_NTF_AVAIL,
	RP_PU_STATE_WAIT_INSTANT,
	RP_PU_STATE_WAIT_INSTANT_ON_AIR,
};

/* LLCP Remote Procedure PHY Update FSM events */
enum {
	/* Procedure run */
	RP_PU_EVT_RUN,

	/* Request received */
	RP_PU_EVT_PHY_REQ,

	/* Ack received */
	RP_PU_EVT_ACK,

	/* Indication received */
	RP_PU_EVT_PHY_UPDATE_IND,

	/* Ready to notify host */
	RP_PU_EVT_NTF,
};

/*
 * LLCP Local Procedure PHY Update FSM
 */

#if 0
static void sr_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct node_rx_pu *pdu;

	/* Piggy-back on stored RX node */
	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	LL_ASSERT(ntf);

	if (ctx->data.pu.ntf_pu) {
		LL_ASSERT(ntf->hdr.type == NODE_RX_TYPE_RETAIN);
		ntf->hdr.type = NODE_RX_TYPE_PHY_UPDATE;
		ntf->hdr.handle = conn->lll.handle;
		pdu = (struct node_rx_pu *)ntf->pdu;

		pdu->status = ctx->data.pu.error;
		pdu->rx = conn->lll.phy_rx;
		pdu->tx = conn->lll.phy_tx;
	} else {
		ntf->hdr.type = NODE_RX_TYPE_RELEASE;
	}

	/* Enqueue notification towards LL */
	ll_rx_put_sched(ntf->hdr.link, ntf);

	ctx->data.pu.ntf_pu = 0;
}

static void lp_sr_complete_finalize(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	llcp_rr_set_paused_cmd(conn, PROC_NONE);
	ctx->state = LP_SR_STATE_IDLE;
}

static void lp_sr_tx_ntf(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	sr_ntf(conn, ctx);
#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	pu_dle_ntf(conn, ctx);
#endif
	lp_sr_complete_finalize(conn, ctx);
}

static void lp_sr_send_subrating_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	if (llcp_lr_ispaused(conn) || llcp_rr_get_collision(conn) ||
	    !llcp_tx_alloc_peek(conn, ctx) ||
	    (llcp_rr_get_paused_cmd(conn) == PROC_PHY_UPDATE)) {
		ctx->state = LP_PU_STATE_WAIT_TX_PHY_REQ;
	} else {
		llcp_rr_set_incompat(conn, INCOMPAT_RESOLVABLE);
		llcp_rr_set_paused_cmd(conn, PROC_CTE_REQ);
		ctx->tx_opcode = PDU_DATA_LLCTRL_TYPE_PHY_REQ;

		/* Allocate TX node */
		ctx->node_ref.tx = llcp_tx_alloc(conn, ctx);
		lp_pu_tx(conn, ctx, evt, param);
	}
}
#endif

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

#if defined(CONFIG_BT_CENTRAL)
static void lp_sr_send_subrating_req_or_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				            void *param)
{
	uint8_t opcode = ctx->proc == PROC_CONN_SUBRATING_REQ ?
				PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ :
				PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
	lp_sr_tx(conn, ctx, opcode);
#if 0
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_PU_STATE_WAIT_TX_PHY_UPDATE_IND;
	} else {
		ctx->tx_opcode = PDU_DATA_LLCTRL_TYPE_PHY_UPD_IND;

		/* Allocate TX node */
		ctx->node_ref.tx = llcp_tx_alloc(conn, ctx);
		lp_pu_tx(conn, ctx, evt, param);
	}
#endif
}
#endif /* CONFIG_BT_CENTRAL */

static void lp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		switch (ctx->proc) {
		case PROC_CONN_SUBRATING_REQ:
		case PROC_CONN_SUBRATING_IND:
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

static void lp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_SR_STATE_IDLE:
		lp_sr_st_idle(conn, ctx, evt, param);
		break;
#if 0
	case LP_PU_STATE_WAIT_TX_PHY_REQ:
		lp_pu_st_wait_tx_phy_req(conn, ctx, evt, param);
		break;
	case LP_PU_STATE_WAIT_TX_ACK_PHY_REQ:
		lp_pu_st_wait_tx_ack_phy_req(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_CENTRAL)
	case LP_PU_STATE_WAIT_RX_PHY_RSP:
		lp_pu_st_wait_rx_phy_rsp(conn, ctx, evt, param);
		break;
	case LP_PU_STATE_WAIT_TX_PHY_UPDATE_IND:
		lp_pu_st_wait_tx_phy_update_ind(conn, ctx, evt, param);
		break;
	case LP_PU_STATE_WAIT_TX_ACK_PHY_UPDATE_IND:
		lp_pu_st_wait_tx_ack_phy_update_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
	case LP_PU_STATE_WAIT_RX_PHY_UPDATE_IND:
		lp_pu_st_wait_rx_phy_update_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
	case LP_PU_STATE_WAIT_INSTANT:
		lp_pu_st_wait_instant(conn, ctx, evt, param);
		break;
	case LP_PU_STATE_WAIT_INSTANT_ON_AIR:
		lp_pu_st_wait_instant_on_air(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	case LP_PU_STATE_WAIT_NTF_AVAIL:
		lp_pu_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */
#endif
	default:
		/* Unknown state */
		LL_ASSERT(0);
	}
}

void llcp_lp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_RUN, param);
}
