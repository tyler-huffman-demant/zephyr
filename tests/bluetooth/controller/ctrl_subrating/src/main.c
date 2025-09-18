/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/ztest.h>

#define ULL_LLCP_UNITTEST

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
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
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

/* Default connection values */
#define SUBRATE_MIN 5U
#define SUBRATE_MAX 10U
#define LATENCY 1U
#define CONTINUATION_NUMBER 2U
#define SUPERVISION_TIMEOUT 3U

struct pdu_data_llctrl_subrate_req subrate_req = { .subrate_factor_min = SUBRATE_MIN,
						   .subrate_factor_max = SUBRATE_MAX,
						   .max_latency = LATENCY,
						   .continuation_number = CONTINUATION_NUMBER,
						   .timeout = SUPERVISION_TIMEOUT
};

struct pdu_data_llctrl_subrate_req subrate_ind = { .subrate_factor_min = SUBRATE_MIN,
						   .subrate_factor_max = SUBRATE_MAX,
						   .max_latency = LATENCY,
						   .continuation_number = CONTINUATION_NUMBER,
						   .timeout = SUPERVISION_TIMEOUT
};

static struct ll_conn conn;


static void conn_update_setup(void *data)
{
	test_setup(&conn);

	/* Initialize lll conn parameters (different from new) */
	struct lll_conn *lll = &conn.lll;

	//TODO: Do we need this?
	lll->interval = 0;
	lll->latency = 0;
	conn.supervision_timeout = 1U;
	lll->event_counter = 0;
}

/*
 * Central-initiated Connection Parameters Request procedure.
 * Central requests change in LE connection parameters, peripheral’s Host accepts.
 *
 * +------------+                +------------+            +-----+
 * | UT Central |                | LL Central |            | LT  |
 * +------------+                +------------+            +-----+
 *    |                           |                           |
 *    | LE Subrate Request        |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    | Command status            |                           |
 *    |<--------------------------|                           |
 *    |                           |   LL_SUBRATE_IND          |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_Ack                  |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    | LE Subrate Change         |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(central_loc, test_subrating_ind_accept)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;

	struct node_rx_subrate_change su = {  };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_subrate_update(&conn, SUBRATE_MIN, SUBRATE_MAX, LATENCY, CONTINUATION_NUMBER, SUPERVISION_TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &subrate_ind);
	lt_rx_q_is_empty(&conn);

	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &su);
	ut_rx_q_is_empty();

	/* Release Ntf */
	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());

}

/*
 * Peripheral-initiated Subrating Request procedure.
 * Peripheral requests start of subrating, Central accepts.
 *
 * +----+                        +----+                    +----+
 * | UT |                        | LL |                    | LT |
 * +----+                        +----+                    +----+
 *    |                           |                           |
 *    | LE Subrate Request        |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    | Command status            |                           |
 *    |<--------------------------|                           |
 *    |                           |   LL_SUBRATE_REQ          |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_SUBRATE_IND          |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           |   LL_Ack                  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    | LE Subrate Change         |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(periph_loc, test_subrating_req_accept)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	//struct pdu_data *pdu;

	struct node_rx_subrate_change su = {  };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_subrate_request(&conn, SUBRATE_MIN, SUBRATE_MAX, LATENCY, CONTINUATION_NUMBER, SUPERVISION_TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &subrate_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_SUBRATE_IND, &conn, &subrate_ind);

	/* Done */
	event_done(&conn);

	event_prepare(&conn);
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &su);
	ut_rx_q_is_empty();

	/* Release Ntf */
	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral-initiated Subrating Request procedure.
 * Peripheral requests start of subrating, Central rejects.
 *
 * +----+                        +----+                    +----+
 * | UT |                        | LL |                    | LT |
 * +----+                        +----+                    +----+
 *    |                           |                           |
 *    | LE Subrate Request        |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    | Command status            |                           |
 *    |<--------------------------|                           |
 *    |                           |   LL_SUBRATE_REQ          |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_REJECT_EXT_IND       |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           |   LL_Ack                  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    | LE Subrate Change         |                           |
 *    | Status=Error code         |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(periph_loc, test_subrating_req_reject)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	//struct pdu_data *pdu;

	struct node_rx_subrate_change su = {  };
	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE
	};

	//TODO: Get an accurate remote feature
	struct pdu_data_llctrl_reject_ind reject_ind = { .error_code =
								 BT_HCI_ERR_UNSUPP_REMOTE_FEATURE };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_subrate_request(&conn, SUBRATE_MIN, SUBRATE_MAX, LATENCY, CONTINUATION_NUMBER, SUPERVISION_TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &subrate_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	event_prepare(&conn);
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	ut_rx_node(NODE_SUBRATE_CHANGE, &ntf, &su);
	ut_rx_q_is_empty();

	/* Release Ntf */
	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

#if 0
/*
 * Peripheral-initiated Connection Parameters Request procedure.
 * Peripheral requests change in LE connection parameters, central’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(periph_loc, test_conn_update_periph_loc_accept)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT, NULL);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);
	//conn_param_req.reference_conn_event_count = event_counter(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	//conn_update_ind.instant = event_counter(&conn) + 6U;
	//instant = conn_update_ind.instant;
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

	/* Done */
	event_done(&conn);

	/* */
	//while (!is_instant_reached(&conn, instant)) {
	//	/* Prepare */
	//	event_prepare(&conn);

	//	/* Tx Queue should NOT have a LL Control PDU */
	//	lt_rx_q_is_empty(&conn);

	//	/* Done */
	//	event_done(&conn);

	//	/* There should NOT be a host notification */
	//	ut_rx_q_is_empty();
	//}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	release_ntf(ntf);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}
#endif

ZTEST_SUITE(central_loc, NULL, NULL, conn_update_setup, NULL, NULL);
ZTEST_SUITE(periph_loc, NULL, NULL, conn_update_setup, NULL, NULL);
