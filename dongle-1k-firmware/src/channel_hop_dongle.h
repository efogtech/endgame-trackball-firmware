/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Dongle-side (PRX) channel-hop manager. Owns quarantine + committed-next
 * state, plus a peer-idle flag fed by ESB_PKT_IDLE that gates the silence
 * watchdog so a genuinely idle link (user away / BLE slot switched) does
 * not trigger spurious hops.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

void channel_hop_dongle_init(uint8_t initial_channel);

/* Called from main.c when the dongle enters/leaves STATE_PAIRED. The
 * silence watchdog only runs while paired; otherwise PAIR_REQ / VERIFY
 * traffic would constantly rearm or drop it during the handshake. */
void channel_hop_dongle_set_paired(bool paired);

/* RX dispatch helpers called from main.c on_esb_rx.
 *   note_rx_active: any packet OTHER than ESB_PKT_IDLE. Flags peer not
 *     idle and rearms the watchdog.
 *   note_rx_idle:   ESB_PKT_IDLE specifically. Flags peer idle and
 *     cancels the watchdog.
 */
void channel_hop_dongle_note_rx_active(void);
void channel_hop_dongle_note_rx_idle(void);

/* RX dispatch for CHANNEL_HOP_PROPOSAL packets. Updates committed_next
 * and queues a CHANNEL_HOP_CONFIRM ACK payload on pipe 1. */
void channel_hop_dongle_on_rx_proposal(const uint8_t *data, uint8_t len);

/* RX dispatch for HOP_OFFER (cooperative-hop handshake start). MUST be
 * called BEFORE channel_hop_dongle_note_rx_active() in the on_esb_rx
 * dispatch path — the rejection logic inspects m_speculative_prev /
 * m_rollback_active before they get cleared by note_rx_active. Queues
 * a HOP_ACCEPT in the pipe-1 ACK and arms the commit timer for
 * (hop_in_ms - PRX_SETUP_BUDGET_MS) from now. */
void channel_hop_dongle_on_rx_offer(const uint8_t *data, uint8_t len);

/* Called from main.c after every inbound packet has been fully dispatched
 * (so on_rx_proposal has had a chance to populate committed_next first).
 * If committed_next is still INVALID and the peer is known-active, queues
 * an ESB_PKT_CHANNEL_HOP_REQUEST on the pipe-1 ACK so the endpoint sends
 * a fresh PROPOSAL on its next TX rather than waiting for its 60 s steady-
 * state cadence. No-op when the request would be redundant. */
void channel_hop_dongle_after_rx(uint8_t pipe);

/* Shell / debug getters. */
uint8_t channel_hop_dongle_get_committed(void);
uint8_t channel_hop_dongle_get_quarantine_count(void);
bool    channel_hop_dongle_is_quarantined(uint8_t channel);
bool    channel_hop_dongle_is_peer_idle(void);
