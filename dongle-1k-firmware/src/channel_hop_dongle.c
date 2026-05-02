/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk_esb/protocol.h>
#include <zmk_esb/channel_hop.h>
#include "channel_hop_dongle.h"
#include "esb_prx.h"
#include "led_status.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle_chhop, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)

/* Compile-time invariants for the hop / rollback timing knobs. Cross-firmware
 * constraints against ZMK_ESB_ENDPOINT_* live on the keyboard side and cannot
 * be checked here (the endpoint Kconfig is not visible in the dongle build);
 * see each symbol's help text for those. */
BUILD_ASSERT(CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS >
             CONFIG_DONGLE_CHANNEL_HOP_RX_SILENCE_MS,
             "ROLLBACK_SILENCE_MS must exceed RX_SILENCE_MS so the speculative "
             "hop wins the first attempt before the rollback dwell engages");
BUILD_ASSERT(CONFIG_DONGLE_COOP_HOP_VALIDATE_MS <=
             CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS,
             "COOP_HOP_VALIDATE_MS must be <= ROLLBACK_SILENCE_MS so the "
             "coop-hop validate path always preempts the rollback path");
BUILD_ASSERT(CONFIG_DONGLE_CHANNEL_HOP_VALIDATE_FAIL_ROLLBACK_MS == 0 ||
             CONFIG_DONGLE_CHANNEL_HOP_VALIDATE_FAIL_ROLLBACK_MS <
             CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS,
             "VALIDATE_FAIL_ROLLBACK_MS (when nonzero) must be < "
             "ROLLBACK_SILENCE_MS or the fast-track timer would actually "
             "delay rollback entry instead of speeding it up");
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_PEEK_IDLE)
BUILD_ASSERT(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_DWELL_MS <
             CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS,
             "PEEK_IDLE_DWELL_MS must be < PEEK_IDLE_INTERVAL_MS so the "
             "dongle spends some time on its working channel between peeks");
#endif

static struct quarantine_state m_quarantine;
static uint8_t m_committed_next = CHANNEL_HOP_INVALID;

/* Armed only while paired AND peer is known-active. ESB_PKT_IDLE flips
 * peer_idle to true and cancels the watchdog; any other RX flips it back
 * to false and reschedules. Start with peer_idle=true so we do not hop
 * before we have ever heard from the endpoint. */
static bool m_paired;
static bool m_peer_idle = true;

/* Speculative-hop state. When the silence watchdog trips we hop to
 * committed_next without quarantining the old channel, and arm the
 * validation timer. If any non-IDLE RX arrives before validation expires
 * we were right: quarantine the old channel and clear speculative state.
 * If validation expires with no RX, we were wrong (peer is genuinely
 * gone): revert to the saved old channel and do NOT quarantine.
 *
 * CHANNEL_HOP_INVALID in m_speculative_prev means "no speculative hop in
 * progress". */
static uint8_t m_speculative_prev = CHANNEL_HOP_INVALID;

/* The channel hop_work_fn handed to esb_prx_set_channel(). Captured at hop
 * time so validate_work_fn can log the actual speculative target even if
 * another state machine (e.g. rollback dwell) moved the radio in the
 * meantime — esb_prx_get_channel() at validate time is unreliable. */
static uint8_t m_speculative_target = CHANNEL_HOP_INVALID;

/* k_uptime_get_32() deadline; while now is before it, hop_work_fn defers
 * instead of firing a fresh speculative hop. Set when one resolves
 * (validated or reverted). uint32_t reads/writes are atomic on Cortex-M;
 * comparison uses signed subtraction to handle the 49-day wraparound. */
static uint32_t m_spec_cooldown_until;

/* Cooperative-hop state. Set when an inbound HOP_OFFER has been
 * accepted (HOP_ACCEPT queued in the ACK payload, commit_work armed).
 * Cleared by the commit work itself or by a hard teardown
 * (set_paired(false)).
 *
 * `m_coop_hop_target` is held SEPARATELY from `m_committed_next` so a
 * stale PROPOSAL arriving mid-handshake cannot poison the agreed
 * channel. The commit work uses m_coop_hop_target; the speculative
 * silence-watchdog path uses m_committed_next. They never alias.
 *
 * `m_coop_hop_seq_last` is purely for log-line dedup; ESB hardware
 * already dedupes retransmitted packets, but if a stray re-OFFER
 * with the same seq makes it through, we silently coalesce.
 *
 * `m_coop_hop_revert_to` is the channel we just left, set in
 * coop_hop_commit_work_fn AFTER the radio retunes. While set, the
 * coop hop is "unconfirmed": coop_hop_validate_work_fn will revert
 * if no RX arrives within COOP_HOP_VALIDATE_MS, and the deferred
 * old-channel quarantine fires from note_rx_active/note_rx_idle when
 * RX confirms the endpoint joined. INVALID = no coop hop in flight,
 * or the most recent one has been resolved. Mirrors m_speculative_prev
 * but stays separate because the two paths use different quarantine
 * durations (CHANNEL_QUARANTINE_MS vs COOP_HOP_QUARANTINE_MS). */
static bool    m_coop_hop_armed;
static uint8_t m_coop_hop_target = CHANNEL_HOP_INVALID;
static uint8_t m_coop_hop_seq_last;
static uint8_t m_coop_hop_revert_to = CHANNEL_HOP_INVALID;

/* Rollback state. Engaged after DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS
 * of total silence — typically meaning the keyboard rebooted and came
 * up on the DTS default channel. We dwell across up to three distinct
 * channels in round-robin until any RX arrives:
 *   - DONGLE_ESB_RF_CHANNEL      (DTS default)
 *   - m_rollback_last_ch         (where we were when silence began)
 *   - m_rollback_last_tried      (most recent failed speculative-hop
 *                                  target — could be the right channel
 *                                  if validation simply got unlucky)
 * Duplicates and INVALID slots are skipped. Exit on RX (confirmed find). */
static bool    m_rollback_active;
static uint8_t m_rollback_last_ch    = CHANNEL_HOP_INVALID;
static uint8_t m_rollback_last_tried = CHANNEL_HOP_INVALID;
static uint8_t m_rollback_idx;

static struct k_work_delayable m_silence_work;
static struct k_work_delayable m_validate_work;
static struct k_work_delayable m_rollback_silence_work;
static struct k_work_delayable m_rollback_dwell_work;
static struct k_work_delayable m_coop_hop_commit_work;
static struct k_work_delayable m_coop_hop_validate_work;
static struct k_work           m_hop_work;

static void rearm_silence_if_needed(void);

static void hop_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    const uint8_t current = esb_prx_get_channel();

    if (m_rollback_active) {
        /* Rollback owns the radio; its dwell cycle revisits
         * m_rollback_last_tried so a missed speculative target is not
         * lost. Symmetric to enter_rollback's guard against
         * m_speculative_prev — an m_hop_work submitted by silence_work_fn
         * on the same tick rollback_silence_work_fn enters rollback would
         * otherwise stomp on the dwell cycle. */
        return;
    }
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        /* Already speculating; watchdog shouldn't even be running. Guard. */
        return;
    }
    if (m_committed_next == CHANNEL_HOP_INVALID) {
        /* No channel to hop to. Keep quiet — the endpoint is in fast-
         * retry negotiate mode and will populate committed_next as soon
         * as any PROPOSAL reaches us. Spamming a WRN every
         * RX_SILENCE_MS doesn't help. */
        LOG_WRN("silence watchdog: no committed next yet, staying on %u", current);
        return;
    }
    if (m_committed_next == current) {
        m_committed_next = CHANNEL_HOP_INVALID;
        return;
    }

#if CONFIG_DONGLE_CHANNEL_HOP_SPECULATIVE_COOLDOWN_MS > 0
    {
        const uint32_t now = k_uptime_get_32();
        const int32_t remaining = (int32_t)(m_spec_cooldown_until - now);
        if (remaining > 0) {
            /* Defer rather than cancel: a fresh PROPOSAL after cooldown
             * should still be honored. Re-arm silence_work for the
             * remainder so we re-evaluate then. */
            LOG_INF("speculative cooldown: %d ms left, deferring hop on %u",
                    remaining, current);
            k_work_reschedule(&m_silence_work, K_MSEC(remaining));
            return;
        }
        /* Cooldown expired. The silence watchdog firing post-cooldown is
         * the right signal to hop: an "extend cooldown on any RX during
         * cooldown" rule would lock out hops on a weak link, because
         * trickle RX (REQUEST-triggered PROPOSAL bursts, occasional HID)
         * keeps the latch flipped indefinitely while the link is clearly
         * degraded. */
    }
#endif

    /* Speculative hop: do NOT quarantine yet — if RX resumes we confirm,
     * otherwise we revert. */
    const uint8_t target = m_committed_next;
    m_committed_next = CHANNEL_HOP_INVALID;

    const int err = esb_prx_set_channel(target);
    if (err) {
        LOG_ERR("channel hop %u -> %u failed: %d", current, target, err);
        return;
    }
    led_status_flash_nolink();
    m_speculative_prev = current;
    m_speculative_target = target;
    k_work_reschedule(&m_validate_work, K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_VALIDATE_MS));
    LOG_INF("speculative hop: %u -> %u", current, target);
}

static void validate_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    /* rx_thread (K_PRIO_COOP) is higher priority than the system workqueue
     * (preemptible) and can preempt us mid-function. If it does so between
     * the guard and the m_speculative_prev=INVALID write, confirm_speculative
     * would call k_work_cancel_delayable on this *already-running* handler —
     * which per Zephyr semantics only prevents future rescheduling, not the
     * current invocation. We would then resume and revert despite the hop
     * having just been confirmed. Bracket the read+clear with k_sched_lock
     * so the transition is atomic wrt other threads. Radio I/O stays out
     * of the lock — it may yield. */
    k_sched_lock();
    if (m_speculative_prev == CHANNEL_HOP_INVALID) {
        k_sched_unlock();
        return;
    }
    const uint8_t revert_to = m_speculative_prev;
    const uint8_t failed_target = m_speculative_target;
    m_speculative_prev = CHANNEL_HOP_INVALID;
    m_speculative_target = CHANNEL_HOP_INVALID;
    /* Don't trust committed_next anymore — the PROPOSAL that produced it
     * is clearly out of date with whatever the endpoint is actually on
     * (or the endpoint is gone entirely). Next PROPOSAL will re-populate. */
    m_committed_next = CHANNEL_HOP_INVALID;
    k_sched_unlock();

    const int err = esb_prx_set_channel(revert_to);
    if (err) {
        LOG_ERR("revert %u -> %u failed: %d", failed_target, revert_to, err);
        return;
    }
    led_status_set_link_lost(true);
    /* Remember the failed target for the rollback dwell cycle: validation
     * can fail for benign reasons (one missed packet) and that channel may
     * still be where the peer lives. */
    m_rollback_last_tried = failed_target;
    /* Clear cooldown on validate-fail. The cooldown defends speculative
     * hops against a stale PROPOSAL re-firing the same wrong target;
     * that risk is gone because committed_next was just cleared above.
     * Refreshing it here would also turn FAIL_ROLLBACK_MS into a dead
     * letter: the rollback_silence_work scheduled below would land
     * inside the cooldown window and enter_rollback would defer instead
     * of entering the dwell cycle, so the fast-track-to-rollback path
     * never wins on a real desync. */
    m_spec_cooldown_until = 0;
    LOG_INF("speculation failed, reverted %u → %u, committed_next cleared",
            failed_target, revert_to);
    /* rollback_silence_work is still ticking down from the last real RX.
     * Additionally rearm the short silence watchdog so that if a stray
     * PROPOSAL repopulates m_committed_next during the rollback window,
     * we can fire a second speculative hop without first needing real RX
     * (which is exactly what's missing when the link is torn down). */
    if (m_paired && !m_peer_idle) {
        k_work_reschedule(&m_silence_work,
                          K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_RX_SILENCE_MS));
        /* A failed validation is positive evidence of desync — we know
         * the keyboard is not where committed_next pointed. Skip the
         * remainder of ROLLBACK_SILENCE_MS and arm
         * the rollback dwell on the much shorter desync deadline so
         * the keyboard can be re-found via the dwell cycle promptly.
         * The standard rollback_silence_work timer is also still armed,
         * so this is a "whichever fires first" race — the short timer
         * wins on a real desync, the long one wins if real RX recovers
         * in the meantime (note_rx_active cancels both). */
        if (CONFIG_DONGLE_CHANNEL_HOP_VALIDATE_FAIL_ROLLBACK_MS > 0) {
            k_work_reschedule(&m_rollback_silence_work,
                              K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_VALIDATE_FAIL_ROLLBACK_MS));
        }
    }
}

static void confirm_speculative_if_any(void) {
    if (m_speculative_prev == CHANNEL_HOP_INVALID) {
        return;
    }
    /* Non-IDLE RX on the new channel — the hop was correct. */
    k_work_cancel_delayable(&m_validate_work);
    quarantine_add(&m_quarantine, m_speculative_prev,
                   CONFIG_DONGLE_CHANNEL_QUARANTINE_MS);
    LOG_INF("validated speculative hop on %u, quarantined %u for %u ms",
            esb_prx_get_channel(), m_speculative_prev,
            (unsigned)CONFIG_DONGLE_CHANNEL_QUARANTINE_MS);
    m_speculative_prev = CHANNEL_HOP_INVALID;
    m_speculative_target = CHANNEL_HOP_INVALID;
    m_spec_cooldown_until = k_uptime_get_32() +
        CONFIG_DONGLE_CHANNEL_HOP_SPECULATIVE_COOLDOWN_MS;

    /* Pre-arm the silence-watchdog target with a local pick. hop_work_fn
     * cleared m_committed_next before the speculative hop, and there is
     * no path inside the dongle that ever sets it again — only an
     * inbound PROPOSAL from the keyboard does. If the keyboard's
     * negotiate_work happens to be in slow tempo (60 s INTERVAL) when
     * validation lands, the dongle can sit with m_committed_next=INVALID
     * for tens of seconds. Any silence_work in that window then falls
     * through hop_work_fn's "no committed next yet, staying on N" branch
     * and the safety net is gone — the keyboard transitions to rollback
     * dwell instead of hopping to a useful target.
     *
     * Local pick is "best guess" — the keyboard may pick differently
     * for its next hop — but a guess is strictly better than INVALID:
     *   - If a real desync follows, we hop and either the keyboard is
     *     also there (validate succeeds) or the keyboard's post_hop_burst
     *     reconverges via PROPOSAL/CONFIRM after the inevitable revert.
     *   - If the silence was just a brief gap with no real desync,
     *     validate-fail → revert puts us back on the same channel
     *     within VALIDATE_MS, identical to today's failed-speculation
     *     path — strictly better than today's "silently stay deaf"
     *     path.
     * The next inbound PROPOSAL overwrites this guess with the
     * keyboard-agreed value, so the local pick has at most a single
     * silence-cycle's worth of influence. */
    const uint8_t current = esb_prx_get_channel();
    m_committed_next = channel_hop_pick(
        &m_quarantine,
        CONFIG_DONGLE_CHANNEL_QUARANTINE_MIN_DISTANCE,
        current);
}

static void silence_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_paired || m_peer_idle) {
        return;
    }
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        return;
    }
    k_work_submit(&m_hop_work);
}

static void coop_hop_commit_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_coop_hop_armed) {
        /* Disarmed by note_rx_active (endpoint kept talking on the old
         * channel = it aborted) or by set_paired(false). */
        return;
    }
    const uint8_t current = esb_prx_get_channel();
    const uint8_t target = m_coop_hop_target;

    if (target == CHANNEL_HOP_INVALID || target == current) {
        m_coop_hop_armed = false;
        return;
    }

    /* Mirror the speculative-hop guard: if a real speculative hop
     * snuck in between OFFER and now, leave it to validate_work. */
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        m_coop_hop_armed = false;
        return;
    }

    const int err = esb_prx_set_channel(target);
    if (err) {
        LOG_ERR("coop hop %u -> %u failed: %d", current, target, err);
        m_coop_hop_armed = false;
        return;
    }

    led_status_flash_nolink();
    m_committed_next = CHANNEL_HOP_INVALID;
    m_spec_cooldown_until = k_uptime_get_32() +
        CONFIG_DONGLE_COOP_HOP_COOLDOWN_MS;
    m_coop_hop_armed = false;
    m_coop_hop_target = CHANNEL_HOP_INVALID;
    /* Track the channel we just left so coop_hop_validate_work_fn can
     * revert if the endpoint never joined us (it missed the ACCEPT ACK
     * and aborted while we hopped solo). The deferred quarantine for
     * `current` lives in note_rx_active/note_rx_idle: only confirmed
     * coop hops poison `current`, otherwise revert lands cleanly. */
    m_coop_hop_revert_to = current;

    LOG_INF("coop hop committed: %u -> %u (validating)", current, target);

    /* Arm the coop-hop validation timer — mirrors the speculative-hop
     * validate/revert pattern. If non-IDLE/IDLE RX arrives on the new
     * channel before COOP_HOP_VALIDATE_MS, the rx hooks confirm the hop
     * (quarantine `current`, clear m_coop_hop_revert_to). Otherwise
     * coop_hop_validate_work_fn reverts the radio to `current`.
     *
     * Push out rollback_silence_work past cooldown + the normal silence
     * window so its baseline reflects "we just hopped" rather than
     * whichever RX preceded the OFFER. */
    if (m_paired && !m_peer_idle) {
        k_work_reschedule(&m_coop_hop_validate_work,
                          K_MSEC(CONFIG_DONGLE_COOP_HOP_VALIDATE_MS));
        k_work_reschedule(&m_rollback_silence_work,
                          K_MSEC(CONFIG_DONGLE_COOP_HOP_COOLDOWN_MS +
                                 CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS));
    }
}

static void coop_hop_validate_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_paired || m_peer_idle) {
        return;
    }
    if (m_coop_hop_revert_to == CHANNEL_HOP_INVALID) {
        /* Already confirmed by an rx hook between schedule and fire. */
        return;
    }
    /* If a speculative hop or rollback dwell snuck in between commit
     * and validate, leave them to their own machinery. */
    if (m_speculative_prev != CHANNEL_HOP_INVALID || m_rollback_active) {
        m_coop_hop_revert_to = CHANNEL_HOP_INVALID;
        return;
    }

    const uint8_t failed_target = esb_prx_get_channel();
    const uint8_t revert_to     = m_coop_hop_revert_to;
    m_coop_hop_revert_to = CHANNEL_HOP_INVALID;

    const int err = esb_prx_set_channel(revert_to);
    if (err) {
        LOG_ERR("coop revert %u -> %u failed: %d", failed_target, revert_to, err);
        return;
    }
    led_status_set_link_lost(true);

    /* The abandoned coop target was never quarantined (we deferred), so
     * nothing to lift. Stash it for any later rollback dwell: the
     * common orphan case is "endpoint missed ACCEPT ACK and stayed
     * put", but the less-common false-negative case is "endpoint did
     * hop but link too noisy to confirm in VALIDATE_MS" — for that
     * case, the failed target IS where the endpoint lives, so the
     * dwell cycle should visit it. Same defensive logic as the
     * speculative validate path. Reset spec_cooldown so the silence /
     * rollback timers below get a fresh baseline. */
    m_rollback_last_tried = failed_target;
    m_committed_next      = CHANNEL_HOP_INVALID;
    m_spec_cooldown_until = 0;
    LOG_INF("coop hop validation failed, reverted %u -> %u",
            failed_target, revert_to);

    rearm_silence_if_needed();
}

void channel_hop_dongle_on_rx_offer(const uint8_t *data, uint8_t len) {
    if (len < sizeof(struct esb_pkt_hop_offer)) {
        return;
    }
    if (!m_paired) {
        /* Stranger keyboard: don't let it move our radio. */
        return;
    }
    const struct esb_pkt_hop_offer *o = (const void *)data;
    if (o->target_channel >= CHANNEL_HOP_CHANNEL_COUNT) {
        return;
    }

    const uint8_t current = esb_prx_get_channel();

    /* Reject paths — degrade gracefully to existing recovery logic. */
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        /* Mid-validation: receiving an OFFER on the OLD channel means
         * the endpoint did not follow our speculative hop. Let
         * validate_work resolve before considering a coop hop. */
        LOG_INF("coop OFFER ignored: speculative hop in flight");
        return;
    }
    if (m_rollback_active) {
        /* Rollback is dwell-cycling channels; coop-hop scheduling on
         * top of that produces unpredictable channels. */
        LOG_INF("coop OFFER ignored: rollback active");
        return;
    }
    if (o->target_channel == current) {
        /* Pathological: nothing to do. */
        return;
    }

    /* Pick the agreed channel: take the OFFER's target unless it is
     * locally quarantined, in which case counter-propose. Mirrors the
     * existing PROPOSAL/CONFIRM behavior. */
    uint8_t agreed;
    bool accepted;
    if (!quarantine_is(&m_quarantine, o->target_channel)) {
        agreed = o->target_channel;
        accepted = true;
    } else {
        const uint8_t counter = channel_hop_pick(
            &m_quarantine,
            CONFIG_DONGLE_CHANNEL_QUARANTINE_MIN_DISTANCE,
            current);
        if (counter == CHANNEL_HOP_INVALID || counter == current) {
            /* No good alternative; honor the offered channel anyway —
             * the endpoint already decided this was the best option,
             * and a quarantine that has nothing better to suggest is
             * just a hint at this point. */
            agreed = o->target_channel;
            accepted = true;
        } else {
            agreed = counter;
            accepted = false;
        }
    }

    /* Queue the ACCEPT in the ACK payload BEFORE arming the timer.
     * If the queue fails (FIFO full), abort — the endpoint will not
     * see an ACCEPT and won't arm its own timer either, so the
     * handshake aborts symmetrically.
     *
     * Flush first: if a low-priority payload (e.g. LINK_STATS) is
     * already queued, the endpoint's TX_SUCCESS on the OFFER would
     * pick it up instead of the ACCEPT and the handshake would silently
     * fail. The ACCEPT must ride the first ACK after the OFFER arrived. */
    esb_prx_flush_acks();
    struct esb_pkt_hop_accept resp = {
        .type           = ESB_PKT_HOP_ACCEPT,
        .agreed_channel = agreed,
        .accepted       = accepted ? 1U : 0U,
        .seq            = o->seq,
    };
    const int err = esb_prx_queue_ack(ESB_PIPE_DATA,
                                      (uint8_t *)&resp, sizeof(resp));
    if (err) {
        LOG_WRN("coop ACCEPT queue failed: %d", err);
        return;
    }

    /* Arm the commit timer. Subtract the PRX setup budget so the
     * dongle's stop_rx → set_channel → start_rx sequence completes by
     * the same wall-clock instant the endpoint flips. */
    uint32_t arm_in_ms = o->hop_in_ms;
    if (arm_in_ms > CONFIG_DONGLE_COOP_HOP_PRX_SETUP_BUDGET_MS) {
        arm_in_ms -= CONFIG_DONGLE_COOP_HOP_PRX_SETUP_BUDGET_MS;
    } else {
        arm_in_ms = 0;
    }

    m_coop_hop_target = agreed;
    m_coop_hop_seq_last = o->seq;
    m_coop_hop_armed = true;
    /* Seed the rollback dwell cycle with the pre-hop channel as a
     * defence-in-depth fallback: coop_hop_validate_work_fn handles the
     * common orphan case (endpoint missed the ACCEPT ACK) by reverting
     * directly within COOP_HOP_VALIDATE_MS, but if that work somehow
     * fails to fire (e.g. workqueue starvation), the dwell cycle still
     * has a concrete candidate to visit. Harmless when the hop succeeds
     * normally: rollback only builds the cycle if silence_work +
     * rollback_silence_work both trip. */
    m_rollback_last_tried = current;
    k_work_reschedule(&m_coop_hop_commit_work, K_MSEC(arm_in_ms));

    LOG_INF("coop OFFER accepted: target=%u agreed=%u seq=%u arm_in=%ums (%s)",
            o->target_channel, agreed, o->seq, (unsigned)arm_in_ms,
            accepted ? "accept" : "counter");
}

static void exit_rollback(void) {
    if (!m_rollback_active) {
        return;
    }
    m_rollback_active = false;
    m_rollback_idx = 0;
    k_work_cancel_delayable(&m_rollback_dwell_work);
}

/* Build the active dwell cycle (default + last_ch + last_tried, deduped
 * and dropping INVALID). Returns the number of slots filled. */
static uint8_t build_rollback_cycle(uint8_t out[3]) {
    const uint8_t default_ch = CONFIG_DONGLE_ESB_RF_CHANNEL;
    uint8_t n = 0;
    out[n++] = default_ch;
    if (m_rollback_last_ch != CHANNEL_HOP_INVALID &&
        m_rollback_last_ch != default_ch) {
        out[n++] = m_rollback_last_ch;
    }
    if (m_rollback_last_tried != CHANNEL_HOP_INVALID &&
        m_rollback_last_tried != default_ch &&
        m_rollback_last_tried != m_rollback_last_ch) {
        out[n++] = m_rollback_last_tried;
    }
    return n;
}

static void rollback_dwell_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_paired || m_peer_idle || !m_rollback_active) {
        return;
    }
    uint8_t cycle[3];
    const uint8_t n = build_rollback_cycle(cycle);
    if (n <= 1) {
        /* Lost all alternatives mid-cycle (shouldn't happen — last_ch
         * doesn't get cleared during rollback). Stay put and retry. */
        k_work_reschedule(&m_rollback_dwell_work,
                          K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_DWELL_MS));
        return;
    }
    /* Step to the next slot. Recomputing the cycle each tick means
     * m_rollback_idx may now point past a slot that disappeared, but
     * mod-n keeps it in range; we accept that we may revisit a channel
     * out of order rather than carry stale indices. */
    const uint8_t next_idx = (m_rollback_idx + 1) % n;
    const uint8_t target = cycle[next_idx];
    const int err = esb_prx_set_channel(target);
    if (err) {
        LOG_ERR("rollback set_channel(%u) failed: %d", target, err);
        /* Keep trying on a later tick (don't advance idx). */
    } else {
        m_rollback_idx = next_idx;
        LOG_DBG("rollback dwell → %u (slot %u/%u)", target, next_idx, n);
    }
    k_work_reschedule(&m_rollback_dwell_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_DWELL_MS));
}

static void enter_rollback(void) {
    if (m_rollback_active) {
        return;
    }
    if (!m_paired || m_peer_idle) {
        return;
    }
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        /* A speculative hop is still being validated — let that resolve
         * first. The rollback watchdog will re-fire and we'll try again. */
        return;
    }

    /* Rollback does NOT gate on m_spec_cooldown_until. The cooldown's
     * job is to defend speculative hops from a stale PROPOSAL re-firing
     * the same wrong target; rollback firing means we have already had
     * ROLLBACK_SILENCE_MS of total silence — independent positive
     * evidence of a dead link, not a candidate for "wait it out".
     * Coop hops separately push rollback_silence_work out by
     * COOP_HOP_COOLDOWN_MS + ROLLBACK_SILENCE_MS in
     * coop_hop_commit_work_fn, so a healthy-but-quiet post-coop window
     * does not reach this path. A hypothetical "rollback honors
     * cooldown" rule would make FAIL_ROLLBACK_MS a no-op: a validate-
     * fail-then-rollback sequence would land inside the cooldown the
     * fail itself just set, and defer instead of entering the dwell. */
    const uint8_t current = esb_prx_get_channel();
    const uint8_t default_ch = CONFIG_DONGLE_ESB_RF_CHANNEL;

    /* Refresh "last active" with where we are now if it's a genuine
     * alternative. Otherwise keep whatever m_rollback_last_ch already
     * held from a previous session — that was a real candidate the
     * dongle was on before it bounced back to default. */
    if (current != default_ch) {
        m_rollback_last_ch = current;
    }

    /* Need at least one distinct alternative channel for a meaningful
     * dwell cycle. Both last_ch and last_tried are eligible. */
    uint8_t cycle[3];
    const uint8_t n = build_rollback_cycle(cycle);
    if (n <= 1) {
        LOG_INF("rollback skipped: no distinct alternative channels (current=%u default=%u last_ch=%u last_tried=%u)",
                current, default_ch, m_rollback_last_ch, m_rollback_last_tried);
        return;
    }

    m_rollback_active = true;
    /* Start on default — a rebooted keyboard is more likely to be there. */
    if (current != default_ch) {
        const int err = esb_prx_set_channel(default_ch);
        if (err) {
            LOG_ERR("rollback initial switch to default (%u) failed: %d",
                    default_ch, err);
            m_rollback_active = false;
            return;
        }
    }
    led_status_set_link_lost(true);
    /* default_ch is always cycle[0]. */
    m_rollback_idx = 0;
    LOG_INF("rollback: default=%u last_ch=%u last_tried=%u (%u-channel cycle), starting on default",
            default_ch, m_rollback_last_ch, m_rollback_last_tried, n);
    k_work_reschedule(&m_rollback_dwell_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_DWELL_MS));
}

static void rollback_silence_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    enter_rollback();
}

static void rearm_silence_if_needed(void) {
    if (!m_paired || m_peer_idle) {
        k_work_cancel_delayable(&m_silence_work);
        k_work_cancel_delayable(&m_rollback_silence_work);
        return;
    }
    k_work_reschedule(&m_silence_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_RX_SILENCE_MS));
    k_work_reschedule(&m_rollback_silence_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_HOP_ROLLBACK_SILENCE_MS));
}

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_PEEK_IDLE)

/* Idle-peek: while paired and peer-idle on a non-default channel, every
 * CHANNEL_PEEK_IDLE_INTERVAL_MS briefly hop to the DTS-default channel
 * and listen for CHANNEL_PEEK_IDLE_DWELL_MS. The rollback dwell cycle
 * covers the same "keyboard rebooted onto default" case but is gated
 * on !peer_idle, so a long idle window leaves that recovery off. This
 * is the idle-window counterpart. Single work item is reused for both
 * phases: m_peek_active distinguishes dwell (true) from interval (false). */
static bool    m_peek_active;
static uint8_t m_peek_pre_channel = CHANNEL_HOP_INVALID;
static struct k_work_delayable m_peek_work;

static void peek_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    const uint8_t current = esb_prx_get_channel();
    const uint8_t default_ch = CONFIG_DONGLE_ESB_RF_CHANNEL;

    if (m_peek_active) {
        /* Dwell phase: listen window ended. */
        if (!m_peer_idle) {
            /* note_rx_active fired during the dwell — the peer is here
             * on the default channel. Leave the radio put. Clearing
             * peek state also happens from peek_on_peer_active via
             * note_rx_active's hook, but the dispatch thread may not
             * have preempted us yet; double-clear is harmless. */
            m_peek_active = false;
            m_peek_pre_channel = CHANNEL_HOP_INVALID;
            LOG_INF("peek caught peer on %u", current);
            return;
        }
        const uint8_t revert_to = m_peek_pre_channel;
        m_peek_active = false;
        m_peek_pre_channel = CHANNEL_HOP_INVALID;
        if (revert_to != CHANNEL_HOP_INVALID && revert_to != current) {
            const int err = esb_prx_set_channel(revert_to);
            if (err) {
                LOG_ERR("peek revert %u -> %u failed: %d", current, revert_to, err);
            } else {
                LOG_DBG("peek revert: %u -> %u (nothing heard)", current, revert_to);
            }
        }
        if (m_paired && m_peer_idle &&
            esb_prx_get_channel() != default_ch) {
            k_work_reschedule(&m_peek_work,
                              K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS));
        }
        return;
    }

    /* Interval phase. */
    if (!m_paired || !m_peer_idle) {
        /* Conditions slipped between scheduling and firing. The state-
         * transition hooks (peek_on_peer_active, peek_on_unpair) are
         * responsible for rearming if conditions come back. */
        return;
    }
    if (current == default_ch) {
        /* Already on default — nothing to peek at. Recheck next interval
         * in case another hop moves the radio. */
        k_work_reschedule(&m_peek_work,
                          K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS));
        return;
    }
    if (m_speculative_prev != CHANNEL_HOP_INVALID ||
        m_coop_hop_armed || m_rollback_active) {
        /* Another state machine owns the radio. These shouldn't
         * normally coexist with peer_idle=true (each requires peer
         * activity to arm), but guard defensively. */
        k_work_reschedule(&m_peek_work,
                          K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS));
        return;
    }

    /* Race note: peer_idle is read here, but an RX arriving between this
     * point and esb_prx_set_channel returning would flip it through the
     * dispatch thread (COOP prio 2, preempts this worker). The peek
     * would then hop off just-confirmed-active channel. Silence watchdog
     * (rearmed by note_rx_active) catches and recovers within
     * RX_SILENCE_MS — accept the narrow window rather than bracket the
     * hop with k_sched_lock (esb_prx_set_channel yields). */
    m_peek_pre_channel = current;
    const int err = esb_prx_set_channel(default_ch);
    if (err) {
        LOG_ERR("peek %u -> %u failed: %d", current, default_ch, err);
        m_peek_pre_channel = CHANNEL_HOP_INVALID;
        k_work_reschedule(&m_peek_work,
                          K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS));
        return;
    }
    m_peek_active = true;
    LOG_DBG("peek: %u -> %u (dwell %u ms)", current, default_ch,
            (unsigned)CONFIG_DONGLE_CHANNEL_PEEK_IDLE_DWELL_MS);
    k_work_reschedule(&m_peek_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_DWELL_MS));
}

static void peek_init(void) {
    m_peek_active = false;
    m_peek_pre_channel = CHANNEL_HOP_INVALID;
    k_work_init_delayable(&m_peek_work, peek_work_fn);
}

static void peek_on_unpair(void) {
    /* If we were parked on default for a peek when unpair fires, revert
     * to the pre-peek channel so we don't carry the default assignment
     * into the next pairing cycle. Harmless otherwise. */
    if (m_peek_active && m_peek_pre_channel != CHANNEL_HOP_INVALID) {
        const uint8_t current = esb_prx_get_channel();
        if (current != m_peek_pre_channel) {
            (void)esb_prx_set_channel(m_peek_pre_channel);
        }
    }
    m_peek_active = false;
    m_peek_pre_channel = CHANNEL_HOP_INVALID;
    k_work_cancel_delayable(&m_peek_work);
}

static void peek_on_peer_active(void) {
    /* Peer is no longer idle. Cancel any pending interval-phase fire.
     * If a dwell is in flight, the current channel (default) is where
     * the peer just spoke — clear peek state to abandon the pre-peek
     * channel, don't revert. */
    m_peek_active = false;
    m_peek_pre_channel = CHANNEL_HOP_INVALID;
    k_work_cancel_delayable(&m_peek_work);
}

static void rearm_peek_if_needed(void) {
    if (!m_paired || !m_peer_idle) {
        return;
    }
    if (m_peek_active) {
        /* Dwell timer already running — do not clobber. */
        return;
    }
    if (esb_prx_get_channel() == CONFIG_DONGLE_ESB_RF_CHANNEL) {
        return;
    }
    k_work_reschedule(&m_peek_work,
                      K_MSEC(CONFIG_DONGLE_CHANNEL_PEEK_IDLE_INTERVAL_MS));
}

#else /* !CONFIG_DONGLE_CHANNEL_PEEK_IDLE */

static inline void peek_init(void) {}
static inline void peek_on_unpair(void) {}
static inline void peek_on_peer_active(void) {}
static inline void rearm_peek_if_needed(void) {}

#endif /* CONFIG_DONGLE_CHANNEL_PEEK_IDLE */

void channel_hop_dongle_init(uint8_t initial_channel) {
    ARG_UNUSED(initial_channel);
    quarantine_reset(&m_quarantine);
    m_committed_next   = CHANNEL_HOP_INVALID;
    m_speculative_prev = CHANNEL_HOP_INVALID;
    m_speculative_target = CHANNEL_HOP_INVALID;
    m_spec_cooldown_until = 0;
    m_rollback_active     = false;
    m_rollback_last_ch    = CHANNEL_HOP_INVALID;
    m_rollback_last_tried = CHANNEL_HOP_INVALID;
    m_rollback_idx        = 0;
    m_coop_hop_armed       = false;
    m_coop_hop_target      = CHANNEL_HOP_INVALID;
    m_coop_hop_revert_to = CHANNEL_HOP_INVALID;
    m_coop_hop_seq_last    = 0;
    m_peer_idle = true;
    m_paired    = false;
    k_work_init_delayable(&m_silence_work, silence_work_fn);
    k_work_init_delayable(&m_validate_work, validate_work_fn);
    k_work_init_delayable(&m_rollback_silence_work, rollback_silence_work_fn);
    k_work_init_delayable(&m_rollback_dwell_work, rollback_dwell_work_fn);
    k_work_init_delayable(&m_coop_hop_commit_work, coop_hop_commit_work_fn);
    k_work_init_delayable(&m_coop_hop_validate_work, coop_hop_validate_work_fn);
    k_work_init(&m_hop_work, hop_work_fn);
    peek_init();
}

void channel_hop_dongle_set_paired(bool paired) {
    if (m_paired != paired) {
        LOG_INF("paired=%d (channel_hop %s)", paired ? 1 : 0,
                paired ? "armed" : "disarmed");
    }
    m_paired = paired;
    if (!paired) {
        m_peer_idle = true;
        m_committed_next = CHANNEL_HOP_INVALID;
        /* If we were mid-speculative-hop when unpairing, don't bother
         * reverting — the link is tearing down anyway. */
        m_speculative_prev = CHANNEL_HOP_INVALID;
        m_speculative_target = CHANNEL_HOP_INVALID;
        m_spec_cooldown_until = 0;
        k_work_cancel_delayable(&m_validate_work);
        /* Drop any in-flight cooperative-hop too. The endpoint will
         * not commit either (its own coop_hop_abort runs via the
         * disconnect path), and we don't want a stale commit_work to
         * fire after we have unpaired. */
        m_coop_hop_armed = false;
        m_coop_hop_target = CHANNEL_HOP_INVALID;
        m_coop_hop_revert_to = CHANNEL_HOP_INVALID;
        k_work_cancel_delayable(&m_coop_hop_commit_work);
        k_work_cancel_delayable(&m_coop_hop_validate_work);
        exit_rollback();
        led_status_set_link_lost(false);
        peek_on_unpair();
    }
    rearm_silence_if_needed();
    rearm_peek_if_needed();
}

/* RX on the (possibly new-after-coop-hop) channel confirms the endpoint
 * joined us. Cancel the validate timer, apply the deferred old-channel
 * quarantine, and clear the revert pointer. Distinct from the
 * "phantom-hop disarm" the comment in note_rx_active warns against:
 * by the time this runs, coop_hop_commit_work_fn has already retuned
 * the radio, so any RX must have arrived on the new channel. */
static void confirm_coop_hop_if_any(void) {
    if (m_coop_hop_revert_to == CHANNEL_HOP_INVALID) {
        return;
    }
    k_work_cancel_delayable(&m_coop_hop_validate_work);
    quarantine_add(&m_quarantine, m_coop_hop_revert_to,
                   CONFIG_DONGLE_COOP_HOP_QUARANTINE_MS);
    LOG_INF("coop hop validated on %u, quarantined %u for %u ms",
            esb_prx_get_channel(), m_coop_hop_revert_to,
            (unsigned)CONFIG_DONGLE_COOP_HOP_QUARANTINE_MS);
    m_coop_hop_revert_to = CHANNEL_HOP_INVALID;
}

void channel_hop_dongle_note_rx_active(void) {
    /* Any non-IDLE packet: peer is currently transmitting, so we are not
     * idle. If we were speculating, rolling back, or coop-validating,
     * this confirms the current channel is where the peer lives.
     *
     * Note: do NOT use this hook to disarm a pending coop hop on the
     * pre-hop channel. The endpoint keeps transmitting user HID on the
     * pre-hop channel during the HOP_IN_MS commit window by design (it
     * does not flip its radio until its own commit timer fires), so a
     * "non-OFFER packet during commit window = endpoint aborted" rule
     * would cancel essentially every coop hop whenever the user is
     * moving the trackball. The coop confirm hook below is safe because
     * it runs AFTER the radio has retuned: any RX it acts on was
     * received on the new channel and unambiguously means the endpoint
     * joined us. */
    if (m_peer_idle) {
        LOG_INF("peer active again on %u", esb_prx_get_channel());
    }
    m_peer_idle = false;

    confirm_coop_hop_if_any();
    confirm_speculative_if_any();
    exit_rollback();
    led_status_set_link_lost(false);
    peek_on_peer_active();
    rearm_silence_if_needed();
}

void channel_hop_dongle_note_rx_idle(void) {
    /* Peer explicitly declared idle. Cancel watchdogs until real
     * traffic resumes. Also confirms any speculative hop / rollback /
     * coop hop — receiving IDLE on the new channel means the peer is
     * there. */
    if (!m_peer_idle) {
        LOG_INF("peer went idle on %u", esb_prx_get_channel());
    }
    m_peer_idle = true;
    confirm_coop_hop_if_any();
    confirm_speculative_if_any();
    exit_rollback();
    led_status_set_link_lost(false);
    k_work_cancel_delayable(&m_silence_work);
    k_work_cancel_delayable(&m_rollback_silence_work);
    rearm_peek_if_needed();
}

void channel_hop_dongle_on_rx_proposal(const uint8_t *data, uint8_t len) {
    if (len < sizeof(struct esb_pkt_channel_hop_proposal)) {
        return;
    }
    const struct esb_pkt_channel_hop_proposal *p = (const void *)data;
    if (p->proposed >= CHANNEL_HOP_CHANNEL_COUNT) {
        return;
    }

    const uint8_t current = esb_prx_get_channel();
    uint8_t agreed;
    bool accepted;

    if (!quarantine_is(&m_quarantine, p->proposed) && p->proposed != current) {
        agreed   = p->proposed;
        accepted = true;
    } else {
        const uint8_t counter = channel_hop_pick(
            &m_quarantine,
            CONFIG_DONGLE_CHANNEL_QUARANTINE_MIN_DISTANCE,
            current);
        if (counter == CHANNEL_HOP_INVALID) {
            agreed   = p->proposed;
            accepted = true;
        } else {
            agreed   = counter;
            accepted = false;
        }
    }

    m_committed_next = agreed;

    struct esb_pkt_channel_hop_confirm cfm = {
        .type     = ESB_PKT_CHANNEL_HOP_CONFIRM,
        .agreed   = agreed,
        .accepted = accepted ? 1 : 0,
    };
    const int err = esb_prx_queue_ack(ESB_PIPE_DATA, (uint8_t *)&cfm, sizeof(cfm));
    if (err) {
        LOG_WRN("CONFIRM queue failed: %d", err);
    } else {
        LOG_INF("PROPOSAL rx (proposed=%u current_ep=%u) → committed_next=%u (%s)",
                p->proposed, p->current, agreed,
                accepted ? "accept" : "counter");
    }
}

void channel_hop_dongle_after_rx(uint8_t pipe) {
    if (!m_paired || m_peer_idle) {
        return;
    }
    if (pipe != ESB_PIPE_DATA) {
        /* REQUEST is a pipe-1 ACK payload; queueing on pipe 0 would never
         * reach the endpoint (it only TXs HID/etc on pipe 1 in steady
         * state). */
        return;
    }
    if (m_committed_next != CHANNEL_HOP_INVALID) {
        return;
    }
    if (m_speculative_prev != CHANNEL_HOP_INVALID) {
        /* Mid-validation: committed_next was just consumed for the hop
         * we are validating. Don't ask for a refresh until we know
         * whether this hop sticks. */
        return;
    }
    const struct esb_pkt_channel_hop_request req = {
        .type = ESB_PKT_CHANNEL_HOP_REQUEST,
    };
    const int err = esb_prx_queue_ack(ESB_PIPE_DATA,
                                      (uint8_t *)&req, sizeof(req));
    if (err) {
        LOG_WRN("REQUEST queue failed: %d", err);
    } else {
        LOG_INF("queued REQUEST in ACK (committed_next is INVALID, on %u)",
                esb_prx_get_channel());
    }
}

uint8_t channel_hop_dongle_get_committed(void) {
    return m_committed_next;
}

uint8_t channel_hop_dongle_get_quarantine_count(void) {
    return quarantine_count(&m_quarantine);
}

bool channel_hop_dongle_is_quarantined(uint8_t channel) {
    return quarantine_is(&m_quarantine, channel);
}

bool channel_hop_dongle_is_peer_idle(void) {
    return m_peer_idle;
}

#else /* !CONFIG_DONGLE_CHANNEL_HOP */

void channel_hop_dongle_init(uint8_t initial_channel) { ARG_UNUSED(initial_channel); }
void channel_hop_dongle_set_paired(bool paired) { ARG_UNUSED(paired); }
void channel_hop_dongle_note_rx_active(void) {}
void channel_hop_dongle_note_rx_idle(void) {}
void channel_hop_dongle_on_rx_proposal(const uint8_t *data, uint8_t len) { ARG_UNUSED(data); ARG_UNUSED(len); }
void channel_hop_dongle_on_rx_offer(const uint8_t *data, uint8_t len) { ARG_UNUSED(data); ARG_UNUSED(len); }
void channel_hop_dongle_after_rx(uint8_t pipe) { ARG_UNUSED(pipe); }
uint8_t channel_hop_dongle_get_committed(void) { return CHANNEL_HOP_INVALID; }
uint8_t channel_hop_dongle_get_quarantine_count(void) { return 0; }
bool channel_hop_dongle_is_quarantined(uint8_t channel) { ARG_UNUSED(channel); return false; }
bool channel_hop_dongle_is_peer_idle(void) { return true; }

#endif /* CONFIG_DONGLE_CHANNEL_HOP */
