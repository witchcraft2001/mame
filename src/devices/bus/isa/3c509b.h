// license:BSD-3-Clause
// copyright-holders:Dmitry
/***************************************************************************

    ISA8 3Com 3C509B-TPO EtherLink III Ethernet adapter

    Register-level model of the 3C509B "Parallel Tasking" ASIC, including
    the legacy ID-port activation sequence used to configure the card
    before the ISA I/O window is enabled.  Unlike the non-B 3C509, the B
    revision explicitly supports 8-bit-only ISA slots: every 16-bit
    register is accessed as two consecutive 8-bit cycles (low byte first,
    then high byte), which is the only access mode this device implements.

    See MAME_3C509B_ISA8.md at the repository root for the register/window
    map, the ID-port handshake, and the CIP/timing model this file follows.

***************************************************************************/

#ifndef MAME_BUS_ISA_3C509B_H
#define MAME_BUS_ISA_3C509B_H

#pragma once

#include "isa.h"
#include "dinetwork.h"


class isa8_3c509b_device :
	public device_t,
	public device_isa8_card_interface,
	public device_network_interface
{
public:
	isa8_3c509b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual int recv_start_cb(uint8_t *buf, int length) override;
	virtual void recv_complete_cb(int result) override;
	virtual void send_complete_cb(int result) override;

private:
	// ID port front-end state: IDLE (nothing seen) -> ARMED (one zero byte
	// seen) -> SEQ (second zero byte primed the LFSR, comparing the
	// 255-byte sequence) -> CMD (sequence matched, accepting ID commands).
	enum id_state : uint8_t { ID_IDLE, ID_ARMED, ID_SEQ, ID_CMD };

	// deferred (CIP-gated) operations, executed by m_cmd_timer
	enum pending_op : uint8_t
	{
		OP_NONE,
		OP_GLOBAL_RESET,
		OP_EEPROM_READ,
		OP_EEPROM_WRITE,
		OP_EEPROM_ERASE,
		OP_EEPROM_ERASE_ALL,
		OP_EEPROM_EWEN,
		OP_EEPROM_EWDS,
		OP_RX_DISCARD,
		OP_RX_RESET,
		OP_TX_RESET,
		OP_GENERIC,
	};

	// The analogue 10BASE-T path recovers independently of the digital
	// AUTOINIT/CIP command engine after a global reset.
	enum link_state : uint8_t { LINK_DOWN, LINK_RECOVERING, LINK_READY };

	// RX descriptor ring depth.  The 3C509B does not have the eight-packet
	// RX Status limit of the original 3C509 (tech reference 6.31, FIFO
	// Diagnostic bit 12 is reserved on the B revision), so the binding
	// constraint is the byte capacity of the RX partition; this only caps
	// the number of concurrently staged descriptors.
	static constexpr unsigned RX_SLOTS = 256;

	// Longest frame the receiver will take off the wire before cutting the
	// remainder (tech reference 6.17: oversize frames keep being received
	// until 1,792 bytes).
	static constexpr unsigned RX_MAX_BYTES = 1792;

	static constexpr unsigned TX_STATUS_DEPTH = 31;

	// 32 KiB packet SRAM, split 1:1 between the RX and TX partitions (the
	// "practical default" noted in MAME_3C509B_ISA8.md 6.2).  Both
	// partitions are modeled as byte rings, so the Free-byte registers and
	// the overrun conditions follow the real byte accounting.
	static constexpr unsigned SRAM_PARTITION_BYTES = 16384;

	// 10 MHz reference for the Window 1 Timer, which counts one tick per
	// 32 clocks (3.2us, the quadbyte rate).
	static constexpr uint32_t TIMER_CLOCK = 10'000'000;

	TIMER_CALLBACK_MEMBER(cmd_timer_done);
	TIMER_CALLBACK_MEMBER(txkick_timer_done);
	TIMER_CALLBACK_MEMBER(txdrop_timer_done);
	TIMER_CALLBACK_MEMBER(link_timer_done);

	// ID port (legacy activation) handlers
	uint8_t id_r(offs_t offset);
	void id_w(offs_t offset, uint8_t data);
	void id_command(uint8_t cmd);
	static uint8_t id_lfsr_next(uint8_t v);

	// operating window handlers
	uint8_t op_r(offs_t offset);
	void op_w(offs_t offset, uint8_t data);
	uint16_t reg_read(uint8_t reg);
	void reg_write(uint8_t reg, uint16_t data);
	void do_command(uint16_t cmd);
	void eeprom_command(uint16_t cmd);
	void start_cip(pending_op op, uint16_t arg, attotime const &delay);

	uint16_t tx_used_bytes() const;
	uint16_t rx_used_bytes() const;
	bool rx_fifo_full() const;
	void bump_stat(uint8_t index);
	void bump_stat_word(uint16_t &counter, uint16_t amount);
	void stats_full_recheck();
	void update_tx_available();

	uint8_t timer_value() const;
	void timer_restart();

	void invalidate_latches();
	uint8_t fifo_read_byte();
	void fifo_write_byte(uint8_t data);
	void tx_kick();
	void push_tx_status(uint8_t status);
	void pop_tx_status();
	void rx_discard();
	void rx_reset();
	void tx_reset();
	void adapter_failure(uint16_t diag_bit);
	bool accept_filter(uint8_t const *buf, int length) const;

	static uint8_t decode_irq(uint8_t config);
	static uint8_t decode_iobase_index(uint8_t config);
	static uint8_t pnp_serial_checksum(uint16_t const *eeprom);

	void build_eeprom();
	void reset_link_recovery();
	void maybe_start_link_recovery();
	bool tpo_selected() const;
	void activate();
	void deactivate();
	void global_reset();
	void global_reset_finish();

	uint16_t masked_status() const;
	void update_irq();
	void irq_out(int state);

	required_ioport m_config;
	required_ioport m_eeprom_profile;
	required_ioport m_link_recovery;
	emu_timer *m_cmd_timer;
	emu_timer *m_txkick_timer; // interframe gap between queued TX frames
	emu_timer *m_txdrop_timer; // wire-time completion for a frame lost while link is down
	emu_timer *m_link_timer;

	// EEPROM (word-addressed, 0x00-0x3f)
	uint16_t m_eeprom[0x40];
	uint16_t m_eeprom_data; // shared by Window 0 and the ID-port contention shifter
	bool m_eeprom_busy;
	bool m_eeprom_wren;

	// ID port / legacy activation state
	int8_t m_id_port;
	uint8_t m_id_state; // id_state enum value, stored as uint8_t for save_item
	uint8_t m_id_lfsr;
	uint16_t m_id_pos;
	uint8_t m_id_tag;

	// A0 byte latches (independent for read and write direction)
	bool m_rd_latch_valid;
	uint8_t m_rd_latch_off;
	uint8_t m_rd_latch_hi;
	bool m_wr_latch_valid;
	uint8_t m_wr_latch_off;
	uint8_t m_wr_latch_lo;

	// pending CIP operation
	uint8_t m_pending_op; // pending_op enum value
	uint16_t m_pending_arg;

	// window 0: setup
	uint8_t m_window;
	uint16_t m_config_control;
	uint16_t m_addr_config;
	uint16_t m_resource_config;
	bool m_activated;
	uint16_t m_iobase;
	uint8_t m_irq;
	bool m_irq_line_state;
	uint8_t m_link_state; // link_state enum value, stored as uint8_t for save_item

	// window 1/3/5: operating state
	uint16_t m_status;
	uint16_t m_int_mask;
	uint16_t m_read_zero_mask;
	uint8_t m_rx_filter;
	uint16_t m_rx_early_thresh;
	uint16_t m_tx_avail_thresh;
	uint16_t m_tx_start_thresh;
	bool m_rx_enabled;
	bool m_tx_enabled;
	uint32_t m_internal_config;

	// Window 1 Timer: free-running 3.2us counter, saturating at 0xff.  Only
	// the reference timestamp (in TIMER_CLOCK ticks) is stored; the count is
	// derived on read.
	uint64_t m_timer_base;

	// TX: frames are PIO-assembled one at a time in m_tx_buf, but the TX
	// partition holds multiple completed frames, so a driver may keep
	// writing while an earlier frame is still on the wire.  Completed
	// entries (preamble + DWORD-padded data, exactly as written) are queued
	// in the m_txq byte ring and transmitted in order, one per
	// device_network_interface send(), separated by the 10 Mbps
	// interframe gap; completion is asynchronous via the bandwidth-timed
	// send_complete_cb().
	uint8_t m_tx_buf[4 + 2048];
	uint16_t m_tx_buf_len;
	uint16_t m_tx_len;
	uint16_t m_tx_expected;
	uint8_t m_tx_error; // TX Status error bits latched while assembling
	uint8_t m_txq[SRAM_PARTITION_BYTES];
	uint16_t m_txq_head; // ring offset of the oldest queued entry
	uint16_t m_txq_used; // bytes occupied by completed queued entries
	bool m_tx_pending; // true while the head entry awaits send_complete_cb
	uint16_t m_tx_pending_len;
	bool m_tx_pending_notify;
	bool m_tx_reset_required; // jabber/underrun latch: TxEnable alone cannot recover
	bool m_tx_status_overflow;

	// TX status stack: read (base+0x0B) peeks the head entry without
	// removing it; a write to base+0x0B pops exactly one entry.
	uint8_t m_tx_status_stack[TX_STATUS_DEPTH];
	uint8_t m_tx_status_count;

	// RX: byte-addressed ring over the RX partition plus a descriptor ring
	// holding one length/status pair per staged packet.  A packet is only
	// removed from the head by an explicit RX_DISCARD command; reading its
	// data via the FIFO never advances the ring.
	uint8_t m_rxq[SRAM_PARTITION_BYTES];
	uint16_t m_rxq_head; // ring offset of the head packet's first byte
	uint16_t m_rxq_used; // DWORD-padded bytes occupied by staged packets
	uint16_t m_rx_len[RX_SLOTS];
	uint16_t m_rx_status[RX_SLOTS];
	uint16_t m_rx_head;
	uint16_t m_rx_count;
	uint16_t m_rx_cursor;
	bool m_rx_receiving; // a packet is between recv_start_cb and recv_complete_cb

	// staged by recv_start_cb, committed to the ring by recv_complete_cb
	// once the simulated transfer time has elapsed
	uint8_t m_rx_stage_data[RX_MAX_BYTES];
	uint16_t m_rx_stage_len;
	uint16_t m_rx_stage_status;

	// window 2: station address (Address0..Address5, ISA byte order)
	uint8_t m_station[6];

	// window 4: diagnostics
	uint16_t m_fifo_diag; // sticky RX Underrun / TX Overrun bits only
	uint16_t m_net_diag; // loopback select bits only; the rest is derived
	uint16_t m_eth_status;
	uint16_t m_media_status; // host-writable bits only; the rest is derived

	// window 6: statistics
	uint8_t m_stat_byte[9];
	uint16_t m_stat_tx_bytes;
	uint16_t m_stat_rx_bytes;
	bool m_stats_enabled;
};

DECLARE_DEVICE_TYPE(ISA8_3C509B, isa8_3c509b_device)

#endif // MAME_BUS_ISA_3C509B_H
