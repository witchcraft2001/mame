// license:BSD-3-Clause
// copyright-holders:Dmitry Mikhalchenkov
/***************************************************************************

    3Com 3C509B-TPO EtherLink III ISA Ethernet adapter

***************************************************************************/

#ifndef MAME_BUS_ISA_3C509B_H
#define MAME_BUS_ISA_3C509B_H

#pragma once

#include "isa.h"
#include "dinetwork.h"

#include <array>


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
	virtual void device_post_load() override;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual int recv_start_cb(uint8_t *buf, int length) override;
	virtual void recv_complete_cb(int result) override;
	virtual void send_complete_cb(int result) override;

private:
	enum id_state : uint8_t { ID_IDLE, ID_ARMED, ID_SEQ, ID_CMD };

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

	static constexpr unsigned RX_SLOTS = 256;
	static constexpr unsigned MAX_PACKET_BYTES = 1792;
	static constexpr unsigned TX_STATUS_DEPTH = 31;
	static constexpr unsigned SRAM_PARTITION_BYTES = 16384;
	static constexpr uint32_t TIMER_CLOCK = 10'000'000;

	TIMER_CALLBACK_MEMBER(cmd_timer_done);
	TIMER_CALLBACK_MEMBER(txkick_timer_done);

	uint8_t id_r(offs_t offset);
	void id_w(offs_t offset, uint8_t data);
	void id_command(uint8_t cmd);
	static uint8_t id_lfsr_next(uint8_t v);

	uint8_t op_r(offs_t offset);
	void op_w(offs_t offset, uint8_t data);
	uint16_t reg_read(uint8_t reg);
	void reg_write(uint8_t reg, uint16_t data);
	void do_command(uint16_t cmd);
	void eeprom_command(uint16_t cmd);
	void start_cip(pending_op op, uint16_t arg, attotime const &delay);

	uint16_t tx_used_bytes() const;
	bool rx_fifo_full() const;
	void bump_stat(uint8_t index);
	void bump_stat_word(uint16_t &counter, uint16_t amount);
	void stats_full_recheck();
	void update_tx_available();
	uint16_t read_stat_word(uint16_t &counter);

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
	bool accept_filter(uint8_t const *buf) const;

	static uint8_t pnp_serial_checksum(std::array<uint16_t, 0x40> const &eeprom);
	bool eeprom_busy() const;

	void build_eeprom();
	bool tpo_selected() const;
	void activate();
	void deactivate();
	void install_io(uint16_t base);
	void global_reset();
	void global_reset_finish();

	uint16_t masked_status() const;
	void update_irq();
	void irq_out(int state);

	required_ioport m_config;
	emu_timer *m_cmd_timer;
	emu_timer *m_txkick_timer;

	std::array<uint16_t, 0x40> m_eeprom;
	uint16_t m_eeprom_data;
	bool m_eeprom_wren;
	bool m_eeprom_initialized;

	uint8_t m_id_state;
	uint8_t m_id_lfsr;
	uint16_t m_id_pos;
	uint8_t m_id_tag;

	bool m_rd_latch_valid;
	uint8_t m_rd_latch_off;
	uint8_t m_rd_latch_hi;
	bool m_wr_latch_valid;
	uint8_t m_wr_latch_off;
	uint8_t m_wr_latch_lo;

	uint8_t m_pending_op;
	uint16_t m_pending_arg;

	uint8_t m_window;
	uint16_t m_config_control;
	uint16_t m_addr_config;
	uint16_t m_resource_config;
	bool m_activated;
	uint16_t m_iobase;
	uint16_t m_installed_iobase;
	uint8_t m_irq;
	bool m_irq_line_state;

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

	// Reference time for the 3.2 us Window 1 timer.
	uint64_t m_timer_base;

	// Transmit assembly buffer and completed-frame ring.
	uint8_t m_tx_buf[4 + 2048];
	uint16_t m_tx_buf_len;
	uint16_t m_tx_len;
	uint16_t m_tx_expected;
	uint8_t m_tx_error;
	uint8_t m_txq[SRAM_PARTITION_BYTES];
	uint16_t m_txq_head;
	uint16_t m_txq_used;
	bool m_tx_pending;
	uint16_t m_tx_pending_len;
	bool m_tx_pending_notify;
	bool m_tx_reset_required;
	bool m_tx_status_overflow;

	uint8_t m_tx_status_stack[TX_STATUS_DEPTH];
	uint8_t m_tx_status_count;

	// Receive data ring and packet descriptors.
	uint8_t m_rxq[SRAM_PARTITION_BYTES];
	uint16_t m_rxq_head;
	uint16_t m_rxq_used;
	uint16_t m_rx_len[RX_SLOTS];
	uint16_t m_rx_status[RX_SLOTS];
	uint16_t m_rx_head;
	uint16_t m_rx_count;
	uint16_t m_rx_cursor;
	bool m_rx_receiving;

	uint8_t m_rx_stage_data[MAX_PACKET_BYTES];
	uint16_t m_rx_stage_len;
	uint16_t m_rx_stage_status;

	uint8_t m_station[6];

	uint16_t m_fifo_diag;
	uint16_t m_net_diag;
	uint16_t m_media_status;

	uint8_t m_stat_byte[9];
	uint16_t m_stat_tx_bytes;
	uint16_t m_stat_rx_bytes;
	bool m_stats_enabled;
};

DECLARE_DEVICE_TYPE(ISA8_3C509B, isa8_3c509b_device)

#endif // MAME_BUS_ISA_3C509B_H
