// license:BSD-3-Clause
// copyright-holders:Dmitry
/***************************************************************************

    ISA8 3Com 3C509B-TPO EtherLink III Ethernet adapter

    Register reference: 3Com "EtherLink III Parallel Tasking ISA, EISA,
    Micro Channel, and PCMCIA Adapter Drivers Technical Reference",
    part no. 09-0398-002B (Aug 1994). See MAME_3C509B_ISA8.md at the
    repository root for the derived register/window map, the ID-port
    handshake, and the CIP/timing model this file implements; clean-room
    behavioral cross-checks against the Linux 3c509 driver (GPL, consulted
    for observable sequencing only, no code copied) are noted inline where
    relevant. Structural inspiration (register window dispatch style) from
    the abandoned 2004 QEMU 3c509b.c patch by Antony T Curtis (MIT
    licensed); the legacy ID-port activation sequence is a fresh
    implementation, since the QEMU patch relied on QEMU's own ISA-PnP
    framework instead.

    The 3C509B is the first EtherLink III revision to officially support
    8-bit-only ISA slots: every 16-bit register is accessed as a pair of
    8-bit cycles (low byte, then high byte); the 16-bit operation itself
    is latched in and executed on the write of the *high* byte.  Reads
    behave symmetrically.  A handful of registers (the RX/TX packet FIFO
    at +0x00/+0x02 only, Timer, TX Status, ROM Control, and the Window 6
    byte statistics) are plain byte-wide registers and bypass this latch
    entirely.

***************************************************************************/

#include "emu.h"
#include "3c509b.h"

#include "multibyte.h"

#include <algorithm>


#define LOG_ID    (1U << 1)
#define LOG_IO    (1U << 2)
#define LOG_CMD   (1U << 3)
#define LOG_NET   (1U << 4)
#define LOG_IRQ   (1U << 5)

#define VERBOSE (LOG_GENERAL)

#include "logmacro.h"

#define LOGID(...)  LOGMASKED(LOG_ID, __VA_ARGS__)
#define LOGIO(...)  LOGMASKED(LOG_IO, __VA_ARGS__)
#define LOGCMD(...) LOGMASKED(LOG_CMD, __VA_ARGS__)
#define LOGNET(...) LOGMASKED(LOG_NET, __VA_ARGS__)
#define LOGIRQ(...) LOGMASKED(LOG_IRQ, __VA_ARGS__)


namespace {

// Status register bits (window-independent, offset 0x0e)
constexpr uint16_t ST_INT_LATCH        = 0x0001;
constexpr uint16_t ST_ADAPTER_FAILURE  = 0x0002;
constexpr uint16_t ST_TX_COMPLETE      = 0x0004;
constexpr uint16_t ST_TX_AVAILABLE     = 0x0008;
constexpr uint16_t ST_RX_COMPLETE      = 0x0010;
constexpr uint16_t ST_RX_EARLY         = 0x0020;
constexpr uint16_t ST_INT_REQUESTED    = 0x0040;
constexpr uint16_t ST_STATS_FULL       = 0x0080;
constexpr uint16_t ST_CMD_IN_PROGRESS  = 0x1000;
constexpr uint16_t ST_ACK_MASK         = 0x0069; // acknowledgeable via AckIntr

// RX filter bits (Window 5, offset 0x08)
constexpr uint8_t RXF_INDIVIDUAL = 0x01;
constexpr uint8_t RXF_GROUP      = 0x02;
constexpr uint8_t RXF_BROADCAST  = 0x04;
constexpr uint8_t RXF_PROMISC    = 0x08;

// Command register opcodes (cmd >> 11)
enum : uint8_t
{
	CMD_GLOBAL_RESET   = 0,
	CMD_SELECT_WINDOW  = 1,
	CMD_START_COAX     = 2,
	CMD_RX_DISABLE     = 3,
	CMD_RX_ENABLE      = 4,
	CMD_RX_RESET       = 5,
	CMD_RX_DISCARD     = 8,
	CMD_TX_ENABLE      = 9,
	CMD_TX_DISABLE     = 10,
	CMD_TX_RESET       = 11,
	CMD_FAKE_INTR      = 12,
	CMD_ACK_INTR       = 13,
	CMD_SET_INTR_ENB   = 14,
	CMD_SET_STATUS_ENB = 15,
	CMD_SET_RX_FILTER  = 16,
	CMD_SET_RX_THRESH  = 17,
	CMD_SET_TX_THRESH  = 18,
	CMD_SET_TX_START   = 19,
	CMD_STATS_ENABLE   = 21,
	CMD_STATS_DISABLE  = 22,
	CMD_STOP_COAX      = 23,
	CMD_POWER_UP       = 27,
	CMD_POWER_DOWN     = 28,
	CMD_POWER_AUTO     = 29,
};

} // anonymous namespace


DEFINE_DEVICE_TYPE(ISA8_3C509B, isa8_3c509b_device, "3c509b", "3Com 3C509B-TPO EtherLink III ISA8 Ethernet Adapter")


isa8_3c509b_device::isa8_3c509b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISA8_3C509B, tag, owner, clock),
	device_isa8_card_interface(mconfig, *this),
	device_network_interface(mconfig, *this, 10), // 10 Mib/s
	m_config(*this, "CONFIG"),
	m_cmd_timer(nullptr),
	m_txkick_timer(nullptr),
	m_eeprom{ },
	m_eeprom_data(0),
	m_eeprom_busy(false),
	m_eeprom_wren(false),
	m_id_port(-1),
	m_id_state(ID_IDLE),
	m_id_lfsr(0xff),
	m_id_pos(0),
	m_id_tag(0),
	m_id_eeprom_data(0),
	m_rd_latch_valid(false),
	m_rd_latch_off(0),
	m_rd_latch_hi(0),
	m_wr_latch_valid(false),
	m_wr_latch_off(0),
	m_wr_latch_lo(0),
	m_pending_op(OP_NONE),
	m_pending_arg(0),
	m_window(0),
	m_config_control(0),
	m_addr_config(0),
	m_resource_config(0),
	m_activated(false),
	m_iobase(0x0300),
	m_irq(3),
	m_irq_line_state(false),
	m_status(0),
	m_int_mask(0),
	m_read_zero_mask(0),
	m_rx_filter(0),
	m_rx_early_thresh(0),
	m_tx_avail_thresh(0),
	m_tx_start_thresh(0),
	m_rx_enabled(false),
	m_tx_enabled(false),
	m_internal_config(0),
	m_tx_buf{ },
	m_tx_buf_len(0),
	m_tx_len(0),
	m_tx_expected(0),
	m_txq{ },
	m_txq_head(0),
	m_txq_used(0),
	m_tx_pending(false),
	m_tx_pending_len(0),
	m_tx_pending_notify(false),
	m_tx_status_stack{ },
	m_tx_status_count(0),
	m_rx_data{ },
	m_rx_len{ },
	m_rx_status{ },
	m_rx_head(0),
	m_rx_count(0),
	m_rx_cursor(0),
	m_rx_stage_data{ },
	m_rx_stage_len(0),
	m_rx_stage_status(0),
	m_station{ },
	m_fifo_diag(0),
	m_net_diag(0),
	m_eth_status(0),
	m_media_status(0),
	m_stat_byte{ },
	m_stat_tx_bytes(0),
	m_stat_rx_bytes(0),
	m_stats_enabled(false)
{
}


void isa8_3c509b_device::device_start()
{
	set_isa_device();

	// Legacy ID port: only addresses of the form 0x1xy0 (low nibble 0)
	// decode; the superset range is installed once and gated at runtime
	// the same way the operating window below is.
	m_isa->install_device(0x0100, 0x01ff,
			read8sm_delegate(*this, FUNC(isa8_3c509b_device::id_r)),
			write8sm_delegate(*this, FUNC(isa8_3c509b_device::id_w)));

	// Operating window: 16-byte register block relocatable to any of the
	// 31 bases 0x0200-0x03e0 (step 0x10); the superset covers every
	// possible base and op_r/op_w gate on the currently configured
	// m_iobase.
	m_isa->install_device(0x0200, 0x03ff,
			read8sm_delegate(*this, FUNC(isa8_3c509b_device::op_r)),
			write8sm_delegate(*this, FUNC(isa8_3c509b_device::op_w)));

	m_cmd_timer = timer_alloc(FUNC(isa8_3c509b_device::cmd_timer_done), this);
	m_txkick_timer = timer_alloc(FUNC(isa8_3c509b_device::txkick_timer_done), this);

	save_item(NAME(m_eeprom));
	save_item(NAME(m_eeprom_data));
	save_item(NAME(m_eeprom_busy));
	save_item(NAME(m_eeprom_wren));
	save_item(NAME(m_id_port));
	save_item(NAME(m_id_state));
	save_item(NAME(m_id_lfsr));
	save_item(NAME(m_id_pos));
	save_item(NAME(m_id_tag));
	save_item(NAME(m_id_eeprom_data));
	save_item(NAME(m_rd_latch_valid));
	save_item(NAME(m_rd_latch_off));
	save_item(NAME(m_rd_latch_hi));
	save_item(NAME(m_wr_latch_valid));
	save_item(NAME(m_wr_latch_off));
	save_item(NAME(m_wr_latch_lo));
	save_item(NAME(m_pending_op));
	save_item(NAME(m_pending_arg));
	save_item(NAME(m_window));
	save_item(NAME(m_config_control));
	save_item(NAME(m_addr_config));
	save_item(NAME(m_resource_config));
	save_item(NAME(m_activated));
	save_item(NAME(m_iobase));
	save_item(NAME(m_irq));
	save_item(NAME(m_irq_line_state));
	save_item(NAME(m_status));
	save_item(NAME(m_int_mask));
	save_item(NAME(m_read_zero_mask));
	save_item(NAME(m_rx_filter));
	save_item(NAME(m_rx_early_thresh));
	save_item(NAME(m_tx_avail_thresh));
	save_item(NAME(m_tx_start_thresh));
	save_item(NAME(m_rx_enabled));
	save_item(NAME(m_tx_enabled));
	save_item(NAME(m_internal_config));
	save_item(NAME(m_tx_buf));
	save_item(NAME(m_tx_buf_len));
	save_item(NAME(m_tx_len));
	save_item(NAME(m_tx_expected));
	save_item(NAME(m_txq));
	save_item(NAME(m_txq_head));
	save_item(NAME(m_txq_used));
	save_item(NAME(m_tx_pending));
	save_item(NAME(m_tx_pending_len));
	save_item(NAME(m_tx_pending_notify));
	save_item(NAME(m_tx_status_stack));
	save_item(NAME(m_tx_status_count));
	save_item(NAME(m_rx_data));
	save_item(NAME(m_rx_len));
	save_item(NAME(m_rx_status));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_rx_cursor));
	save_item(NAME(m_rx_stage_data));
	save_item(NAME(m_rx_stage_len));
	save_item(NAME(m_rx_stage_status));
	save_item(NAME(m_station));
	save_item(NAME(m_fifo_diag));
	save_item(NAME(m_net_diag));
	save_item(NAME(m_eth_status));
	save_item(NAME(m_media_status));
	save_item(NAME(m_stat_byte));
	save_item(NAME(m_stat_tx_bytes));
	save_item(NAME(m_stat_rx_bytes));
	save_item(NAME(m_stats_enabled));
}


void isa8_3c509b_device::device_reset()
{
	m_cmd_timer->reset();
	m_pending_op = OP_NONE;
	m_eeprom_busy = false;

	build_eeprom();
	global_reset_finish();

	osd_printf_verbose("3c509b: power-on reset mac=%02x:%02x:%02x:%02x:%02x:%02x auto_activate=%s\n",
			get_mac()[0], get_mac()[1], get_mac()[2], get_mac()[3], get_mac()[4], get_mac()[5],
			BIT(m_config->read(), 7) ? "yes" : "no");
}


static INPUT_PORTS_START(3c509b)
	PORT_START("CONFIG")
	PORT_CONFNAME(0x1f, 0x10, "3C509B I/O base")
	PORT_CONFSETTING(0x00, "0x200")
	PORT_CONFSETTING(0x01, "0x210")
	PORT_CONFSETTING(0x02, "0x220")
	PORT_CONFSETTING(0x03, "0x230")
	PORT_CONFSETTING(0x04, "0x240")
	PORT_CONFSETTING(0x05, "0x250")
	PORT_CONFSETTING(0x06, "0x260")
	PORT_CONFSETTING(0x07, "0x270")
	PORT_CONFSETTING(0x08, "0x280")
	PORT_CONFSETTING(0x09, "0x290")
	PORT_CONFSETTING(0x0a, "0x2A0")
	PORT_CONFSETTING(0x0b, "0x2B0")
	PORT_CONFSETTING(0x0c, "0x2C0")
	PORT_CONFSETTING(0x0d, "0x2D0")
	PORT_CONFSETTING(0x0e, "0x2E0")
	PORT_CONFSETTING(0x0f, "0x2F0")
	PORT_CONFSETTING(0x10, "0x300")
	PORT_CONFSETTING(0x11, "0x310")
	PORT_CONFSETTING(0x12, "0x320")
	PORT_CONFSETTING(0x13, "0x330")
	PORT_CONFSETTING(0x14, "0x340")
	PORT_CONFSETTING(0x15, "0x350")
	PORT_CONFSETTING(0x16, "0x360")
	PORT_CONFSETTING(0x17, "0x370")
	PORT_CONFSETTING(0x18, "0x380")
	PORT_CONFSETTING(0x19, "0x390")
	PORT_CONFSETTING(0x1a, "0x3A0")
	PORT_CONFSETTING(0x1b, "0x3B0")
	PORT_CONFSETTING(0x1c, "0x3C0")
	PORT_CONFSETTING(0x1d, "0x3D0")
	PORT_CONFSETTING(0x1e, "0x3E0")
	PORT_CONFNAME(0x60, 0x00, "3C509B IRQ")
	PORT_CONFSETTING(0x00, "IRQ3")
	PORT_CONFSETTING(0x20, "IRQ5")
	PORT_CONFSETTING(0x40, "IRQ7")
	PORT_CONFSETTING(0x60, "IRQ2/9")
	PORT_CONFNAME(0x80, 0x00, "3C509B Activation")
	PORT_CONFSETTING(0x00, "Real (ID sequence required)")
	PORT_CONFSETTING(0x80, "Auto-activate at reset")
INPUT_PORTS_END


ioport_constructor isa8_3c509b_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(3c509b);
}


// ---------------------------------------------------------------------
// EEPROM construction
// ---------------------------------------------------------------------

uint8_t isa8_3c509b_device::decode_irq(uint8_t config)
{
	switch ((config >> 5) & 0x03)
	{
	case 0: return 3;
	case 1: return 5;
	case 2: return 7;
	default: return 9;
	}
}


uint8_t isa8_3c509b_device::decode_iobase_index(uint8_t config)
{
	return config & 0x1f;
}


void isa8_3c509b_device::build_eeprom()
{
	std::fill(std::begin(m_eeprom), std::end(m_eeprom), uint16_t(0x0000));

	uint32_t mac_tail = 0x509b3c;
	for (char const *p = tag(); *p != '\0'; p++)
		mac_tail = (mac_tail * 33) ^ uint8_t(*p);

	uint8_t mac[6] = { 0x02, 0x60, 0x8c, 0x00, 0x00, 0x00 };
	put_u24be(&mac[3], mac_tail);
	set_mac(mac);

	// Word low byte = Address(2n), high byte = Address(2n+1), matching the
	// Window 2 station-address byte order (see reg_read/reg_write, window 2).
	m_eeprom[0x00] = mac[0] | (uint16_t(mac[1]) << 8);
	m_eeprom[0x01] = mac[2] | (uint16_t(mac[3]) << 8);
	m_eeprom[0x02] = mac[4] | (uint16_t(mac[5]) << 8);

	m_eeprom[0x03] = 0x9550; // Product ID: 3C509B-TPO
	m_eeprom[0x07] = 0x6d50; // 3Com manufacturer ID

	const uint8_t config = m_config->read();
	m_eeprom[0x08] = decode_iobase_index(config); // XCVR select = 00 (TPO), bits4:0 = base index
	m_eeprom[0x09] = uint16_t(decode_irq(config)) << 12; // informational only on Sprinter

	m_eeprom[0x0a] = m_eeprom[0x00]; // OEM node address (mirrors the factory MAC)
	m_eeprom[0x0b] = m_eeprom[0x01];
	m_eeprom[0x0c] = m_eeprom[0x02];

	m_eeprom[0x0d] = 0x0001; // software info: link beat policy enabled
	m_eeprom[0x0e] = 0x0000; // compatibility word

	m_eeprom[0x10] = 0x2083; // capabilities word
	m_eeprom[0x12] = 0x0002; // internal config low: RAM SIZE = 010b (32 KiB)
	m_eeprom[0x13] = 0x0001; // internal config high: ISA contention only (classic activation)
	m_eeprom[0x14] = 0x0001; // secondary software info: hardware revision 1 (B)

	uint8_t hi = 0, lo = 0;
	for (unsigned w = 0x00; w <= 0x0e; w++)
	{
		if (w == 0x08 || w == 0x09 || w == 0x0d)
			continue;
		hi ^= uint8_t(m_eeprom[w] & 0xff);
		hi ^= uint8_t(m_eeprom[w] >> 8);
	}
	for (unsigned w : { 0x08u, 0x09u, 0x0du })
	{
		lo ^= uint8_t(m_eeprom[w] & 0xff);
		lo ^= uint8_t(m_eeprom[w] >> 8);
	}
	m_eeprom[0x0f] = (uint16_t(hi) << 8) | lo;

	uint8_t hi2 = 0, lo2 = 0;
	for (unsigned w = 0x10; w <= 0x12; w++)
	{
		hi2 ^= uint8_t(m_eeprom[w] & 0xff);
		hi2 ^= uint8_t(m_eeprom[w] >> 8);
	}
	for (unsigned w = 0x20; w <= 0x3f; w++)
	{
		hi2 ^= uint8_t(m_eeprom[w] & 0xff);
		hi2 ^= uint8_t(m_eeprom[w] >> 8);
	}
	for (unsigned w = 0x13; w <= 0x16; w++)
	{
		lo2 ^= uint8_t(m_eeprom[w] & 0xff);
		lo2 ^= uint8_t(m_eeprom[w] >> 8);
	}
	m_eeprom[0x17] = (uint16_t(hi2) << 8) | lo2;
}


// ---------------------------------------------------------------------
// Activation / reset
// ---------------------------------------------------------------------

void isa8_3c509b_device::activate()
{
	if ((m_addr_config & 0x1f) == 0x1f)
	{
		LOGID("3c509b: activate refused, address config selects EISA (unsupported on ISA8)\n");
		return;
	}

	if (m_irq_line_state)
	{
		irq_out(0);
		m_irq_line_state = false;
	}

	m_iobase = 0x0200 + uint16_t(m_addr_config & 0x1f) * 0x10;

	// IRQ is purely informational on Sprinter (no ISA8 IRQ line is wired to
	// the Z80); an out-of-range value never blocks activation.
	static bool const valid_irq[16] =
	{
		false, false, false, true,  false, true,  false, true,
		false, true,  true,  true,  true,  false, false, true
	};
	uint8_t irq = (m_resource_config >> 12) & 0x0f;
	if (!valid_irq[irq])
		irq = 3;
	m_irq = irq;

	m_activated = true;
	update_irq();

	LOGID("3c509b: activated io=%04x irq=%u\n", m_iobase, m_irq);
}


void isa8_3c509b_device::deactivate()
{
	if (m_irq_line_state)
	{
		irq_out(0);
		m_irq_line_state = false;
	}
	m_activated = false;
}


void isa8_3c509b_device::global_reset()
{
	// Host-triggered reset (Command register GlobalReset, Config Control
	// RST bit, ID command 0xC0-0xCF): AUTOINIT takes on the order of
	// 250-310us, during which the operating window and the ID sequence
	// are not accepted (see op_r/op_w/id_r/id_w's CIP guards).
	deactivate();
	start_cip(OP_GLOBAL_RESET, 0, attotime::from_usec(300));
}


void isa8_3c509b_device::global_reset_finish()
{
	deactivate();

	m_window = 0;
	// Only ENA (bit0) and RST (bit2, non-sticky) are host-writable; the
	// remaining Configuration Control bits are POR status and are not
	// modeled precisely pending comparison against a real card trace
	// (see MAME_3C509B_ISA8.md section 10.3).
	m_config_control = 0x0000;
	m_addr_config = m_eeprom[0x08];
	m_resource_config = m_eeprom[0x09];

	m_status = 0;
	m_int_mask = 0;
	m_read_zero_mask = 0;
	m_rx_filter = 0;
	m_rx_early_thresh = 0;
	m_tx_avail_thresh = 0;
	m_tx_start_thresh = 0;
	m_rx_enabled = false;
	m_tx_enabled = false;
	// Internal Configuration: 32-bit register seeded from EEPROM words
	// 0x12 (low)/0x13 (high) - RAM size/partition and the classic
	// (ISA-contention-only) activation indicator.
	m_internal_config = uint32_t(m_eeprom[0x12]) | (uint32_t(m_eeprom[0x13]) << 16);

	m_tx_buf_len = 0;
	m_tx_len = 0;
	m_tx_expected = 0;
	m_txq_head = 0;
	m_txq_used = 0;
	m_txkick_timer->reset();
	m_tx_pending = false;
	m_tx_pending_len = 0;
	m_tx_pending_notify = false;
	m_tx_status_count = 0;

	m_rx_head = 0;
	m_rx_count = 0;
	m_rx_cursor = 0;

	// Window 2 station address: low byte of the pair at +00 is Address0,
	// high byte is Address1, and so on (see reg_read/reg_write, window 2).
	m_station[0] = uint8_t(m_eeprom[0x00]);
	m_station[1] = uint8_t(m_eeprom[0x00] >> 8);
	m_station[2] = uint8_t(m_eeprom[0x01]);
	m_station[3] = uint8_t(m_eeprom[0x01] >> 8);
	m_station[4] = uint8_t(m_eeprom[0x02]);
	m_station[5] = uint8_t(m_eeprom[0x02] >> 8);

	m_fifo_diag = 0x0000;
	m_net_diag = 0x0004; // ASIC revision 2 in bits 5..1, no loopback selected
	m_eth_status = 0x0000;
	m_media_status = 0x0000;
	set_loopback(false);

	std::fill(std::begin(m_stat_byte), std::end(m_stat_byte), uint8_t(0));
	m_stat_tx_bytes = 0;
	m_stat_rx_bytes = 0;
	m_stats_enabled = false;

	m_rd_latch_valid = false;
	m_wr_latch_valid = false;

	m_id_state = ID_IDLE;
	m_id_port = -1;
	m_id_lfsr = 0xff;
	m_id_pos = 0;
	m_id_tag = 0;

	LOGID("3c509b: global reset complete\n");

	if (BIT(m_config->read(), 7))
		activate();

	update_irq();
}


// ---------------------------------------------------------------------
// Status / IRQ
// ---------------------------------------------------------------------

uint16_t isa8_3c509b_device::masked_status() const
{
	// Read Zero Mask forces the corresponding status bit(s) to read back
	// as zero; a clear bit in the mask shows the real status bit value.
	return m_status & ~m_read_zero_mask;
}


void isa8_3c509b_device::update_irq()
{
	const uint16_t triggers = masked_status() & m_int_mask & 0x00fe;
	if (triggers)
		m_status |= ST_INT_LATCH;
	// Interrupt Latch is only ever cleared by AckIntr(IntLatch), never
	// automatically just because the triggering condition went away.

	const bool line = ((m_status & ST_INT_LATCH) != 0) && BIT(m_config_control, 0) && m_activated;
	if (line != m_irq_line_state)
	{
		m_irq_line_state = line;
		irq_out(line ? 1 : 0);
	}
}


void isa8_3c509b_device::irq_out(int state)
{
	LOGIRQ("3c509b: irq%u state=%d\n", m_irq, state);

	switch (m_irq)
	{
	case 2: case 9: m_isa->irq2_w(state); break;
	case 3:         m_isa->irq3_w(state); break;
	case 4:         m_isa->irq4_w(state); break;
	case 5:         m_isa->irq5_w(state); break;
	case 6:         m_isa->irq6_w(state); break;
	case 7:         m_isa->irq7_w(state); break;
	default: break; // 10/11/12/15 have no ISA8 line to route to
	}
}


// ---------------------------------------------------------------------
// Deferred (CIP-gated) command completion
// ---------------------------------------------------------------------

void isa8_3c509b_device::start_cip(pending_op op, uint16_t arg, attotime const &delay)
{
	m_pending_op = op;
	m_pending_arg = arg;
	m_status |= ST_CMD_IN_PROGRESS;
	m_eeprom_busy = (op == OP_EEPROM_READ) || (op == OP_EEPROM_WRITE) ||
			(op == OP_EEPROM_ERASE) || (op == OP_EEPROM_ERASE_ALL) || (op == OP_ID_EEPROM_READ);
	m_cmd_timer->adjust(delay);
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::cmd_timer_done)
{
	const uint8_t op = m_pending_op;
	const uint16_t arg = m_pending_arg;
	m_pending_op = OP_NONE;
	m_eeprom_busy = false;
	m_status &= ~ST_CMD_IN_PROGRESS;

	switch (op)
	{
	case OP_GLOBAL_RESET:
		global_reset_finish();
		break;
	case OP_EEPROM_READ:
		m_eeprom_data = m_eeprom[arg & 0x3f];
		break;
	case OP_EEPROM_WRITE:
		if (m_eeprom_wren)
			m_eeprom[arg & 0x3f] &= m_eeprom_data;
		break;
	case OP_EEPROM_ERASE:
		if (m_eeprom_wren)
			m_eeprom[arg & 0x3f] = 0xffff;
		break;
	case OP_EEPROM_ERASE_ALL:
		if (m_eeprom_wren)
			std::fill(std::begin(m_eeprom), std::end(m_eeprom), uint16_t(0xffff));
		break;
	case OP_ID_EEPROM_READ:
		m_id_eeprom_data = m_eeprom[arg & 0x3f];
		break;
	case OP_RX_DISCARD:
		rx_discard();
		break;
	case OP_GENERIC:
	case OP_NONE:
	default:
		break;
	}

	update_irq();
}


// ---------------------------------------------------------------------
// ID port (legacy activation sequence)
// ---------------------------------------------------------------------

uint8_t isa8_3c509b_device::id_lfsr_next(uint8_t v)
{
	return BIT(v, 7) ? uint8_t((v << 1) ^ 0xcf) : uint8_t(v << 1);
}


uint8_t isa8_3c509b_device::id_r(offs_t offset)
{
	if ((offset & 0x0f) != 0)
		return 0xff;
	if (m_pending_op == OP_GLOBAL_RESET)
		return 0xff; // AUTOINIT in progress: ID port not yet accepted
	if ((m_id_port < 0) || ((offset >> 4) != m_id_port))
		return 0xff;
	if (m_id_state != ID_CMD)
		return 0xff;
	if (m_id_tag != 0)
		return 0xff; // already tagged: no longer participates in contention

	const uint8_t result = 0xfe | BIT(m_id_eeprom_data, 15);
	if (!machine().side_effects_disabled())
		m_id_eeprom_data <<= 1;
	return result;
}


void isa8_3c509b_device::id_w(offs_t offset, uint8_t data)
{
	if ((offset & 0x0f) != 0)
		return;
	if (m_pending_op == OP_GLOBAL_RESET)
		return; // AUTOINIT in progress: ID sequence not yet accepted

	invalidate_latches();

	const int8_t port = int8_t(offset >> 4);

	if (data == 0x00)
	{
		if (m_id_port != port)
		{
			m_id_port = port;
			m_id_state = ID_ARMED;
			LOGID("3c509b: id port selected @ %04x (armed, waiting for second zero)\n", 0x0100 + (offset & 0xf0));
			return;
		}

		// second (or a later, resynchronizing) zero on the already
		// selected port: (re)prime the 255-byte LFSR sequence
		m_id_lfsr = 0xff;
		m_id_pos = 0;
		m_id_state = ID_SEQ;
		return;
	}

	if (m_id_port != port)
		return;

	switch (m_id_state)
	{
	case ID_SEQ:
		if (data == m_id_lfsr)
		{
			m_id_lfsr = id_lfsr_next(m_id_lfsr);
			m_id_pos++;
			if (m_id_pos >= 255)
			{
				m_id_state = ID_CMD;
				LOGID("3c509b: id sequence complete, entering command mode\n");
			}
		}
		else
		{
			m_id_lfsr = 0xff;
			m_id_pos = 0;
		}
		break;

	case ID_CMD:
		id_command(data);
		break;

	case ID_IDLE:
	case ID_ARMED:
	default:
		break; // stray byte before the two-zero handshake: ignored
	}
}


void isa8_3c509b_device::id_command(uint8_t cmd)
{
	if (m_status & ST_CMD_IN_PROGRESS)
	{
		LOGID("3c509b: id command %02x ignored, CIP busy\n", cmd);
		return;
	}

	LOGID("3c509b: id command %02x\n", cmd);

	if (cmd <= 0x7f)
	{
		m_id_state = ID_SEQ;
		m_id_lfsr = 0xff;
		m_id_pos = 0;
	}
	else if (cmd <= 0xbf)
	{
		start_cip(OP_ID_EEPROM_READ, cmd & 0x3f, attotime::from_usec(170));
	}
	else if (cmd <= 0xcf)
	{
		global_reset();
	}
	else if (cmd <= 0xd7)
	{
		m_id_tag = cmd & 0x07;
	}
	else if (cmd <= 0xdf)
	{
		if ((cmd & 0x07) != m_id_tag)
			m_id_state = ID_IDLE; // not addressed to us: drop out, needs a fresh two-zero handshake
	}
	else if (cmd <= 0xfe)
	{
		m_addr_config = (m_addr_config & ~0x1f) | (cmd & 0x1f);
		activate();
		m_id_state = ID_IDLE;
	}
	else // 0xff
	{
		activate();
		m_id_state = ID_IDLE;
	}
}


// ---------------------------------------------------------------------
// Operating window: byte-pair frontend
// ---------------------------------------------------------------------

void isa8_3c509b_device::invalidate_latches()
{
	m_rd_latch_valid = false;
	m_wr_latch_valid = false;
}


uint8_t isa8_3c509b_device::op_r(offs_t offset)
{
	if (!m_activated || (m_pending_op == OP_GLOBAL_RESET))
		return 0xff;
	const uint16_t port = 0x0200 + uint16_t(offset);
	if ((port < m_iobase) || (port > (m_iobase + 0x0f)))
		return 0xff;
	const uint8_t reg = uint8_t(port - m_iobase);

	// byte-stream/byte-only registers bypass the word latch entirely
	if ((m_window == 1) && ((reg == 0x00) || (reg == 0x02)))
	{
		invalidate_latches();
		return fifo_read_byte();
	}
	if ((m_window == 1) && ((reg == 0x01) || (reg == 0x03)))
	{
		invalidate_latches(); // odd PIO offsets are not valid FIFO ports
		return 0xff;
	}
	if ((m_window == 1) && (reg == 0x0a))
	{
		invalidate_latches();
		return 0xff; // Timer: reads as permanently "pegged"
	}
	if ((m_window == 1) && (reg == 0x0b))
	{
		invalidate_latches();
		// TX Status: reading peeks the head entry, it does not pop it.
		return m_tx_status_count ? m_tx_status_stack[0] : 0x00;
	}
	if ((m_window == 3) && (reg == 0x05))
	{
		invalidate_latches();
		return 0x00; // ROM Control: no boot ROM fitted
	}
	if ((m_window == 6) && (reg <= 0x08))
	{
		invalidate_latches();
		const uint8_t v = m_stat_byte[reg];
		m_stat_byte[reg] = 0;
		stats_full_recheck();
		return v;
	}

	LOGIO("3c509b: read w%u reg=%02x\n", m_window, reg);

	if (!(reg & 1))
	{
		const uint16_t word = reg_read(reg);
		m_rd_latch_valid = true;
		m_rd_latch_off = reg;
		m_rd_latch_hi = uint8_t(word >> 8);
		return uint8_t(word);
	}
	else
	{
		const uint8_t even = reg & ~1;
		if (m_rd_latch_valid && (m_rd_latch_off == even))
		{
			m_rd_latch_valid = false;
			return m_rd_latch_hi;
		}
		if (reg != 0x0f) // Status/Command allows any standalone byte read
			LOGID("3c509b: orphaned odd-byte read at reg=%02x (no preceding even read)\n", reg);
		return uint8_t(reg_read(even) >> 8);
	}
}


void isa8_3c509b_device::op_w(offs_t offset, uint8_t data)
{
	if (!m_activated || (m_pending_op == OP_GLOBAL_RESET))
		return;
	const uint16_t port = 0x0200 + uint16_t(offset);
	if ((port < m_iobase) || (port > (m_iobase + 0x0f)))
		return;
	const uint8_t reg = uint8_t(port - m_iobase);

	if ((m_window == 1) && ((reg == 0x00) || (reg == 0x02)))
	{
		invalidate_latches();
		fifo_write_byte(data);
		return;
	}
	if ((m_window == 1) && ((reg == 0x01) || (reg == 0x03)))
	{
		invalidate_latches(); // odd PIO offsets are not valid FIFO ports
		return;
	}
	if ((m_window == 1) && (reg == 0x0a))
	{
		invalidate_latches();
		return; // Timer: byte register, writes ignored
	}
	if ((m_window == 1) && (reg == 0x0b))
	{
		invalidate_latches();
		// TX Status: a write pops exactly one (already-read) head entry.
		if (m_tx_status_count)
		{
			m_tx_status_count--;
			for (uint8_t i = 0; i < m_tx_status_count; i++)
				m_tx_status_stack[i] = m_tx_status_stack[i + 1];
			if (m_tx_status_count == 0)
				m_status &= ~ST_TX_COMPLETE;
			update_irq();
		}
		return;
	}
	if ((m_window == 3) && (reg == 0x05))
	{
		invalidate_latches(); // ROM Control: not implemented, ignore writes
		return;
	}

	LOGIO("3c509b: write w%u reg=%02x data=%02x\n", m_window, reg, data);

	if (!(reg & 1))
	{
		m_wr_latch_valid = true;
		m_wr_latch_off = reg;
		m_wr_latch_lo = data;
		return;
	}
	else
	{
		const uint8_t even = reg & ~1;
		uint8_t lo = 0;
		if (m_wr_latch_valid && (m_wr_latch_off == even))
		{
			lo = m_wr_latch_lo;
		}
		else
		{
			LOGID("3c509b: orphaned odd-byte write at reg=%02x (no preceding even write, assuming lo=0)\n", reg);
		}
		m_wr_latch_valid = false;

		const uint16_t word = lo | (uint16_t(data) << 8);
		if (even == 0x0e)
			do_command(word);
		else
			reg_write(even, word);
	}
}


// ---------------------------------------------------------------------
// Register windows
// ---------------------------------------------------------------------

uint16_t isa8_3c509b_device::tx_used_bytes() const
{
	// Queued complete entries (preamble + DWORD-padded data, including the
	// entry currently on the wire, which is only popped by
	// send_complete_cb) plus whatever has been PIO-staged so far of the
	// frame under assembly.
	const uint32_t used = uint32_t(m_txq_used) + m_tx_buf_len;
	return uint16_t(std::min<uint32_t>(used, SRAM_PARTITION_BYTES));
}


uint16_t isa8_3c509b_device::rx_used_bytes() const
{
	uint32_t used = 0;
	for (unsigned i = 0; i < m_rx_count; i++)
	{
		const uint8_t slot = (m_rx_head + i) % RX_SLOTS;
		used += (uint32_t(m_rx_len[slot]) + 3) & ~uint32_t(3);
	}
	return uint16_t(std::min<uint32_t>(used, SRAM_PARTITION_BYTES));
}


uint16_t isa8_3c509b_device::reg_read(uint8_t reg)
{
	if (reg == 0x0e)
		return masked_status() | (uint16_t(m_window) << 13);

	switch (m_window)
	{
	case 0:
		switch (reg)
		{
		case 0x00: return m_eeprom[0x07];
		case 0x02: return m_eeprom[0x03];
		case 0x04: return m_config_control;
		case 0x06: return m_addr_config;
		case 0x08: return m_resource_config;
		case 0x0a: return m_eeprom_busy ? 0x8000 : 0x0000; // EBY
		case 0x0c: return m_eeprom_data;
		}
		break;

	case 1:
		switch (reg)
		{
		case 0x08:
			if (m_rx_count == 0)
				return 0x8000;
			{
				const uint8_t slot = m_rx_head;
				const uint16_t remaining = m_rx_len[slot] - std::min<uint16_t>(m_rx_cursor, m_rx_len[slot]);
				return (m_rx_status[slot] & 0xf800) | (remaining & 0x07ff);
			}
		case 0x0c:
			// TX Free, rounded down to a DWORD (Window 1 semantics)
			return (SRAM_PARTITION_BYTES - tx_used_bytes()) & ~uint16_t(3);
		}
		break;

	case 2:
		switch (reg)
		{
		case 0x00: return m_station[0] | (uint16_t(m_station[1]) << 8);
		case 0x02: return m_station[2] | (uint16_t(m_station[3]) << 8);
		case 0x04: return m_station[4] | (uint16_t(m_station[5]) << 8);
		case 0x06: return 0x0000; // station address mask, unused
		}
		break;

	case 3:
		switch (reg)
		{
		case 0x00: return uint16_t(m_internal_config);
		case 0x02: return uint16_t(m_internal_config >> 16);
		case 0x0a: return SRAM_PARTITION_BYTES - rx_used_bytes(); // RX Free, exact byte count
		case 0x0c: return SRAM_PARTITION_BYTES - tx_used_bytes(); // TX Free, exact byte count
		}
		break;

	case 4:
		switch (reg)
		{
		case 0x04: return m_fifo_diag;
		case 0x06: return m_net_diag;
		case 0x08: return m_eth_status;
		case 0x0a:
		{
			// Link Beat Detected (bit15, read-only) reflects whether a
			// real network backend or loopback is actually present, not
			// an unconditional constant; bits 7/6 are the host-writable
			// jabber-guard/link-beat enables stored in m_media_status.
			const bool link_beat_enabled = BIT(m_media_status, 6);
			const bool link_present = ((m_net_diag & 0xf000) != 0) || (get_interface() >= 0);
			uint16_t v = m_media_status & 0x00c0;
			if (link_beat_enabled && link_present)
				v |= 0x8000;
			return v;
		}
		}
		break;

	case 5:
		switch (reg)
		{
		case 0x00: return m_tx_start_thresh;
		case 0x02: return m_tx_avail_thresh;
		case 0x06: return m_rx_early_thresh;
		case 0x08: return m_rx_filter;
		case 0x0a: return m_int_mask;
		case 0x0c: return m_read_zero_mask;
		}
		break;

	case 6:
		switch (reg)
		{
		case 0x0a: { const uint16_t v = m_stat_rx_bytes; m_stat_rx_bytes = 0; stats_full_recheck(); return v; }
		case 0x0c: { const uint16_t v = m_stat_tx_bytes; m_stat_tx_bytes = 0; stats_full_recheck(); return v; }
		}
		break;
	}

	return 0xffff;
}


void isa8_3c509b_device::reg_write(uint8_t reg, uint16_t data)
{
	switch (m_window)
	{
	case 0:
		switch (reg)
		{
		case 0x04:
			m_config_control = (m_config_control & ~0x0001) | (data & 0x0001); // only ENA is writable
			update_irq();
			if (BIT(data, 2))
				global_reset(); // RST: not stored, triggers AUTOINIT
			break;
		case 0x06:
			m_addr_config = data;
			if ((data & 0x1f) == 0x1f)
				deactivate();
			break;
		case 0x08:
			m_resource_config = data;
			break;
		case 0x0a:
			eeprom_command(data);
			break;
		case 0x0c:
			m_eeprom_data = data;
			break;
		}
		break;

	case 2:
		switch (reg)
		{
		case 0x00: m_station[0] = uint8_t(data); m_station[1] = uint8_t(data >> 8); break;
		case 0x02: m_station[2] = uint8_t(data); m_station[3] = uint8_t(data >> 8); break;
		case 0x04: m_station[4] = uint8_t(data); m_station[5] = uint8_t(data >> 8); break;
		}
		break;

	case 3:
		switch (reg)
		{
		case 0x00: m_internal_config = (m_internal_config & 0xffff0000) | data; break;
		case 0x02: m_internal_config = (m_internal_config & 0x0000ffff) | (uint32_t(data) << 16); break;
		}
		break;

	case 4:
		switch (reg)
		{
		case 0x06:
			// bits 15..12 are the writable loopback select, bits 5..1
			// (ASIC revision) are read-only and preserved.
			m_net_diag = (data & 0xf000) | (m_net_diag & 0x003e);
			set_loopback((data & 0xf000) != 0);
			break;
		case 0x0a:
			// bits 7/6 (link beat enable, jabber guard enable) writable;
			// remaining bits are read-only status.
			m_media_status = (m_media_status & ~0x00c0) | (data & 0x00c0);
			break;
		}
		break;

	default:
		break;
	}
}


void isa8_3c509b_device::eeprom_command(uint16_t cmd)
{
	if (m_status & ST_CMD_IN_PROGRESS)
		return; // command engine busy, ignore

	const uint8_t addr = cmd & 0x3f;

	if ((cmd & 0xc0) == 0x80)
		start_cip(OP_EEPROM_READ, addr, attotime::from_usec(170));
	else if ((cmd & 0xc0) == 0x40)
		start_cip(OP_EEPROM_WRITE, addr, attotime::from_usec(170));
	else if ((cmd & 0xc0) == 0xc0)
		start_cip(OP_EEPROM_ERASE, addr, attotime::from_usec(170));
	else
	{
		switch (cmd & 0x30)
		{
		case 0x00: m_eeprom_wren = false; break; // EWDS
		case 0x20: start_cip(OP_EEPROM_ERASE_ALL, 0, attotime::from_usec(170)); break;
		case 0x30: m_eeprom_wren = true; break; // EWEN
		default: break; // write-all: not implemented, no driver depends on it
		}
	}
}


// ---------------------------------------------------------------------
// Command register (offset 0x0e, all windows)
// ---------------------------------------------------------------------

void isa8_3c509b_device::do_command(uint16_t cmd)
{
	if (m_status & ST_CMD_IN_PROGRESS)
	{
		LOGCMD("3c509b: command %04x ignored, CIP busy\n", cmd);
		return;
	}

	const uint8_t code = uint8_t(cmd >> 11);
	const uint16_t param = cmd & 0x07ff;

	LOGCMD("3c509b: command %u param=%03x\n", code, param);

	if (code == CMD_GLOBAL_RESET)
	{
		global_reset();
		return;
	}

	if (code == CMD_RX_DISCARD)
	{
		if (m_rx_count == 0)
			return;
		start_cip(OP_RX_DISCARD, 0, attotime::from_usec(5));
		return;
	}

	switch (code)
	{
	case CMD_SELECT_WINDOW:
		m_window = param & 0x07;
		break;
	case CMD_START_COAX:
	case CMD_STOP_COAX:
		break; // TPO card: no coax transceiver
	case CMD_RX_DISABLE:
		m_rx_enabled = false;
		break;
	case CMD_RX_ENABLE:
		m_rx_enabled = true;
		break;
	case CMD_RX_RESET:
		m_rx_head = 0;
		m_rx_count = 0;
		m_rx_cursor = 0;
		m_status &= ~ST_RX_COMPLETE;
		m_fifo_diag &= ~0x0003; // RX overrun + RX underrun diag bits
		break;
	case CMD_TX_ENABLE:
		m_tx_enabled = true;
		tx_kick(); // resume any entries queued before a TxDisable
		break;
	case CMD_TX_DISABLE:
		m_tx_enabled = false;
		break;
	case CMD_TX_RESET:
		m_tx_buf_len = 0;
		m_tx_len = 0;
		m_tx_expected = 0;
		m_txq_head = 0;
		m_txq_used = 0;
		m_txkick_timer->reset();
		m_tx_pending = false; // an in-flight send_complete_cb is now stale
		m_tx_status_count = 0;
		m_status &= ~(ST_TX_COMPLETE | ST_TX_AVAILABLE);
		m_fifo_diag &= ~0x0010; // TX overrun diag bit
		break;
	case CMD_FAKE_INTR:
		m_status |= ST_INT_REQUESTED;
		break;
	case CMD_ACK_INTR:
		m_status &= ~(param & ST_ACK_MASK);
		break;
	case CMD_SET_INTR_ENB:
		m_int_mask = param;
		break;
	case CMD_SET_STATUS_ENB:
		m_read_zero_mask = param;
		break;
	case CMD_SET_RX_FILTER:
		m_rx_filter = param & 0x0f;
		break;
	case CMD_SET_RX_THRESH:
		m_rx_early_thresh = param & 0x07fc;
		break;
	case CMD_SET_TX_THRESH:
		m_tx_avail_thresh = param & 0x07fc;
		// a driver may poll for TX Available before ever transmitting;
		// assert the event right away if the space is already free
		update_tx_available();
		break;
	case CMD_SET_TX_START:
		m_tx_start_thresh = param & 0x07fc;
		break;
	case CMD_STATS_ENABLE:
		m_stats_enabled = true;
		break;
	case CMD_STATS_DISABLE:
		m_stats_enabled = false;
		break;
	case CMD_POWER_UP:
	case CMD_POWER_DOWN:
	case CMD_POWER_AUTO:
		break;
	default:
		break;
	}

	// No published exact duration for these operations either; hold CIP
	// briefly so polling software observes a real (if short) busy window.
	start_cip(OP_GENERIC, 0, attotime::from_usec(2));
}


// ---------------------------------------------------------------------
// TX FIFO and TX Status
// ---------------------------------------------------------------------

void isa8_3c509b_device::fifo_write_byte(uint8_t data)
{
	if (!m_tx_enabled)
		return;
	if (((uint32_t(m_txq_used) + m_tx_buf_len) >= SRAM_PARTITION_BYTES) ||
			(m_tx_buf_len >= std::size(m_tx_buf)))
	{
		// TX partition exhausted: the driver wrote past TX Free.  FIFO
		// Diagnostic bit assignments are model-defined (the tech reference
		// does not publish the exact layout): bit4 = TX overrun, sticky
		// until TxReset/GlobalReset.
		m_fifo_diag |= 0x0010;
		return;
	}

	m_tx_buf[m_tx_buf_len++] = data;

	if (m_tx_buf_len == 2)
		m_tx_len = m_tx_buf[0] | (uint16_t(m_tx_buf[1]) << 8);

	if (m_tx_buf_len == 4)
	{
		// preamble word0 = length/flags, word1 = reserved; frame data is
		// padded up to the next DWORD boundary after the preamble.
		const uint16_t len = m_tx_len & 0x07ff;
		m_tx_expected = 4 + ((len + 3) & ~uint16_t(3));
	}

	if (m_tx_expected && (m_tx_buf_len >= m_tx_expected))
	{
		// completed entry: append it verbatim to the TX queue ring and
		// (if idle) put it on the wire
		for (uint16_t i = 0; i < m_tx_buf_len; i++)
			m_txq[(uint32_t(m_txq_head) + m_txq_used + i) % SRAM_PARTITION_BYTES] = m_tx_buf[i];
		m_txq_used += m_tx_buf_len;
		m_tx_buf_len = 0;
		m_tx_len = 0;
		m_tx_expected = 0;
		tx_kick();
	}
}


void isa8_3c509b_device::tx_kick()
{
	if (m_tx_pending || !m_tx_enabled || (m_txq_used < 4))
		return;

	const uint16_t len_word = m_txq[m_txq_head] |
			(uint16_t(m_txq[(m_txq_head + 1) % SRAM_PARTITION_BYTES]) << 8);
	const uint16_t len = len_word & 0x07ff;
	const bool notify = BIT(len_word, 15);

	uint8_t frame[2048];
	const uint16_t frame_len = std::max<uint16_t>(len, 60);
	for (uint16_t i = 0; i < len; i++)
		frame[i] = m_txq[(uint32_t(m_txq_head) + 4 + i) % SRAM_PARTITION_BYTES];
	std::fill(frame + len, frame + frame_len, uint8_t(0x00));

	LOGNET("3c509b: tx launch len=%u (padded=%u) notify=%d queued=%u\n", len, frame_len, notify, m_txq_used);

	m_tx_pending = true;
	m_tx_pending_len = frame_len;
	m_tx_pending_notify = notify;

	// send() schedules send_complete_cb() at the configured bandwidth; if
	// loopback is active it also synchronously invokes recv_start_cb() and
	// schedules recv_complete_cb() the same way (device_network_interface).
	// The entry stays in the queue (and in the TX Free accounting) until
	// send_complete_cb pops it.
	send(frame, frame_len);
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::txkick_timer_done)
{
	tx_kick();
}


void isa8_3c509b_device::send_complete_cb(int result)
{
	if (!m_tx_pending)
		return;
	m_tx_pending = false;

	// pop the transmitted entry from the TX queue ring
	const uint16_t len_word = m_txq[m_txq_head] |
			(uint16_t(m_txq[(m_txq_head + 1) % SRAM_PARTITION_BYTES]) << 8);
	const uint16_t entry = 4 + (((len_word & 0x07ff) + 3) & ~uint16_t(3));
	const uint16_t pop = std::min<uint16_t>(entry, m_txq_used);
	m_txq_head = (m_txq_head + pop) % SRAM_PARTITION_BYTES;
	m_txq_used -= pop;

	if (m_tx_pending_notify)
	{
		if (m_tx_status_count >= TX_STATUS_DEPTH)
		{
			// stack overflow: flag it in the newest entry and disable the
			// transmitter until TxReset (0x04 = Status Overflow)
			m_tx_status_stack[TX_STATUS_DEPTH - 1] |= 0x84;
			m_tx_enabled = false;
		}
		else
		{
			m_tx_status_stack[m_tx_status_count++] = 0x80; // Complete + Interrupt-on-Success-Requested, no error
		}
		m_status |= ST_TX_COMPLETE;
	}

	if (m_stats_enabled)
	{
		bump_stat(6); // frames transmitted OK
		const uint32_t total = uint32_t(m_stat_tx_bytes) + m_tx_pending_len;
		m_stat_tx_bytes = uint16_t(std::min<uint32_t>(total, 0xffff));
		if (m_stat_tx_bytes == 0xffff)
			m_status |= ST_STATS_FULL;
	}

	update_tx_available();
	update_irq();

	// next queued frame goes out after the standard 10 Mbps interframe
	// gap (96 bit times = 9.6us)
	if (m_txq_used)
		m_txkick_timer->adjust(attotime::from_nsec(9600));
}


void isa8_3c509b_device::update_tx_available()
{
	// TX Available is a latched event (cleared via AckIntr): it asserts
	// whenever the DWORD-rounded TX free space reaches the programmed
	// threshold, including immediately at SetTxAvailableThreshold time if
	// the space is already there.
	const uint16_t free = (SRAM_PARTITION_BYTES - tx_used_bytes()) & ~uint16_t(3);
	if (free >= m_tx_avail_thresh)
	{
		m_status |= ST_TX_AVAILABLE;
		update_irq();
	}
}


void isa8_3c509b_device::bump_stat(uint8_t index)
{
	if (!m_stats_enabled)
		return;
	if (m_stat_byte[index] != 0xff)
		m_stat_byte[index]++;
	if (m_stat_byte[index] == 0xff)
	{
		m_status |= ST_STATS_FULL;
		update_irq();
	}
}


void isa8_3c509b_device::stats_full_recheck()
{
	// Statistics Full is not in the AckIntr mask; it clears once the
	// saturated counter(s) have been read (reading clears the counter).
	if (!(m_status & ST_STATS_FULL))
		return;
	for (uint8_t v : m_stat_byte)
		if (v == 0xff)
			return;
	if ((m_stat_tx_bytes == 0xffff) || (m_stat_rx_bytes == 0xffff))
		return;
	m_status &= ~ST_STATS_FULL;
	update_irq();
}


// ---------------------------------------------------------------------
// RX FIFO and RX Discard
// ---------------------------------------------------------------------

uint8_t isa8_3c509b_device::fifo_read_byte()
{
	if (m_rx_count == 0)
	{
		m_fifo_diag |= 0x0002; // RX underrun (model-defined bit), sticky until RxReset
		return 0x00;
	}

	const uint8_t slot = m_rx_head;
	const uint16_t len = m_rx_len[slot];
	// data beyond the frame length up to the DWORD-padded boundary reads
	// back as zero; reading the head packet's data never removes it, only
	// an explicit RX_DISCARD command advances the ring.
	const uint16_t padded = (len + 3) & ~uint16_t(3);
	if (m_rx_cursor >= padded)
		return 0x00;

	const uint8_t v = (m_rx_cursor < len) ? m_rx_data[slot][m_rx_cursor] : 0x00;
	m_rx_cursor++;
	return v;
}


void isa8_3c509b_device::rx_discard()
{
	if (m_rx_count == 0)
		return;

	m_rx_head = (m_rx_head + 1) % RX_SLOTS;
	m_rx_count--;
	m_rx_cursor = 0;

	if (m_rx_count == 0)
		m_status &= ~ST_RX_COMPLETE;
	else
		m_status |= ST_RX_COMPLETE; // next packet is now visible as head

	update_irq();
}


bool isa8_3c509b_device::accept_filter(uint8_t const *buf, int length) const
{
	if (length < 6)
		return false;
	if (m_rx_filter & RXF_PROMISC)
		return true;

	bool bcast = true;
	for (int i = 0; i < 6 && bcast; i++)
		if (buf[i] != 0xff)
			bcast = false;

	if (bcast)
		return (m_rx_filter & (RXF_BROADCAST | RXF_GROUP)) != 0; // group also accepts broadcast

	if (buf[0] & 0x01)
		return (m_rx_filter & RXF_GROUP) != 0;

	if (m_rx_filter & RXF_INDIVIDUAL)
		return std::equal(buf, buf + 6, m_station);

	return false;
}


int isa8_3c509b_device::recv_start_cb(uint8_t *buf, int length)
{
	if (!m_activated || !m_rx_enabled || (m_pending_op == OP_GLOBAL_RESET))
		return 0;
	if ((length < 6) || (length > int(RX_SLOT_BYTES)))
		return 0;
	if (!accept_filter(buf, length))
		return 0;
	const uint16_t padded = (uint16_t(length) + 3) & ~uint16_t(3);
	if ((m_rx_count >= RX_SLOTS) ||
			((uint32_t(rx_used_bytes()) + padded) > SRAM_PARTITION_BYTES))
	{
		bump_stat(5); // RX overruns
		m_fifo_diag |= 0x0001; // RX Overrun (sticky until RxReset/GlobalReset)
		return 0; // RX partition/ring full, drop
	}

	std::copy_n(buf, length, m_rx_stage_data);
	m_rx_stage_len = uint16_t(length);

	uint16_t status = uint16_t(length) & 0x07ff;
	if (length < 60)
		status |= 0x5800;
	else if (length > 1514)
		status |= 0x4800;
	m_rx_stage_status = status;

	LOGNET("3c509b: rx staged len=%d\n", length);
	return length; // nonzero => accepted; recv_complete_cb() commits it after the simulated transfer delay
}


void isa8_3c509b_device::recv_complete_cb(int result)
{
	if (result <= 0)
		return;
	// the receiver may have been reset or disabled while the packet was
	// still "on the wire"; a reset must not resurrect a stale staged frame
	if (!m_rx_enabled)
		return;
	if (m_rx_count >= RX_SLOTS)
		return;

	const uint8_t slot = (m_rx_head + m_rx_count) % RX_SLOTS;
	std::copy_n(m_rx_stage_data, m_rx_stage_len, m_rx_data[slot]);
	m_rx_len[slot] = m_rx_stage_len;
	m_rx_status[slot] = m_rx_stage_status;
	m_rx_count++;

	if (m_stats_enabled)
	{
		bump_stat(7); // frames received OK
		const uint32_t total = uint32_t(m_stat_rx_bytes) + m_rx_stage_len;
		m_stat_rx_bytes = uint16_t(std::min<uint32_t>(total, 0xffff));
		if (m_stat_rx_bytes == 0xffff)
			m_status |= ST_STATS_FULL;
	}

	m_status |= ST_RX_COMPLETE;
	// RX Early fires once accumulated bytes exceed the threshold; a
	// threshold beyond the 1792-byte FIFO effectively disables the event.
	if ((m_rx_early_thresh <= 1792) && (m_rx_stage_len > m_rx_early_thresh))
		m_status |= ST_RX_EARLY;

	update_irq();

	LOGNET("3c509b: rx committed len=%u slot=%u\n", m_rx_stage_len, slot);
}
