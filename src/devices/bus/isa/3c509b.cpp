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
#include <array>


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

// Bits the Read Zero mask can force to zero.  The Interrupt Latch (bit 0)
// is explicitly exempt (tech reference 6.9), and the Read Zero mask
// argument is only 11 bits wide, so Command-in-Progress and the window
// number are always visible.
constexpr uint16_t ST_ZERO_MASKABLE    = 0x07fe;

// Threshold registers: any value above the longest receivable packet
// disables the corresponding event, and 2,044 is the 3C509B power-on
// default for all three (tech reference 6.10/6.11 and 10.6).
constexpr uint16_t THRESH_DISABLED  = 2044;
constexpr uint16_t THRESH_MAX_ACTIVE = 1792;

// FIFO Diagnostic port bits (Window 4, offset 0x04; tech reference 6.31).
// Only the two sticky failure bits are stored, the rest are derived.
constexpr uint16_t FD_RX_RECEIVING = 0x8000; // read-only, packet in flight
constexpr uint16_t FD_RX_UNDERRUN  = 0x2000; // sticky, needs RxReset/GlobalReset
constexpr uint16_t FD_RX_OVERRUN   = 0x0800; // read-only, RX FIFO full
constexpr uint16_t FD_TX_OVERRUN   = 0x0400; // sticky, needs TxReset/GlobalReset
constexpr uint16_t FD_STICKY       = FD_RX_UNDERRUN | FD_TX_OVERRUN;

// TX Status register bits (Window 1, offset 0x0b; tech reference 6.18)
constexpr uint8_t TXS_COMPLETE = 0x80;
constexpr uint8_t TXS_INT_REQ  = 0x40; // interrupt on successful completion requested
constexpr uint8_t TXS_JABBER   = 0x20; // TX Reset required
constexpr uint8_t TXS_UNDERRUN = 0x10; // TX Reset required
constexpr uint8_t TXS_MAX_COLL = 0x08;
constexpr uint8_t TXS_OVERFLOW = 0x04;
constexpr uint8_t TXS_FATAL    = TXS_JABBER | TXS_UNDERRUN;

// RX filter bits (Window 5, offset 0x08)
constexpr uint8_t RXF_INDIVIDUAL = 0x01;
constexpr uint8_t RXF_GROUP      = 0x02;
constexpr uint8_t RXF_BROADCAST  = 0x04;
constexpr uint8_t RXF_PROMISC    = 0x08;

constexpr int LINK_RECOVERY_SECONDS = 3;

constexpr int ethernet_wire_length(bool loopback, int captured_length)
{
	return (!loopback && (captured_length >= 14) && (captured_length < 60)) ? 60 : captured_length;
}

static_assert(ethernet_wire_length(false, 42) == 60); // macOS feth ARP reply
static_assert(ethernet_wire_length(true, 42) == 42);  // controller runt test

// Exact contents read twice through the legacy ID port from a physical
// 3C509B-TPO (assembly 03-0020-002, revision 3, EA=00:20:AF:5D:69:8B) on
// 2026-09-08.  The fixture profile deliberately keeps this image immutable.
constexpr std::array<uint16_t, 0x40> PHYSICAL_TPO_EEPROM =
{
	0x0020, 0xaf5d, 0x698b, 0x9550, 0xb434, 0x0041, 0x4a41, 0x6d50,
	0x0010, 0x3000, 0x0020, 0xaf5d, 0x698b, 0x1310, 0x0000, 0x3223,
	0x2083, 0x0000, 0x0000, 0x0004, 0x0001, 0x0000, 0x0000, 0x0205,
	0x6d50, 0x9550, 0x698b, 0xaf5d, 0x0a5b, 0x1010, 0x1982, 0x3300,
	0x6f43, 0x206d, 0x4333, 0x3035, 0x4239, 0x4520, 0x6874, 0x7265,
	0x694c, 0x6b6e, 0x4920, 0x4949, 0x5015, 0x506d, 0x0295, 0x411c,
	0x80d0, 0x22f7, 0x9ea8, 0x0147, 0x0210, 0x03e0, 0x1010, 0x3779,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

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
	m_eeprom_profile(*this, "EEPROM"),
	m_link_recovery(*this, "LINK"),
	m_cmd_timer(nullptr),
	m_txkick_timer(nullptr),
	m_txdrop_timer(nullptr),
	m_link_timer(nullptr),
	m_eeprom{ },
	m_eeprom_data(0),
	m_eeprom_busy(false),
	m_eeprom_wren(false),
	m_id_port(-1),
	m_id_state(ID_IDLE),
	m_id_lfsr(0xff),
	m_id_pos(0),
	m_id_tag(0),
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
	m_link_state(LINK_DOWN),
	m_status(0),
	m_int_mask(0),
	m_read_zero_mask(0),
	m_rx_filter(0),
	m_rx_early_thresh(THRESH_DISABLED),
	m_tx_avail_thresh(THRESH_DISABLED),
	m_tx_start_thresh(THRESH_DISABLED),
	m_rx_enabled(false),
	m_tx_enabled(false),
	m_internal_config(0),
	m_timer_base(0),
	m_tx_buf{ },
	m_tx_buf_len(0),
	m_tx_len(0),
	m_tx_expected(0),
	m_tx_error(0),
	m_txq{ },
	m_txq_head(0),
	m_txq_used(0),
	m_tx_pending(false),
	m_tx_pending_len(0),
	m_tx_pending_notify(false),
	m_tx_reset_required(false),
	m_tx_status_overflow(false),
	m_tx_status_stack{ },
	m_tx_status_count(0),
	m_rxq{ },
	m_rxq_head(0),
	m_rxq_used(0),
	m_rx_len{ },
	m_rx_status{ },
	m_rx_head(0),
	m_rx_count(0),
	m_rx_cursor(0),
	m_rx_receiving(false),
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
	m_txdrop_timer = timer_alloc(FUNC(isa8_3c509b_device::txdrop_timer_done), this);
	m_link_timer = timer_alloc(FUNC(isa8_3c509b_device::link_timer_done), this);

	// Establish the per-instance default once.  Machine network
	// configuration is loaded after device_start and before device_reset, so
	// a configured MAC can replace this value before the EEPROM is built.
	uint32_t mac_tail = 0x509b3c;
	for (char const *p = tag(); *p != '\0'; p++)
		mac_tail = (mac_tail * 33) ^ uint8_t(*p);
	uint8_t mac[6] = { 0x02, 0x60, 0x8c, 0x00, 0x00, 0x00 };
	put_u24be(&mac[3], mac_tail);
	set_mac(mac);

	save_item(NAME(m_eeprom));
	save_item(NAME(m_eeprom_data));
	save_item(NAME(m_eeprom_busy));
	save_item(NAME(m_eeprom_wren));
	save_item(NAME(m_id_port));
	save_item(NAME(m_id_state));
	save_item(NAME(m_id_lfsr));
	save_item(NAME(m_id_pos));
	save_item(NAME(m_id_tag));
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
	save_item(NAME(m_link_state));
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
	save_item(NAME(m_timer_base));
	save_item(NAME(m_tx_buf));
	save_item(NAME(m_tx_buf_len));
	save_item(NAME(m_tx_len));
	save_item(NAME(m_tx_expected));
	save_item(NAME(m_tx_error));
	save_item(NAME(m_txq));
	save_item(NAME(m_txq_head));
	save_item(NAME(m_txq_used));
	save_item(NAME(m_tx_pending));
	save_item(NAME(m_tx_pending_len));
	save_item(NAME(m_tx_pending_notify));
	save_item(NAME(m_tx_reset_required));
	save_item(NAME(m_tx_status_overflow));
	save_item(NAME(m_tx_status_stack));
	save_item(NAME(m_tx_status_count));
	save_item(NAME(m_rxq));
	save_item(NAME(m_rxq_head));
	save_item(NAME(m_rxq_used));
	save_item(NAME(m_rx_len));
	save_item(NAME(m_rx_status));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_rx_cursor));
	save_item(NAME(m_rx_receiving));
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
	m_txdrop_timer->reset();
	reset_link_recovery();

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

	PORT_START("EEPROM")
	PORT_CONFNAME(0x01, 0x00, "EEPROM profile")
	PORT_CONFSETTING(0x00, "Dynamic")
	PORT_CONFSETTING(0x01, "Physical TPO fixture")

	PORT_START("LINK")
	PORT_CONFNAME(0x01, 0x00, "Link recovery")
	PORT_CONFSETTING(0x00, "Realistic (3 seconds)")
	PORT_CONFSETTING(0x01, "Immediate")
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
	std::copy(PHYSICAL_TPO_EEPROM.begin(), PHYSICAL_TPO_EEPROM.end(), std::begin(m_eeprom));

	if (BIT(m_eeprom_profile->read(), 0))
	{
		static constexpr uint8_t physical_mac[6] = { 0x00, 0x20, 0xaf, 0x5d, 0x69, 0x8b };
		set_mac(physical_mac);
	}
	else
	{
		auto const &mac = get_mac();

		// The serial EEPROM stores each pair in network order (Address 2n in
		// the word's high byte).  Window 2 is a distinct ISA register view and
		// remains low-port-byte first.
		m_eeprom[0x00] = (uint16_t(mac[0]) << 8) | mac[1];
		m_eeprom[0x01] = (uint16_t(mac[2]) << 8) | mac[3];
		m_eeprom[0x02] = (uint16_t(mac[4]) << 8) | mac[5];

		const uint8_t config = m_config->read();
		m_eeprom[0x08] = decode_iobase_index(config); // XCVR select = 00 (TPO)
		m_eeprom[0x09] = uint16_t(decode_irq(config)) << 12;

		m_eeprom[0x0a] = m_eeprom[0x00]; // OEM node address
		m_eeprom[0x0b] = m_eeprom[0x01];
		m_eeprom[0x0c] = m_eeprom[0x02];

		// Keep the PnP serial tied to the factory address just as it is in the
		// physical image, then regenerate the serial-isolation checksum.
		m_eeprom[0x1a] = m_eeprom[0x02];
		m_eeprom[0x1b] = m_eeprom[0x01];
		m_eeprom[0x1c] = (m_eeprom[0x1c] & 0xff00) | pnp_serial_checksum(m_eeprom);
	}

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
	for (unsigned w = 0x18; w <= 0x3f; w++)
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


uint8_t isa8_3c509b_device::pnp_serial_checksum(uint16_t const *eeprom)
{
	// ISA PnP 1.0a appendix B.2: seed 6Ah; feed the vendor/product and
	// serial bytes least-significant bit first.  EEPROM words are presented
	// low byte then high byte to the PnP serial shifter.
	uint8_t lfsr = 0x6a;
	for (unsigned word = 0x18; word <= 0x1b; word++)
	{
		for (unsigned byte = 0; byte < 2; byte++)
		{
			const uint8_t data = uint8_t(eeprom[word] >> (byte * 8));
			for (unsigned bit = 0; bit < 8; bit++)
			{
				const uint8_t feedback = BIT(lfsr, 1) ^ BIT(lfsr, 0) ^ BIT(data, bit);
				lfsr = (lfsr >> 1) | (feedback << 7);
			}
		}
	}
	return lfsr;
}


// ---------------------------------------------------------------------
// Activation / reset
// ---------------------------------------------------------------------

bool isa8_3c509b_device::tpo_selected() const
{
	return ((m_addr_config >> 14) & 0x03) == 0;
}


void isa8_3c509b_device::reset_link_recovery()
{
	m_link_timer->reset();
	m_link_state = LINK_DOWN;
}


void isa8_3c509b_device::maybe_start_link_recovery()
{
	if ((m_link_state != LINK_DOWN) || !m_activated || !tpo_selected() || !BIT(m_media_status, 7))
		return;

	if (BIT(m_link_recovery->read(), 0))
	{
		m_link_state = LINK_READY;
		LOGNET("3c509b: TPO link ready immediately\n");
	}
	else
	{
		m_link_state = LINK_RECOVERING;
		m_link_timer->adjust(attotime::from_seconds(LINK_RECOVERY_SECONDS));
		LOGNET("3c509b: TPO link recovery started (3 seconds)\n");
	}
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::link_timer_done)
{
	if ((m_link_state == LINK_RECOVERING) && m_activated && tpo_selected() && BIT(m_media_status, 7))
	{
		m_link_state = LINK_READY;
		LOGNET("3c509b: TPO link recovery complete\n");
	}
	else
	{
		m_link_state = LINK_DOWN;
	}
}


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
	maybe_start_link_recovery();
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
	// RST bit, ID command 0xC0-0xCF): the adapter is invisible to software
	// until the automatic configuration logic has finished rereading the
	// EEPROM, which the tech reference (7.4) documents as 310us.  The
	// operating window and the ID sequence are refused for that whole
	// interval (see op_r/op_w/id_r/id_w's CIP guards), so a guest that
	// honours the documented delay always finds the card ready.
	reset_link_recovery();
	deactivate();
	start_cip(OP_GLOBAL_RESET, 0, attotime::from_usec(310));
}


void isa8_3c509b_device::global_reset_finish()
{
	deactivate();

	m_window = 0;
	// Configuration Control: ENA (bit0) is host-writable and RST (bit2) is
	// a non-sticky trigger; bits 13..8 are power-on-reset capability bits
	// describing which transceivers the adapter carries (tech reference
	// 7.15).  A 3C509B-TPO has only the on-board 10BASE-T transceiver and
	// the internal VCO, and reports "normal operation mode" in bits 11:10.
	m_config_control = 0x0f00;
	m_addr_config = m_eeprom[0x08];
	m_resource_config = m_eeprom[0x09];

	m_status = 0;
	m_int_mask = 0;
	// "At power-up, the Read Zero mask defaults to zero" (tech reference
	// 6.9): every maskable indication reads back as zero until the driver
	// enables it with SetStatusEnb.
	m_read_zero_mask = 0;
	m_rx_filter = 0;
	m_rx_early_thresh = THRESH_DISABLED;
	m_tx_avail_thresh = THRESH_DISABLED;
	m_tx_start_thresh = THRESH_DISABLED;
	m_rx_enabled = false;
	m_tx_enabled = false;
	// Internal Configuration: 32-bit register seeded from EEPROM words
	// 0x12 (low)/0x13 (high) - RAM size/partition and the classic
	// (ISA-contention-only) activation indicator.
	m_internal_config = uint32_t(m_eeprom[0x12]) | (uint32_t(m_eeprom[0x13]) << 16);

	timer_restart();

	m_tx_buf_len = 0;
	m_tx_len = 0;
	m_tx_expected = 0;
	m_tx_error = 0;
	m_txq_head = 0;
	m_txq_used = 0;
	m_txkick_timer->reset();
	m_txdrop_timer->reset();
	m_tx_pending = false;
	m_tx_pending_len = 0;
	m_tx_pending_notify = false;
	m_tx_reset_required = false;
	m_tx_status_overflow = false;
	m_tx_status_count = 0;

	m_rxq_head = 0;
	m_rxq_used = 0;
	m_rx_head = 0;
	m_rx_count = 0;
	m_rx_cursor = 0;
	m_rx_receiving = false;
	m_rx_stage_len = 0;

	// The EEPROM representation is big-endian within a word.  Window 2 is
	// still exposed to ISA as Address0 on the low byte port, Address1 on the
	// high byte port.  Keep the network-interface identity in lockstep too.
	m_station[0] = uint8_t(m_eeprom[0x00] >> 8);
	m_station[1] = uint8_t(m_eeprom[0x00]);
	m_station[2] = uint8_t(m_eeprom[0x01] >> 8);
	m_station[3] = uint8_t(m_eeprom[0x01]);
	m_station[4] = uint8_t(m_eeprom[0x02] >> 8);
	m_station[5] = uint8_t(m_eeprom[0x02]);
	set_mac(m_station);

	m_fifo_diag = 0x0000;
	m_net_diag = 0x0000; // no loopback selected; the read value adds the derived bits
	m_eth_status = 0x0000;
	// Media Type and Status: link beat (bit7) and jabber (bit6) default to
	// disabled and must be enabled by the driver (tech reference 6.27).
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
	// Read Zero Mask: a *clear* bit forces the corresponding Status bit to
	// read as zero; a set bit shows the real value.  The Interrupt Latch is
	// exempt and Command-in-Progress is outside the 11-bit mask.
	return m_status & ~(ST_ZERO_MASKABLE & ~m_read_zero_mask);
}


void isa8_3c509b_device::update_irq()
{
	// "The Read Zero mask is applied to the Status register before the
	// Interrupt mask.  Clearing a bit in the Read Zero mask also prevents it
	// from causing interrupts." (tech reference 6.9)
	const bool was_latched = (m_status & ST_INT_LATCH) != 0;
	const uint16_t triggers = masked_status() & m_int_mask & 0x00fe;
	if (triggers)
		m_status |= ST_INT_LATCH;
	// Interrupt Latch is only ever cleared by AckIntr(IntLatch), never
	// automatically just because the triggering condition went away.

	if (!was_latched && (m_status & ST_INT_LATCH))
		timer_restart(); // Timer resets on every inactive->active interrupt edge

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
// Window 1 Timer (free-running 3.2us counter)
// ---------------------------------------------------------------------

uint8_t isa8_3c509b_device::timer_value() const
{
	// "The Timer register is a free-running 8-bit counter running off a
	// 10 MHz/32 clock period (period = 3.2us).  When the Timer reaches 255,
	// it stops incrementing." (tech reference 6.22)
	const uint64_t now = machine().time().as_ticks(TIMER_CLOCK);
	const uint64_t elapsed = (now > m_timer_base) ? (now - m_timer_base) : 0;
	return uint8_t(std::min<uint64_t>(elapsed / 32, 0xff));
}


void isa8_3c509b_device::timer_restart()
{
	m_timer_base = machine().time().as_ticks(TIMER_CLOCK);
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
			(op == OP_EEPROM_ERASE) || (op == OP_EEPROM_ERASE_ALL) ||
			(op == OP_EEPROM_EWEN) || (op == OP_EEPROM_EWDS);
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
		if (m_eeprom_wren && !BIT(m_eeprom_profile->read(), 0))
			m_eeprom[arg & 0x3f] &= m_eeprom_data;
		// "the hardware times the write strobe and then automatically
		// executes the Erase/Write Disable command" (tech reference 7.22)
		m_eeprom_wren = false;
		break;
	case OP_EEPROM_ERASE:
		if (m_eeprom_wren && !BIT(m_eeprom_profile->read(), 0))
			m_eeprom[arg & 0x3f] = 0xffff;
		m_eeprom_wren = false;
		break;
	case OP_EEPROM_ERASE_ALL:
		if (m_eeprom_wren && !BIT(m_eeprom_profile->read(), 0))
			std::fill(std::begin(m_eeprom), std::end(m_eeprom), uint16_t(0xffff));
		m_eeprom_wren = false;
		break;
	case OP_EEPROM_EWEN:
		m_eeprom_wren = true;
		break;
	case OP_EEPROM_EWDS:
		m_eeprom_wren = false;
		break;
	case OP_RX_DISCARD:
		rx_discard();
		break;
	case OP_RX_RESET:
		rx_reset();
		break;
	case OP_TX_RESET:
		tx_reset();
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

	// Contention test: the adapter drives bit 15 of the EEPROM Data register
	// onto bit 0 of the host bus and shifts the register left.  While an
	// EEPROM access is still in flight the register has not been reloaded
	// yet, so the shifter must hold: neither the previous word's remaining
	// bits nor the word being fetched may advance (tech reference 7.22:
	// software must wait for EEPROM Busy to clear).
	const uint8_t result = 0xfe | BIT(m_eeprom_data, 15);
	if (m_eeprom_busy)
	{
		LOGID("3c509b: id port read while EEPROM busy, shifter held\n");
		return result;
	}
	if (!machine().side_effects_disabled())
		m_eeprom_data <<= 1;
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
		// Read EEPROM word n into the (single, shared) EEPROM Data
		// register; the documented Read Register execution time is 162us.
		start_cip(OP_EEPROM_READ, cmd & 0x3f, attotime::from_usec(162));
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
		return timer_value();
	}
	if ((m_window == 1) && (reg == 0x0b))
	{
		invalidate_latches();
		// TX Status: reading peeks the head entry, it does not pop it.  The
		// Status Overflow bit is a register-level condition rather than a
		// property of one stacked entry, so it rides along with whatever
		// entry is currently readable until a write clears it.
		return (m_tx_status_count ? m_tx_status_stack[0] : 0x00) |
				(m_tx_status_overflow ? TXS_OVERFLOW : 0x00);
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
		return; // Timer: read-only byte register
	}
	if ((m_window == 1) && (reg == 0x0b))
	{
		invalidate_latches();
		pop_tx_status();
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
	return m_rxq_used;
}


bool isa8_3c509b_device::rx_fifo_full() const
{
	// FIFO Diagnostic RX Overrun: "set when the RX FIFO is full ... the
	// condition is cleared once a few bytes have been read out" (tech
	// reference 6.31).  A partition with less than one minimum-size frame
	// left, or an exhausted descriptor ring, counts as full.
	return (m_rx_count >= RX_SLOTS) || ((SRAM_PARTITION_BYTES - m_rxq_used) < 64);
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
				const uint16_t slot = m_rx_head;
				// RX Bytes counts down as the packet is read and, per tech
				// reference 6.18, keeps going negative (11-bit two's
				// complement) once the host reads into the pad bytes.
				const int32_t remaining = int32_t(m_rx_len[slot]) - int32_t(m_rx_cursor);
				return (m_rx_status[slot] & 0xf800) | (uint16_t(remaining) & 0x07ff);
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
		case 0x04:
		{
			// FIFO Diagnostic: only RX Underrun and TX Overrun are sticky
			// (and need RxReset/TxReset to clear); RX Receiving and RX
			// Overrun track the current FIFO state.  The BIST bits are
			// reserved/undefined on the B revision.
			uint16_t v = m_fifo_diag & FD_STICKY;
			if (m_rx_receiving)
				v |= FD_RX_RECEIVING;
			if (rx_fifo_full())
				v |= FD_RX_OVERRUN;
			return v;
		}
		case 0x06:
		{
			// Net Diagnostics: bits 15..12 are the writable loopback
			// selects, bits 11..7 mirror the transmitter/receiver state and
			// bits 5..1 hold the ASIC revision (2 on the B).
			uint16_t v = (m_net_diag & 0xf000) | (2 << 1);
			if (m_tx_enabled)
				v |= 0x0800; // TX enabled
			if (m_rx_enabled)
				v |= 0x0400; // RX enabled
			if (m_tx_pending)
				v |= 0x0200; // TX transmitting
			if (m_tx_reset_required)
				v |= 0x0100; // jabber/underrun: TX Reset required
			if (m_stats_enabled)
				v |= 0x0080; // statistics enabled
			return v;
		}
		case 0x08: return m_eth_status;
		case 0x0a:
		{
			// Media Type and Status: bit15 TP enabled (reflects the XCVR
			// select in Address Config), bit13 reserved-always-1, bit11
			// Valid link beat detected - which only reports a link once TPO
			// recovery has completed, a real backend is present, and the
			// driver has enabled link beat (bit7).
			uint16_t v = 0x2000;
			if (tpo_selected())
				v |= 0x8000;
			v |= m_media_status & 0x00cc;
			if (m_activated && tpo_selected() && BIT(m_media_status, 7) &&
					(m_link_state == LINK_READY) && (get_interface() >= 0))
				v |= 0x0800;
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
			// Changing only the I/O base must not disturb an in-progress
			// recovery.  Selecting another transceiver, however, makes the TPO
			// state invalid; selecting TPO later starts recovery as soon as all
			// other prerequisites are already present.
			if (!tpo_selected())
				reset_link_recovery();
			else
				maybe_start_link_recovery();
			break;
		case 0x08:
			m_resource_config = data;
			break;
		case 0x0a:
			eeprom_command(data);
			break;
		case 0x0c:
			if (!m_eeprom_busy)
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
		case 0x04:
			break; // FIFO Diagnostic: only the (B-revision reserved) BIST bits are writable
		case 0x06:
			// bits 15..12 are the writable loopback select; everything else
			// is read-only status derived at read time.  Bit 0 (test
			// low-voltage detector) must stay zero outside ASIC testing.
			m_net_diag = data & 0xf000;
			set_loopback((data & 0xf000) != 0);
			break;
		case 0x0a:
			// bit7 link beat enable, bit6 jabber enable, bit3 SQE statistics
			// enable, bit2 CRC strip disable; the rest is read-only status.
			m_media_status = (m_media_status & ~0x00cc) | (data & 0x00cc);
			maybe_start_link_recovery();
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

	// Execution times per tech reference 7.22: read 162us, write/erase 11ms,
	// erase-all 11ms, EWEN/EWDS 60us.
	if ((cmd & 0xc0) == 0x80)
		start_cip(OP_EEPROM_READ, addr, attotime::from_usec(162));
	else if ((cmd & 0xc0) == 0x40)
		start_cip(OP_EEPROM_WRITE, addr, attotime::from_msec(11));
	else if ((cmd & 0xc0) == 0xc0)
		start_cip(OP_EEPROM_ERASE, addr, attotime::from_msec(11));
	else
	{
		switch (cmd & 0x30)
		{
		case 0x00: start_cip(OP_EEPROM_EWDS, 0, attotime::from_usec(60)); break;
		case 0x20: start_cip(OP_EEPROM_ERASE_ALL, 0, attotime::from_msec(11)); break;
		case 0x30: start_cip(OP_EEPROM_EWEN, 0, attotime::from_usec(60)); break;
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

	switch (code)
	{
	case CMD_GLOBAL_RESET:
		global_reset();
		return;

	case CMD_RX_DISCARD:
		if (m_rx_count == 0)
			return;
		start_cip(OP_RX_DISCARD, 0, attotime::from_usec(5));
		return;

	case CMD_RX_RESET:
		// Emptying the RX FIFO and restoring the documented defaults takes
		// the receive datapath out of service for a while: apply the state
		// change when the CIP window closes, not at issue time.
		start_cip(OP_RX_RESET, param, attotime::from_usec(10));
		return;

	case CMD_TX_RESET:
		start_cip(OP_TX_RESET, param, attotime::from_usec(10));
		return;

	default:
		break;
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
	case CMD_TX_ENABLE:
		// "If the error was a jabber or an underrun, then a TX Reset
		// command is required before the TX Enable can be issued."
		if (m_tx_reset_required)
		{
			LOGCMD("3c509b: TxEnable refused, TX Reset required after jabber/underrun\n");
			break;
		}
		m_tx_enabled = true;
		tx_kick(); // resume any entries queued before a TxDisable
		break;
	case CMD_TX_DISABLE:
		m_tx_enabled = false;
		break;
	case CMD_FAKE_INTR:
		m_status |= ST_INT_REQUESTED;
		// Documented way to start the Window 1 Timer for general
		// measurements (tech reference 6.22), which works even when the
		// interrupt itself is masked off.
		timer_restart();
		break;
	case CMD_ACK_INTR:
		m_status &= ~(param & ST_ACK_MASK);
		if (param & ST_TX_AVAILABLE)
		{
			// "turns off the TX Available bit and resets the TX Available
			// threshold to its disabled value" (tech reference 6.8)
			m_tx_avail_thresh = THRESH_DISABLED;
		}
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

	// No published exact duration for these operations; hold CIP briefly so
	// polling software observes a real (if short) busy window.
	start_cip(OP_GENERIC, 0, attotime::from_usec(2));
}


// ---------------------------------------------------------------------
// TX FIFO and TX Status
// ---------------------------------------------------------------------

void isa8_3c509b_device::fifo_write_byte(uint8_t data)
{
	// The FIFO accepts data regardless of the transmitter's enable state:
	// "If the error occurred while the packet was still being copied to the
	// adapter, the host can continue to copy the packet to the adapter,
	// since the transmitter is disabled." (tech reference 6.19)
	const bool space = ((uint32_t(m_txq_used) + m_tx_buf_len) < SRAM_PARTITION_BYTES) &&
			(m_tx_buf_len < std::size(m_tx_buf));
	if (space)
	{
		m_tx_buf[m_tx_buf_len] = data;
	}
	else if (!(m_tx_error & TXS_UNDERRUN))
	{
		// TX partition exhausted: the driver wrote past TX Free.  This is
		// the documented Adapter Failure "transmit overrun" cause, reported
		// through FIFO Diagnostic bit 10 and completed as an underrun in
		// the TX Status stack; recovery requires a TX Reset.
		m_tx_error |= TXS_UNDERRUN;
		m_tx_enabled = false;
		m_tx_reset_required = true;
		adapter_failure(FD_TX_OVERRUN);
	}

	// The byte counter advances either way, so a frame whose copy overran
	// still reaches its documented length and completes with an error
	// instead of stalling the assembly buffer forever.
	m_tx_buf_len++;

	if (m_tx_buf_len == 2)
		m_tx_len = m_tx_buf[0] | (uint16_t(m_tx_buf[1]) << 8);

	if (m_tx_buf_len == 4)
	{
		// preamble word0 = length/flags, word1 = reserved; frame data is
		// padded up to the next DWORD boundary after the preamble.
		const uint16_t len = m_tx_len & 0x07ff;
		m_tx_expected = 4 + ((len + 3) & ~uint16_t(3));
		if (len > 1514)
		{
			// oversize frame: the TP transceiver reports it as a jabber
			// error, which also requires a TX Reset to recover
			m_tx_error |= TXS_JABBER;
			m_tx_enabled = false;
			m_tx_reset_required = true;
		}
	}

	if (m_tx_expected && (m_tx_buf_len >= m_tx_expected))
	{
		const bool notify = BIT(m_tx_len, 15);

		if (m_tx_error)
		{
			// errored frames are discarded rather than queued, and always
			// push a TX Status entry (an error is signalled to the host
			// whether or not the preamble asked for a notification)
			LOGNET("3c509b: tx aborted, error=%02x\n", m_tx_error);
			push_tx_status(TXS_COMPLETE | (notify ? TXS_INT_REQ : 0) | m_tx_error);
			m_tx_error = 0;
		}
		else
		{
			// completed entry: append it verbatim to the TX queue ring and
			// (if idle) put it on the wire
			for (uint16_t i = 0; i < m_tx_buf_len; i++)
				m_txq[(uint32_t(m_txq_head) + m_txq_used + i) % SRAM_PARTITION_BYTES] = m_tx_buf[i];
			m_txq_used += m_tx_buf_len;
		}

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

	// Before the physical TPO path is ready, consume the frame for its normal
	// 10 Mbit/s wire time but never hand it to the host backend and never
	// replay it after recovery.  Internal loopback is an ASIC path and does
	// not depend on the external transceiver.
	if (!m_loopback_control && (m_link_state != LINK_READY))
	{
		LOGNET("3c509b: tx lost while TPO link is not ready\n");
		m_txdrop_timer->adjust(attotime::from_ticks(frame_len, m_bandwidth));
	}
	else
	{
		// send() schedules send_complete_cb() at the configured bandwidth; if
		// loopback is active it also synchronously invokes recv_start_cb() and
		// schedules recv_complete_cb() the same way.
		send(frame, frame_len);
	}
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::txkick_timer_done)
{
	tx_kick();
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::txdrop_timer_done)
{
	send_complete_cb(0);
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

	// A successful transmit is only signalled when the preamble asked for
	// it, and the completed entry keeps the Interrupt Requested bit so the
	// driver can match it against its own queue (tech reference 6.19).
	if (m_tx_pending_notify)
		push_tx_status(TXS_COMPLETE | TXS_INT_REQ);

	if (m_stats_enabled)
	{
		bump_stat(6); // frames transmitted OK
		bump_stat_word(m_stat_tx_bytes, m_tx_pending_len);
	}

	update_tx_available();
	update_irq();

	// next queued frame goes out after the standard 10 Mbps interframe
	// gap (96 bit times = 9.6us)
	if (m_txq_used)
		m_txkick_timer->adjust(attotime::from_nsec(9600));
}


void isa8_3c509b_device::push_tx_status(uint8_t status)
{
	if (m_tx_status_count < TX_STATUS_DEPTH)
		m_tx_status_stack[m_tx_status_count++] = status;

	if (m_tx_status_count >= TX_STATUS_DEPTH)
	{
		// "The TX Status Overflow bit, if set, indicates that the TX Status
		// stack is full, and as a result the transmitter has been disabled.
		// Writing the TX Status register clears this condition; no other
		// action is required."  Disabling the transmitter as the stack fills
		// is what keeps the next completion from being lost.
		m_tx_status_overflow = true;
		m_tx_enabled = false;
	}

	m_status |= ST_TX_COMPLETE;
	update_irq();
}


void isa8_3c509b_device::pop_tx_status()
{
	// TX Status: a write pops exactly one (already-read) head entry.
	if (m_tx_status_count == 0)
		return;

	m_tx_status_count--;
	for (uint8_t i = 0; i < m_tx_status_count; i++)
		m_tx_status_stack[i] = m_tx_status_stack[i + 1];
	if (m_tx_status_count == 0)
		m_status &= ~ST_TX_COMPLETE;

	if (m_tx_status_overflow)
	{
		// popping an entry makes room again and re-enables the transmitter,
		// unless a jabber/underrun still demands a TX Reset
		m_tx_status_overflow = false;
		if (!m_tx_reset_required)
		{
			m_tx_enabled = true;
			tx_kick();
		}
	}

	update_irq();
}


void isa8_3c509b_device::tx_reset()
{
	// "empties the TX FIFO, disables the Ethernet controller transmitter,
	// and resets the TX Available and TX Start thresholds to default values.
	// If a packet is currently being transmitted, the transmit will be
	// aborted." (tech reference 6.7); the TX Status stack is part of the
	// network transmit logic that the command resets.
	m_tx_enabled = false;
	m_tx_buf_len = 0;
	m_tx_len = 0;
	m_tx_expected = 0;
	m_tx_error = 0;
	m_txq_head = 0;
	m_txq_used = 0;
	m_txkick_timer->reset();
	m_txdrop_timer->reset();
	m_tx_pending = false; // an in-flight send_complete_cb is now stale
	m_tx_pending_len = 0;
	m_tx_pending_notify = false;
	m_tx_reset_required = false;
	m_tx_status_overflow = false;
	m_tx_status_count = 0;
	m_tx_avail_thresh = THRESH_DISABLED;
	m_tx_start_thresh = THRESH_DISABLED;
	m_status &= ~(ST_TX_COMPLETE | ST_TX_AVAILABLE);

	m_fifo_diag &= ~FD_TX_OVERRUN;
	if (!(m_fifo_diag & FD_STICKY))
		m_status &= ~ST_ADAPTER_FAILURE;

	LOGCMD("3c509b: tx reset complete\n");
}


void isa8_3c509b_device::update_tx_available()
{
	// TX Available is a latched event: it asserts when the DWORD-rounded TX
	// free space exceeds the programmed threshold, and stays set until it is
	// acknowledged (which also returns the threshold to disabled).
	if (m_tx_avail_thresh > THRESH_MAX_ACTIVE)
		return; // threshold disabled
	const uint16_t free = (SRAM_PARTITION_BYTES - tx_used_bytes()) & ~uint16_t(3);
	if (free > m_tx_avail_thresh)
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
	// "This bit indicates that one or more of the statistics counters are
	// nearing an overrun condition (typically half a counter's maximum
	// value)." (tech reference 6.15)
	if (m_stat_byte[index] >= 0x80)
	{
		m_status |= ST_STATS_FULL;
		update_irq();
	}
}


void isa8_3c509b_device::bump_stat_word(uint16_t &counter, uint16_t amount)
{
	const uint32_t total = uint32_t(counter) + amount;
	counter = uint16_t(std::min<uint32_t>(total, 0xffff));
	if (counter >= 0x8000)
	{
		m_status |= ST_STATS_FULL;
		update_irq();
	}
}


void isa8_3c509b_device::stats_full_recheck()
{
	// Update Statistics is not in the AckIntr mask; it clears once the host
	// has read out the statistics (reading clears each counter).
	if (!(m_status & ST_STATS_FULL))
		return;
	for (uint8_t v : m_stat_byte)
		if (v >= 0x80)
			return;
	if ((m_stat_tx_bytes >= 0x8000) || (m_stat_rx_bytes >= 0x8000))
		return;
	m_status &= ~ST_STATS_FULL;
	update_irq();
}


void isa8_3c509b_device::adapter_failure(uint16_t diag_bit)
{
	// "An error occurred that the adapter was unable to recover from ...
	// Possible causes are described in the FIFO Diagnostic Port register.
	// The host must issue the appropriate Reset command to clear this
	// condition and recover." (tech reference 6.14)
	m_fifo_diag |= diag_bit;
	m_status |= ST_ADAPTER_FAILURE;
	update_irq();
}


// ---------------------------------------------------------------------
// RX FIFO and RX Discard
// ---------------------------------------------------------------------

uint8_t isa8_3c509b_device::fifo_read_byte()
{
	if (m_rx_count == 0)
	{
		// "Receive underrun (host reads data that is not yet available or
		// attempts to read the RX FIFO beyond the pad bytes)"
		adapter_failure(FD_RX_UNDERRUN);
		return 0x00;
	}

	const uint16_t slot = m_rx_head;
	const uint16_t len = m_rx_len[slot];
	// data beyond the frame length up to the DWORD-padded boundary reads
	// back as zero; reading the head packet's data never removes it, only
	// an explicit RX_DISCARD command advances the ring.
	const uint16_t padded = (len + 3) & ~uint16_t(3);
	if (m_rx_cursor >= padded)
	{
		adapter_failure(FD_RX_UNDERRUN);
		return 0x00;
	}

	const uint8_t v = (m_rx_cursor < len)
			? m_rxq[(uint32_t(m_rxq_head) + m_rx_cursor) % SRAM_PARTITION_BYTES]
			: 0x00;
	m_rx_cursor++;
	return v;
}


void isa8_3c509b_device::rx_discard()
{
	if (m_rx_count == 0)
		return;

	const uint16_t padded = (m_rx_len[m_rx_head] + 3) & ~uint16_t(3);
	m_rxq_head = uint16_t((uint32_t(m_rxq_head) + padded) % SRAM_PARTITION_BYTES);
	m_rxq_used -= std::min<uint16_t>(padded, m_rxq_used);

	m_rx_head = (m_rx_head + 1) % RX_SLOTS;
	m_rx_count--;
	m_rx_cursor = 0;

	if (m_rx_count == 0)
		m_status &= ~ST_RX_COMPLETE;
	else
		m_status |= ST_RX_COMPLETE; // next packet is now visible as head

	update_irq();
}


void isa8_3c509b_device::rx_reset()
{
	// "empties the RX FIFO, disables the Ethernet controller [receiver],
	// resets the RX Filter and RX Early threshold to defaults, and aborts
	// reception if a packet is currently being received." (tech reference
	// 6.6)
	m_rx_enabled = false;
	m_rx_filter = 0;
	m_rx_early_thresh = THRESH_DISABLED;
	m_rxq_head = 0;
	m_rxq_used = 0;
	m_rx_head = 0;
	m_rx_count = 0;
	m_rx_cursor = 0;
	m_rx_receiving = false;
	m_rx_stage_len = 0;
	m_status &= ~(ST_RX_COMPLETE | ST_RX_EARLY);

	m_fifo_diag &= ~FD_RX_UNDERRUN;
	if (!(m_fifo_diag & FD_STICKY))
		m_status &= ~ST_ADAPTER_FAILURE;

	LOGCMD("3c509b: rx reset complete\n");
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
	if (!m_loopback_control && (m_link_state != LINK_READY))
		return 0; // external traffic is suppressed during TPO recovery
	if (length < 6)
		return 0;
	if (!accept_filter(buf, length))
		return 0;

	// A real Ethernet MAC supplies padding up to the 60-byte minimum before
	// the FCS.  Host-side virtual Ethernet interfaces do not necessarily do
	// that: macOS feth, for example, delivers its otherwise valid 42-byte ARP
	// replies through pcap without padding.  Treat that omission as a backend
	// representation detail for external traffic.  Internal loopback keeps
	// the exact length so the controller's runt/error path remains testable.
	const uint16_t wire_length = uint16_t(ethernet_wire_length(m_loopback_control, length));

	// oversize frames keep being received until the 1,792-byte cut-off, at
	// which point the remainder is discarded (tech reference 6.17)
	const uint16_t keep = uint16_t(std::min<int>(wire_length, int(RX_MAX_BYTES)));
	const uint16_t padded = (keep + 3) & ~uint16_t(3);
	if ((m_rx_count >= RX_SLOTS) ||
			((uint32_t(m_rxq_used) + padded) > SRAM_PARTITION_BYTES))
	{
		bump_stat(5); // RX overruns
		return 0; // RX partition/descriptor ring full, drop (FIFO Diagnostic
				  // reports RX Overrun for as long as the condition holds)
	}

	const uint16_t captured = std::min<uint16_t>(uint16_t(length), keep);
	std::copy_n(buf, captured, m_rx_stage_data);
	std::fill(m_rx_stage_data + captured, m_rx_stage_data + keep, uint8_t(0));
	m_rx_stage_len = keep;

	uint16_t status = keep & 0x07ff;
	if (wire_length < 60)
		status |= 0x5800; // Error + Runt Packet Error
	else if (wire_length > 1514)
		status |= 0x4800; // Error + Oversize Packet Error
	m_rx_stage_status = status;

	m_rx_receiving = true;

	// RX Early fires while the packet is still on the wire, once the byte
	// count passes the threshold; a threshold above the longest receivable
	// packet disables the event entirely.
	if ((m_rx_early_thresh <= THRESH_MAX_ACTIVE) && (keep > m_rx_early_thresh))
	{
		m_status |= ST_RX_EARLY;
		update_irq();
	}

	LOGNET("3c509b: rx staged len=%d\n", length);
	return length; // nonzero => accepted; recv_complete_cb() commits it after the simulated transfer delay
}


void isa8_3c509b_device::recv_complete_cb(int result)
{
	m_rx_receiving = false;

	if (result <= 0)
		return;
	// the receiver may have been reset or disabled while the packet was
	// still "on the wire"; a reset must not resurrect a stale staged frame
	if (!m_rx_enabled || (m_rx_stage_len == 0))
		return;

	const uint16_t padded = (m_rx_stage_len + 3) & ~uint16_t(3);
	if ((m_rx_count >= RX_SLOTS) ||
			((uint32_t(m_rxq_used) + padded) > SRAM_PARTITION_BYTES))
		return;

	const uint16_t slot = (m_rx_head + m_rx_count) % RX_SLOTS;
	const uint32_t tail = (uint32_t(m_rxq_head) + m_rxq_used) % SRAM_PARTITION_BYTES;
	for (uint16_t i = 0; i < padded; i++)
		m_rxq[(tail + i) % SRAM_PARTITION_BYTES] = (i < m_rx_stage_len) ? m_rx_stage_data[i] : 0x00;
	m_rxq_used += padded;

	m_rx_len[slot] = m_rx_stage_len;
	m_rx_status[slot] = m_rx_stage_status;
	m_rx_count++;
	m_rx_stage_len = 0;

	if (m_stats_enabled)
	{
		bump_stat(7); // frames received OK
		bump_stat_word(m_stat_rx_bytes, m_rx_len[slot]);
	}

	// "The RX Complete bit masks the RX Early bit.  Whenever RX Complete is
	// set, RX Early will be clear." (tech reference 6.10)
	m_status |= ST_RX_COMPLETE;
	m_status &= ~ST_RX_EARLY;

	update_irq();

	LOGNET("3c509b: rx committed len=%u slot=%u\n", m_rx_len[slot], slot);
}
