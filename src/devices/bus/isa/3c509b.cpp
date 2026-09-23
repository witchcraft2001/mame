// license:BSD-3-Clause
// copyright-holders:Dmitry Mikhalchenkov
/***************************************************************************

    3Com 3C509B-TPO EtherLink III ISA Ethernet adapter

    Reference: EtherLink III Parallel Tasking ISA, EISA, Micro Channel,
    and PCMCIA Adapter Drivers Technical Reference, 09-0398-002B.
    This model targets 8-bit ISA hosts; 16-bit registers are accessed as
    low/high byte pairs (section 6).

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

#define VERBOSE 0

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

// Bits 1..10: Interrupt Latch is exempt, while CIP and window are outside the argument.
constexpr uint16_t ST_ZERO_MASKABLE    = 0x07fe;

// Values above the longest packet disable threshold events; 2,044 is the reset default.
constexpr uint16_t THRESH_DISABLED  = 2044;

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
constexpr uint8_t TXS_OVERFLOW = 0x04;

// RX filter bits (Window 5, offset 0x08)
constexpr uint8_t RXF_INDIVIDUAL = 0x01;
constexpr uint8_t RXF_GROUP      = 0x02;
constexpr uint8_t RXF_BROADCAST  = 0x04;
constexpr uint8_t RXF_PROMISC    = 0x08;

// 3C509B-TPO assembly 03-0020-002, revision 3; per-card fields are generated below.
constexpr std::array<uint16_t, 0x40> DEFAULT_EEPROM =
{
	0x0000, 0x0000, 0x0000, 0x9550, 0xb434, 0x0041, 0x4a41, 0x6d50,
	0x0010, 0x3000, 0x0000, 0x0000, 0x0000, 0x1310, 0x0000, 0x0000,
	0x2083, 0x0000, 0x0000, 0x0004, 0x0001, 0x0000, 0x0000, 0x0000,
	0x6d50, 0x9550, 0x0000, 0x0000, 0x0a00, 0x1010, 0x1982, 0x3300,
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
};

} // anonymous namespace


DEFINE_DEVICE_TYPE(ISA8_3C509B, isa8_3c509b_device, "3c509b", "3Com EtherLink III (3C509B-TPO)")


isa8_3c509b_device::isa8_3c509b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISA8_3C509B, tag, owner, clock),
	device_isa8_card_interface(mconfig, *this),
	device_network_interface(mconfig, *this, 10), // 10 Mib/s
	m_config(*this, "CONFIG"),
	m_cmd_timer(nullptr),
	m_txkick_timer(nullptr),
	m_eeprom{ },
	m_eeprom_data(0),
	m_eeprom_wren(false),
	m_eeprom_initialized(false),
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
	m_installed_iobase(0),
	m_irq(3),
	m_irq_line_state(false),
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

	// The configurable 3Com ID port is fixed to the conventional 0x110.
	m_isa->install_device(0x0110, 0x0110,
			read8sm_delegate(*this, FUNC(isa8_3c509b_device::id_r)),
			write8sm_delegate(*this, FUNC(isa8_3c509b_device::id_w)));

	m_cmd_timer = timer_alloc(FUNC(isa8_3c509b_device::cmd_timer_done), this);
	m_txkick_timer = timer_alloc(FUNC(isa8_3c509b_device::txkick_timer_done), this);

	// Network configuration loaded before device_reset may override this MAC.
	uint32_t mac_tail = 0x509b3c;
	for (char const *p = tag(); *p != '\0'; p++)
		mac_tail = (mac_tail * 33) ^ uint8_t(*p);
	uint8_t mac[6] = { 0x02, 0x60, 0x8c, 0x00, 0x00, 0x00 };
	put_u24be(&mac[3], mac_tail);
	set_mac(mac);

	save_item(NAME(m_eeprom));
	save_item(NAME(m_eeprom_data));
	save_item(NAME(m_eeprom_wren));
	save_item(NAME(m_eeprom_initialized));
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

	if (!m_eeprom_initialized)
	{
		// EEPROM contents survive subsequent soft resets.
		build_eeprom();
		m_eeprom_initialized = true;
	}
	global_reset_finish();
}


void isa8_3c509b_device::device_post_load()
{
	if (m_activated)
		install_io(m_iobase);
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
	PORT_CONFSETTING(0x00, "ID sequence")
	PORT_CONFSETTING(0x80, "Automatic at reset")
INPUT_PORTS_END


ioport_constructor isa8_3c509b_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(3c509b);
}


// ---------------------------------------------------------------------
// EEPROM construction
// ---------------------------------------------------------------------

void isa8_3c509b_device::build_eeprom()
{
	m_eeprom = DEFAULT_EEPROM;

	auto const &mac = get_mac();
	for (unsigned i = 0; i < 3; i++)
		m_eeprom[i] = get_u16be(&mac[2 * i]);

	const uint8_t config = m_config->read();
	static constexpr uint8_t IRQ_SELECT[4] = { 3, 5, 7, 9 };
	m_eeprom[0x08] = config & 0x1f; // XCVR select = 00 (TPO)
	m_eeprom[0x09] = uint16_t(IRQ_SELECT[(config >> 5) & 3]) << 12;

	m_eeprom[0x0a] = m_eeprom[0x00]; // OEM node address
	m_eeprom[0x0b] = m_eeprom[0x01];
	m_eeprom[0x0c] = m_eeprom[0x02];

	// Derive the PnP serial number and checksum from the configured address.
	m_eeprom[0x1a] = m_eeprom[0x02];
	m_eeprom[0x1b] = m_eeprom[0x01];
	m_eeprom[0x1c] = (m_eeprom[0x1c] & 0xff00) | pnp_serial_checksum(m_eeprom);

	auto const xor_words = [this] (unsigned first, unsigned last)
	{
		uint8_t result = 0;
		for (unsigned word = first; word <= last; word++)
			result ^= uint8_t(m_eeprom[word]) ^ uint8_t(m_eeprom[word] >> 8);
		return result;
	};

	const uint8_t primary_config = xor_words(0x08, 0x09) ^ xor_words(0x0d, 0x0d);
	m_eeprom[0x0f] = (uint16_t(xor_words(0x00, 0x0e) ^ primary_config) << 8) | primary_config;
	m_eeprom[0x17] = (uint16_t(xor_words(0x10, 0x12) ^ xor_words(0x18, 0x3f)) << 8) | xor_words(0x13, 0x16);
}


uint8_t isa8_3c509b_device::pnp_serial_checksum(std::array<uint16_t, 0x40> const &eeprom)
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


void isa8_3c509b_device::activate()
{
	if ((m_addr_config & 0x1f) == 0x1f)
	{
		LOGID("activate refused, address config selects EISA (unsupported on ISA8)\n");
		return;
	}

	deactivate();

	install_io(0x0200 + uint16_t(m_addr_config & 0x1f) * 0x10);
	m_irq = (m_resource_config >> 12) & 0x0f;

	m_activated = true;
	update_irq();

	LOGID("activated io=%04x irq=%u\n", m_iobase, m_irq);
}


void isa8_3c509b_device::install_io(uint16_t base)
{
	m_iobase = base;
	if (m_installed_iobase == base)
		return;

	if (m_installed_iobase)
		m_isa->unmap_device(m_installed_iobase, m_installed_iobase + 0x0f);

	m_isa->install_device(base, base + 0x0f,
			read8sm_delegate(*this, FUNC(isa8_3c509b_device::op_r)),
			write8sm_delegate(*this, FUNC(isa8_3c509b_device::op_w)));
	m_installed_iobase = base;
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
	// Ignore operating-window and ID-port access during the 310 us EEPROM reread (section 7.4).
	deactivate();
	start_cip(OP_GLOBAL_RESET, 0, attotime::from_usec(310));
}


void isa8_3c509b_device::global_reset_finish()
{
	deactivate();

	m_window = 0;
	m_eeprom_wren = false;
	// ENA is writable, RST is a trigger, and bits 13..8 describe TPO capabilities (section 7.15).
	m_config_control = 0x0f00;
	m_addr_config = m_eeprom[0x08];
	m_resource_config = m_eeprom[0x09];

	m_status = 0;
	m_int_mask = 0;
	// Read Zero mask defaults to zero (section 6.9).
	m_read_zero_mask = 0;
	m_rx_filter = 0;
	m_rx_early_thresh = THRESH_DISABLED;
	m_tx_avail_thresh = THRESH_DISABLED;
	m_tx_start_thresh = THRESH_DISABLED;
	m_rx_enabled = false;
	m_tx_enabled = false;
	// Internal Configuration is seeded from EEPROM words 0x12 and 0x13.
	m_internal_config = uint32_t(m_eeprom[0x12]) | (uint32_t(m_eeprom[0x13]) << 16);

	timer_restart();

	m_tx_buf_len = 0;
	m_tx_len = 0;
	m_tx_expected = 0;
	m_tx_error = 0;
	m_txq_head = 0;
	m_txq_used = 0;
	cancel_send();
	m_txkick_timer->reset();
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
	cancel_receive();
	m_rx_receiving = false;
	m_rx_stage_len = 0;

	// EEPROM words are big-endian; Window 2 exposes Address0 on the low byte.
	for (unsigned i = 0; i < 3; i++)
		put_u16be(&m_station[2 * i], m_eeprom[i]);
	set_mac(m_station);

	m_fifo_diag = 0;
	m_net_diag = 0;
	// Link beat and jabber detection default to disabled (section 6.27).
	m_media_status = 0;
	set_loopback(false);

	std::fill(std::begin(m_stat_byte), std::end(m_stat_byte), uint8_t(0));
	m_stat_tx_bytes = 0;
	m_stat_rx_bytes = 0;
	m_stats_enabled = false;

	m_rd_latch_valid = false;
	m_wr_latch_valid = false;

	m_id_state = ID_IDLE;
	m_id_lfsr = 0xff;
	m_id_pos = 0;
	m_id_tag = 0;

	LOGID("global reset complete\n");

	if (BIT(m_config->read(), 7))
		activate();

	update_irq();
}


// ---------------------------------------------------------------------
// Status / IRQ
// ---------------------------------------------------------------------

uint16_t isa8_3c509b_device::masked_status() const
{
	// Clear Read Zero bits hide status, except Interrupt Latch and CIP.
	return m_status & ~(ST_ZERO_MASKABLE & ~m_read_zero_mask);
}


void isa8_3c509b_device::update_irq()
{
	// Apply Read Zero before the interrupt mask (section 6.9).
	const bool was_latched = (m_status & ST_INT_LATCH) != 0;
	const uint16_t triggers = masked_status() & m_int_mask & 0x00fe;
	if (triggers)
		m_status |= ST_INT_LATCH;
	// Interrupt Latch stays set until AckIntr or reset.

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
	LOGIRQ("irq%u state=%d\n", m_irq, state);

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
	// Free-running 10 MHz/32 timer saturating at 255 (section 6.22).
	const uint64_t now = machine().time().as_ticks(TIMER_CLOCK);
	const uint64_t elapsed = now - m_timer_base;
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
	m_cmd_timer->adjust(delay);
}


bool isa8_3c509b_device::eeprom_busy() const
{
	return (m_pending_op >= OP_EEPROM_READ) && (m_pending_op <= OP_EEPROM_EWDS);
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::cmd_timer_done)
{
	const uint8_t op = m_pending_op;
	const uint16_t arg = m_pending_arg;
	m_pending_op = OP_NONE;
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
		// Write completion automatically disables further writes (section 7.22).
		m_eeprom_wren = false;
		break;
	case OP_EEPROM_ERASE:
		if (m_eeprom_wren)
			m_eeprom[arg & 0x3f] = 0xffff;
		m_eeprom_wren = false;
		break;
	case OP_EEPROM_ERASE_ALL:
		if (m_eeprom_wren)
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


uint8_t isa8_3c509b_device::id_r(offs_t)
{
	if ((m_pending_op == OP_GLOBAL_RESET) || (m_id_state != ID_CMD) || (m_id_tag != 0))
		return 0xff;

	// Contention drives EEPROM Data bit 15 onto D0 and shifts, holding while a read is in flight.
	const uint8_t result = 0xfe | BIT(m_eeprom_data, 15);
	if (eeprom_busy())
	{
		if (!machine().side_effects_disabled())
			LOGID("id port read while EEPROM busy, shifter held\n");
		return result;
	}
	if (!machine().side_effects_disabled())
		m_eeprom_data <<= 1;
	return result;
}


void isa8_3c509b_device::id_w(offs_t, uint8_t data)
{
	if (m_pending_op == OP_GLOBAL_RESET)
		return; // AUTOINIT in progress: ID sequence not yet accepted

	invalidate_latches();

	if (data == 0x00)
	{
		if (m_id_state == ID_IDLE)
		{
			m_id_state = ID_ARMED;
			LOGID("ID port armed\n");
			return;
		}

		m_id_lfsr = 0xff;
		m_id_pos = 0;
		m_id_state = ID_SEQ;
		return;
	}

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
				LOGID("id sequence complete, entering command mode\n");
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

	default:
		break; // stray byte before the two-zero handshake: ignored
	}
}


void isa8_3c509b_device::id_command(uint8_t cmd)
{
	if (m_status & ST_CMD_IN_PROGRESS)
	{
		LOGID("id command %02x ignored, CIP busy\n", cmd);
		return;
	}

	LOGID("id command %02x\n", cmd);

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
	else
	{
		if (cmd != 0xff)
			m_addr_config = (m_addr_config & ~0x1f) | (cmd & 0x1f);
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
	if (!m_activated)
		return 0xff;
	const uint8_t reg = uint8_t(offset);
	const bool side_effects = !machine().side_effects_disabled();

	// Byte-wide registers bypass the word latch.
	switch ((m_window << 4) | reg)
	{
	case 0x10:
	case 0x12:
		if (side_effects)
			invalidate_latches();
		return fifo_read_byte();

	case 0x11:
	case 0x13:
		if (side_effects)
			invalidate_latches();
		return 0xff;

	case 0x1a:
		if (side_effects)
			invalidate_latches();
		return timer_value();

	case 0x1b:
		if (side_effects)
			invalidate_latches();
		// Reading peeks the head; Overflow remains set until a write clears it.
		return (m_tx_status_count ? m_tx_status_stack[0] : 0x00) |
				(m_tx_status_overflow ? TXS_OVERFLOW : 0x00);

	case 0x35:
		if (side_effects)
			invalidate_latches();
		return 0x00; // ROM Control: no boot ROM fitted

	default:
		break;
	}

	if ((m_window == 6) && (reg <= 0x08))
	{
		if (side_effects)
			invalidate_latches();
		const uint8_t v = m_stat_byte[reg];
		if (side_effects)
		{
			m_stat_byte[reg] = 0;
			stats_full_recheck();
		}
		return v;
	}

	if (side_effects)
		LOGIO("read w%u reg=%02x\n", m_window, reg);

	if (!(reg & 1))
	{
		const uint16_t word = reg_read(reg);
		if (side_effects)
		{
			m_rd_latch_valid = true;
			m_rd_latch_off = reg;
			m_rd_latch_hi = uint8_t(word >> 8);
		}
		return uint8_t(word);
	}

	const uint8_t even = reg & ~1;
	if (side_effects && m_rd_latch_valid && (m_rd_latch_off == even))
	{
		m_rd_latch_valid = false;
		return m_rd_latch_hi;
	}
	return uint8_t(reg_read(even) >> 8);
}


void isa8_3c509b_device::op_w(offs_t offset, uint8_t data)
{
	if (!m_activated)
		return;
	const uint8_t reg = uint8_t(offset);

	switch ((m_window << 4) | reg)
	{
	case 0x10:
	case 0x12:
		invalidate_latches();
		fifo_write_byte(data);
		return;

	case 0x11:
	case 0x13:
	case 0x1a:
	case 0x35:
		invalidate_latches();
		return;

	case 0x1b:
		invalidate_latches();
		pop_tx_status();
		return;

	default:
		break;
	}

	LOGIO("write w%u reg=%02x data=%02x\n", m_window, reg, data);

	if (!(reg & 1))
	{
		m_wr_latch_valid = true;
		m_wr_latch_off = reg;
		m_wr_latch_lo = data;
		return;
	}

	const uint8_t even = reg & ~1;
	uint8_t lo = 0;
	if (m_wr_latch_valid && (m_wr_latch_off == even))
		lo = m_wr_latch_lo;
	m_wr_latch_valid = false;

	const uint16_t word = lo | (uint16_t(data) << 8);
	if (even == 0x0e)
		do_command(word);
	else
		reg_write(even, word);
}


// ---------------------------------------------------------------------
// Register windows
// ---------------------------------------------------------------------

uint16_t isa8_3c509b_device::tx_used_bytes() const
{
	// Include queued entries, the frame on the wire, and PIO-staged data.
	const uint32_t used = uint32_t(m_txq_used) + m_tx_buf_len;
	return uint16_t(std::min<uint32_t>(used, SRAM_PARTITION_BYTES));
}


bool isa8_3c509b_device::rx_fifo_full() const
{
	// Reserve space for one minimum frame and one descriptor (section 6.31).
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
		case 0x0a: return eeprom_busy() ? 0x8000 : 0x0000; // EBY
		case 0x0c: return m_eeprom_data;
		}
		break;

	case 1:
		switch (reg)
		{
		case 0x08:
		{
			if (m_rx_count == 0)
				return 0x8000; // incomplete
			// RX Bytes continues negative into padding as 11-bit two's complement (section 6.18).
			const int32_t remaining = int32_t(m_rx_len[m_rx_head]) - int32_t(m_rx_cursor);
			return (m_rx_status[m_rx_head] & 0xf800) | (uint16_t(remaining) & 0x07ff);
		}
		case 0x0c:
			// TX Free, rounded down to a DWORD (Window 1 semantics)
			return (SRAM_PARTITION_BYTES - tx_used_bytes()) & ~uint16_t(3);
		}
		break;

	case 2:
		switch (reg)
		{
		case 0x00:
		case 0x02:
		case 0x04: return get_u16le(&m_station[reg]);
		case 0x06: return 0x0000; // station address mask, unused
		}
		break;

	case 3:
		switch (reg)
		{
		case 0x00: return uint16_t(m_internal_config);
		case 0x02: return uint16_t(m_internal_config >> 16);
		case 0x0a: return SRAM_PARTITION_BYTES - m_rxq_used; // RX Free, exact byte count
		case 0x0c: return SRAM_PARTITION_BYTES - tx_used_bytes(); // TX Free, exact byte count
		}
		break;

	case 4:
		switch (reg)
		{
		case 0x04:
		{
			// RX Underrun and TX Overrun are sticky; other bits are derived.
			uint16_t v = m_fifo_diag & FD_STICKY;
			if (m_rx_receiving)
				v |= FD_RX_RECEIVING;
			if (rx_fifo_full())
				v |= FD_RX_OVERRUN;
			return v;
		}
		case 0x06:
		{
			// Bits 15..12 select loopback; bits 11..7 report state; bits 5..1 report revision 2.
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
		case 0x08: return 0;
		case 0x0a:
		{
			// Link beat requires TPO, the host enable bit, and a network backend.
			uint16_t v = 0x2000;
			if (tpo_selected())
				v |= 0x8000;
			v |= m_media_status & 0x00cc;
			if (tpo_selected() && BIT(m_media_status, 7) && has_net_device())
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
		case 0x0a: return read_stat_word(m_stat_rx_bytes);
		case 0x0c: return read_stat_word(m_stat_tx_bytes);
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
			if (!eeprom_busy())
				m_eeprom_data = data;
			break;
		}
		break;

	case 2:
		switch (reg)
		{
		case 0x00:
		case 0x02:
		case 0x04: put_u16le(&m_station[reg], data); break;
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
			// Only loopback-select bits 15..12 are writable.
			m_net_diag = data & 0xf000;
			set_loopback((data & 0xf000) != 0);
			break;
		case 0x0a:
			// Writable: link beat, jabber, SQE statistics and CRC strip controls.
			m_media_status = (m_media_status & ~0x00cc) | (data & 0x00cc);
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

	// Command durations are from section 7.22.
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
		LOGCMD("command %04x ignored, CIP busy\n", cmd);
		return;
	}

	const uint8_t code = uint8_t(cmd >> 11);
	const uint16_t param = cmd & 0x07ff;

	LOGCMD("command %u param=%03x\n", code, param);

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
		// Apply RX state changes when the CIP interval ends.
		start_cip(OP_RX_RESET, param, attotime::from_usec(10));
		return;
	case CMD_TX_RESET:
		start_cip(OP_TX_RESET, param, attotime::from_usec(10));
		return;
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
		// Jabber and underrun require TX Reset before TX Enable (section 6.19).
		if (m_tx_reset_required)
		{
			LOGCMD("TxEnable refused, TX Reset required after jabber/underrun\n");
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
		// FakeIntr starts the Window 1 Timer even when interrupts are masked (section 6.22).
		timer_restart();
		break;
	case CMD_ACK_INTR:
		m_status &= ~(param & ST_ACK_MASK);
		if (param & ST_TX_AVAILABLE)
		{
			// Acknowledging TX Available disables its threshold (section 6.8).
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
	default:
		break;
	}

	// Hold CIP briefly for commands without published timing.
	start_cip(OP_GENERIC, 0, attotime::from_usec(2));
}


// ---------------------------------------------------------------------
// TX FIFO and TX Status
// ---------------------------------------------------------------------

void isa8_3c509b_device::fifo_write_byte(uint8_t data)
{
	// FIFO data is accepted while the transmitter is disabled (section 6.19).
	const bool space = ((uint32_t(m_txq_used) + m_tx_buf_len) < SRAM_PARTITION_BYTES) &&
			(m_tx_buf_len < std::size(m_tx_buf));
	if (space)
	{
		m_tx_buf[m_tx_buf_len] = data;
	}
	else if (!(m_tx_error & TXS_UNDERRUN))
	{
		// Writing past TX Free reports TX overrun and requires TX Reset.
		m_tx_error |= TXS_UNDERRUN;
		m_tx_enabled = false;
		m_tx_reset_required = true;
		adapter_failure(FD_TX_OVERRUN);
	}

	// Continue counting so an overrun frame still reaches completion.
	m_tx_buf_len++;

	if (m_tx_buf_len == 2)
		m_tx_len = m_tx_buf[0] | (uint16_t(m_tx_buf[1]) << 8);

	if (m_tx_buf_len == 4)
	{
		// Preamble contains length/flags and a reserved word; data is DWORD-padded.
		const uint16_t len = m_tx_len & 0x07ff;
		m_tx_expected = 4 + ((len + 3) & ~uint16_t(3));
		if (len > 1514)
		{
			// Oversize frames report jabber and require TX Reset.
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
			// Errors always push status, regardless of the notification flag.
			LOGNET("tx aborted, error=%02x\n", m_tx_error);
			push_tx_status(TXS_COMPLETE | (notify ? TXS_INT_REQ : 0) | m_tx_error);
			m_tx_error = 0;
		}
		else
		{
			// Queue the completed entry and transmit it when idle.
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

	LOGNET("tx launch len=%u (padded=%u) notify=%d queued=%u\n", len, frame_len, notify, m_txq_used);

	m_tx_pending = true;
	m_tx_pending_len = frame_len;
	m_tx_pending_notify = notify;
	send(frame, frame_len);
}


TIMER_CALLBACK_MEMBER(isa8_3c509b_device::txkick_timer_done)
{
	tx_kick();
}


void isa8_3c509b_device::send_complete_cb(int)
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

	// Successful completion is reported only when requested in the preamble (section 6.19).
	if (m_tx_pending_notify)
		push_tx_status(TXS_COMPLETE | TXS_INT_REQ);

	bump_stat(6); // frames transmitted OK
	bump_stat_word(m_stat_tx_bytes, m_tx_pending_len);

	update_tx_available();
	update_irq();

	// Standard 10 Mbps interframe gap: 96 bit times.
	if (m_txq_used)
		m_txkick_timer->adjust(attotime::from_nsec(9600));
}


void isa8_3c509b_device::push_tx_status(uint8_t status)
{
	if (m_tx_status_count < TX_STATUS_DEPTH)
		m_tx_status_stack[m_tx_status_count++] = status;

	if (m_tx_status_count >= TX_STATUS_DEPTH)
	{
		// A full stack disables the transmitter until an entry is popped (section 6.18).
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
	std::copy_n(&m_tx_status_stack[1], m_tx_status_count, &m_tx_status_stack[0]);
	if (m_tx_status_count == 0)
		m_status &= ~ST_TX_COMPLETE;

	if (m_tx_status_overflow)
	{
		// Re-enable unless jabber or underrun still requires TX Reset.
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
	// Abort transmission and restore TX FIFO and thresholds (section 6.7).
	m_tx_enabled = false;
	m_tx_buf_len = 0;
	m_tx_len = 0;
	m_tx_expected = 0;
	m_tx_error = 0;
	m_txq_head = 0;
	m_txq_used = 0;
	cancel_send();
	m_txkick_timer->reset();
	m_tx_pending = false;
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

	LOGCMD("tx reset complete\n");
}


void isa8_3c509b_device::update_tx_available()
{
	// TX Available remains latched until acknowledged.
	if (m_tx_avail_thresh > MAX_PACKET_BYTES)
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
	// Statistics Full asserts when a counter reaches half its range (section 6.15).
	if (m_stat_byte[index] >= 0x80)
	{
		m_status |= ST_STATS_FULL;
		update_irq();
	}
}


void isa8_3c509b_device::bump_stat_word(uint16_t &counter, uint16_t amount)
{
	if (!m_stats_enabled)
		return;
	const uint32_t total = uint32_t(counter) + amount;
	counter = uint16_t(std::min<uint32_t>(total, 0xffff));
	if (counter >= 0x8000)
	{
		m_status |= ST_STATS_FULL;
		update_irq();
	}
}


uint16_t isa8_3c509b_device::read_stat_word(uint16_t &counter)
{
	const uint16_t value = counter;
	if (!machine().side_effects_disabled())
	{
		counter = 0;
		stats_full_recheck();
	}
	return value;
}


void isa8_3c509b_device::stats_full_recheck()
{
	// Statistics Full clears after all counters below the threshold are read.
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
	// Adapter Failure remains set until the corresponding reset (section 6.14).
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
		// Reading unavailable data reports RX underrun.
		if (!machine().side_effects_disabled())
			adapter_failure(FD_RX_UNDERRUN);
		return 0x00;
	}

	const uint16_t slot = m_rx_head;
	const uint16_t len = m_rx_len[slot];
	// Padding reads as zero; only RX_DISCARD advances the packet ring.
	const uint16_t padded = (len + 3) & ~uint16_t(3);
	if (m_rx_cursor >= padded)
	{
		if (!machine().side_effects_disabled())
			adapter_failure(FD_RX_UNDERRUN);
		return 0x00;
	}

	const uint8_t v = (m_rx_cursor < len)
			? m_rxq[(uint32_t(m_rxq_head) + m_rx_cursor) % SRAM_PARTITION_BYTES]
			: 0x00;
	if (!machine().side_effects_disabled())
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
	// Abort reception and restore RX FIFO, filter and threshold (section 6.6).
	m_rx_enabled = false;
	m_rx_filter = 0;
	m_rx_early_thresh = THRESH_DISABLED;
	m_rxq_head = 0;
	m_rxq_used = 0;
	m_rx_head = 0;
	m_rx_count = 0;
	m_rx_cursor = 0;
	cancel_receive();
	m_rx_receiving = false;
	m_rx_stage_len = 0;
	m_status &= ~(ST_RX_COMPLETE | ST_RX_EARLY);

	m_fifo_diag &= ~FD_RX_UNDERRUN;
	if (!(m_fifo_diag & FD_STICKY))
		m_status &= ~ST_ADAPTER_FAILURE;

	LOGCMD("rx reset complete\n");
}


bool isa8_3c509b_device::accept_filter(uint8_t const *buf) const
{
	if (m_rx_filter & RXF_PROMISC)
		return true;

	if (std::all_of(buf, buf + 6, [] (uint8_t byte) { return byte == 0xff; }))
		return (m_rx_filter & (RXF_BROADCAST | RXF_GROUP)) != 0; // group also accepts broadcast

	if (buf[0] & 0x01)
		return (m_rx_filter & RXF_GROUP) != 0;

	if (m_rx_filter & RXF_INDIVIDUAL)
		return std::equal(buf, buf + 6, m_station);

	return false;
}


int isa8_3c509b_device::recv_start_cb(uint8_t *buf, int length)
{
	if (!m_activated || !m_rx_enabled || m_rx_receiving || (length < 6))
		return 0;
	if (!accept_filter(buf))
		return 0;

	// Virtual interfaces may omit Ethernet padding; pad external traffic and keep loopback exact.
	const int wire_length = (!m_loopback_control && (length >= 14) && (length < 60)) ? 60 : length;

	// Oversize frames are truncated at 1,792 bytes (section 6.17).
	const uint16_t keep = uint16_t(std::min(wire_length, int(MAX_PACKET_BYTES)));
	const uint16_t padded = (keep + 3) & ~uint16_t(3);
	if ((m_rx_count >= RX_SLOTS) ||
			((uint32_t(m_rxq_used) + padded) > SRAM_PARTITION_BYTES))
	{
		bump_stat(5); // RX overruns
		// FIFO Diagnostic reports RX Overrun while the partition is full.
		return 0;
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

	// RX Early fires when the in-flight byte count passes its threshold.
	if ((m_rx_early_thresh <= MAX_PACKET_BYTES) && (keep > m_rx_early_thresh))
	{
		m_status |= ST_RX_EARLY;
		update_irq();
	}

	LOGNET("rx staged len=%d\n", length);
	return length;
}


void isa8_3c509b_device::recv_complete_cb(int result)
{
	m_rx_receiving = false;

	if ((result <= 0) || !m_rx_enabled)
		return;

	const uint16_t padded = (m_rx_stage_len + 3) & ~uint16_t(3);
	const uint16_t slot = (m_rx_head + m_rx_count) % RX_SLOTS;
	const uint32_t tail = (uint32_t(m_rxq_head) + m_rxq_used) % SRAM_PARTITION_BYTES;
	for (uint16_t i = 0; i < padded; i++)
		m_rxq[(tail + i) % SRAM_PARTITION_BYTES] = (i < m_rx_stage_len) ? m_rx_stage_data[i] : 0x00;
	m_rxq_used += padded;

	m_rx_len[slot] = m_rx_stage_len;
	m_rx_status[slot] = m_rx_stage_status;
	m_rx_count++;
	m_rx_stage_len = 0;

	bump_stat(7); // frames received OK
	bump_stat_word(m_stat_rx_bytes, m_rx_len[slot]);

	// RX Complete masks RX Early (section 6.10).
	m_status |= ST_RX_COMPLETE;
	m_status &= ~ST_RX_EARLY;

	update_irq();

	LOGNET("rx committed len=%u slot=%u\n", m_rx_len[slot], slot);
}
