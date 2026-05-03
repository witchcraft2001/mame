// license:BSD-3-Clause
// copyright-holders:Dmitry
/***************************************************************************

    ISA8 Realtek RTL8019AS Ethernet adapter

    The board exposes the DP8390-compatible RTL8019A register window at
    base+0x00..0x0f, an 8-bit remote DMA data port at base+0x10 and a reset
    port at base+0x1f.  Packet RAM is mapped to 0x4000..0x7fff for NE2000-like
    software, but the data port remains strictly 8-bit for Sprinter ISA slots.

***************************************************************************/

#include "emu.h"
#include "rtl8019as.h"

#include "multibyte.h"

#define LOG_IO    (1U << 1)
#define LOG_IRQ   (1U << 2)
#define LOG_DMA   (1U << 3)
#define LOG_NET   (1U << 4)

#define VERBOSE (LOG_GENERAL | LOG_IRQ | LOG_NET)

#include "logmacro.h"

#define LOGIO(...)  LOGMASKED(LOG_IO, __VA_ARGS__)
#define LOGIRQ(...) LOGMASKED(LOG_IRQ, __VA_ARGS__)
#define LOGDMA(...) LOGMASKED(LOG_DMA, __VA_ARGS__)
#define LOGNET(...) LOGMASKED(LOG_NET, __VA_ARGS__)


DEFINE_DEVICE_TYPE(ISA8_RTL8019AS, isa8_rtl8019as_device, "rtl8019as", "RTL8019AS ISA8 Ethernet Adapter")


isa8_rtl8019as_device::isa8_rtl8019as_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISA8_RTL8019AS, tag, owner, clock),
	device_isa8_card_interface(mconfig, *this),
	m_dp8390(*this, "rtl8019a"),
	m_prom{ },
	m_board_ram{ },
	m_irq(3),
	m_iobase(0x0300)
{
}


void isa8_rtl8019as_device::device_add_mconfig(machine_config &config)
{
	RTL8019A(config, m_dp8390, 0);
	m_dp8390->irq_callback().set(FUNC(isa8_rtl8019as_device::irq_w));
	m_dp8390->mem_read_callback().set(FUNC(isa8_rtl8019as_device::mem_read));
	m_dp8390->mem_write_callback().set(FUNC(isa8_rtl8019as_device::mem_write));
}


void isa8_rtl8019as_device::device_start()
{
	const uint8_t config = ioport("CONFIG")->read();
	m_iobase = decode_iobase(config);
	m_irq = decode_irq(config);

	std::fill(std::begin(m_prom), std::end(m_prom), 0x57);
	std::fill(std::begin(m_board_ram), std::end(m_board_ram), 0x00);

	uint32_t mac_tail = 0x123456;
	for (char const *p = tag(); *p != '\0'; p++)
		mac_tail = (mac_tail * 33) ^ uint8_t(*p);

	uint8_t mac[6] = { 0x02, 0x80, 0x19, 0x00, 0x00, 0x00 };
	put_u24be(&mac[3], mac_tail);
	std::copy(std::begin(mac), std::end(mac), m_prom);
	m_dp8390->set_mac(mac);

	set_isa_device();
	m_isa->install_device(m_iobase, m_iobase + 0x1f,
			read8sm_delegate(*this, FUNC(isa8_rtl8019as_device::port_r)),
			write8sm_delegate(*this, FUNC(isa8_rtl8019as_device::port_w)));

	osd_printf_verbose("rtl8019as: start io=%04x irq=%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
			m_iobase, m_irq, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	save_item(NAME(m_prom));
	save_item(NAME(m_board_ram));
	save_item(NAME(m_irq));
	save_item(NAME(m_iobase));
}


void isa8_rtl8019as_device::device_reset()
{
	const uint8_t config = ioport("CONFIG")->read();
	m_irq = decode_irq(config);
	std::copy_n(&m_dp8390->get_mac()[0], 6, m_prom);
}


uint16_t isa8_rtl8019as_device::decode_iobase(uint8_t config)
{
	switch (config & 0x30)
	{
	case 0x10: return 0x0320;
	case 0x20: return 0x0340;
	case 0x30: return 0x0360;
	default:   return 0x0300;
	}
}


uint8_t isa8_rtl8019as_device::decode_irq(uint8_t config)
{
	switch (config & 0x03)
	{
	case 0x00: return 2;
	case 0x02: return 4;
	case 0x03: return 5;
	default:   return 3;
	}
}


uint8_t isa8_rtl8019as_device::port_r(offs_t offset)
{
	uint8_t data = 0xff;

	if (offset < 0x10)
	{
		data = m_dp8390->cs_read(offset);
		LOGIO("rtl8019as: read port=%04x off=%02x data=%02x\n", m_iobase + uint16_t(offset), uint8_t(offset), data);
		return data;
	}

	switch (offset)
	{
	case 0x10:
		data = m_dp8390->remote_read() & 0xff;
		LOGDMA("rtl8019as: remote read data=%02x\n", data);
		return data;

	case 0x1f:
		osd_printf_verbose("rtl8019as: reset clear\n");
		m_dp8390->dp8390_reset(CLEAR_LINE);
		return 0x00;

	default:
		osd_printf_verbose("rtl8019as: invalid read off=%02x\n", uint8_t(offset));
		return 0xff;
	}
}


void isa8_rtl8019as_device::port_w(offs_t offset, uint8_t data)
{
	if (offset < 0x10)
	{
		LOGIO("rtl8019as: write port=%04x off=%02x data=%02x\n", m_iobase + uint16_t(offset), uint8_t(offset), data);
		m_dp8390->cs_write(offset, data);
		return;
	}

	switch (offset)
	{
	case 0x10:
		LOGDMA("rtl8019as: remote write data=%02x\n", data);
		m_dp8390->remote_write(data);
		return;

	case 0x1f:
		osd_printf_verbose("rtl8019as: reset assert\n");
		m_dp8390->dp8390_reset(ASSERT_LINE);
		return;

	default:
		osd_printf_verbose("rtl8019as: invalid write off=%02x data=%02x\n", uint8_t(offset), data);
		return;
	}
}


uint8_t isa8_rtl8019as_device::mem_read(offs_t offset)
{
	if (offset < sizeof(m_prom))
		return m_prom[offset];

	if ((0x4000 <= offset) && (offset < 0x8000))
	{
		const uint8_t data = m_board_ram[offset - 0x4000];
		LOGDMA("rtl8019as: remote read addr=%04x data=%02x\n", uint16_t(offset), data);
		return data;
	}

	osd_printf_verbose("rtl8019as: invalid memory read addr=%04x\n", uint16_t(offset));
	return 0xff;
}


void isa8_rtl8019as_device::mem_write(offs_t offset, uint8_t data)
{
	if ((0x4000 <= offset) && (offset < 0x8000))
	{
		LOGDMA("rtl8019as: remote write addr=%04x data=%02x\n", uint16_t(offset), data);
		m_board_ram[offset - 0x4000] = data;
		return;
	}

	osd_printf_verbose("rtl8019as: invalid memory write addr=%04x data=%02x\n", uint16_t(offset), data);
}


void isa8_rtl8019as_device::irq_w(int state)
{
	osd_printf_verbose("rtl8019as: irq%u state=%d\n", m_irq, state);

	switch (m_irq)
	{
	case 2:
		m_isa->irq2_w(state);
		break;
	case 3:
		m_isa->irq3_w(state);
		break;
	case 4:
		m_isa->irq4_w(state);
		break;
	case 5:
		m_isa->irq5_w(state);
		break;
	}
}


static INPUT_PORTS_START(rtl8019as)
	PORT_START("CONFIG")
	PORT_CONFNAME(0x03, 0x01, "RTL8019AS IRQ")
	PORT_CONFSETTING(0x00, "IRQ2/9")
	PORT_CONFSETTING(0x01, "IRQ3")
	PORT_CONFSETTING(0x02, "IRQ4")
	PORT_CONFSETTING(0x03, "IRQ5")
	PORT_CONFNAME(0x30, 0x00, "RTL8019AS I/O base")
	PORT_CONFSETTING(0x00, "0x300")
	PORT_CONFSETTING(0x10, "0x320")
	PORT_CONFSETTING(0x20, "0x340")
	PORT_CONFSETTING(0x30, "0x360")
INPUT_PORTS_END


ioport_constructor isa8_rtl8019as_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(rtl8019as);
}
