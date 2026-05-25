// license:BSD-3-Clause
// copyright-holders:Dmitry
/***************************************************************************

    ISA8 Realtek RTL8019AS Ethernet adapter

***************************************************************************/

#ifndef MAME_BUS_ISA_RTL8019AS_H
#define MAME_BUS_ISA_RTL8019AS_H

#pragma once

#include "isa.h"

#include "bus/generic/slot.h"
#include "machine/dp8390.h"
#include "machine/intelfsh.h"


class isa8_rtl8019as_device :
	public device_t,
	public device_isa8_card_interface
{
public:
	isa8_rtl8019as_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	static uint16_t decode_iobase(uint8_t config);
	static uint8_t decode_irq(uint8_t config);

	void update_prom(uint8_t const *mac);
	uint8_t port_r(offs_t offset);
	void port_w(offs_t offset, uint8_t data);
	uint8_t mem_read(offs_t offset);
	void mem_write(offs_t offset, uint8_t data);
	uint8_t flash_r(offs_t offset);
	void flash_w(offs_t offset, uint8_t data);
	std::pair<std::error_condition, std::string> flash_image_load(device_image_interface &image);
	void irq_w(int state);

	required_device<rtl8019a_device> m_dp8390;
	required_device<sst_39sf040_device> m_flash;
	required_device<generic_slot_device> m_flash_image;
	uint8_t m_prom[32];
	uint8_t m_board_ram[16 * 1024];
	uint8_t m_irq;
	uint16_t m_iobase;
	bool m_prom_doubled;
	uint32_t m_flash_base;
	uint32_t m_flash_size;
	bool m_flash_page_mode;
};

DECLARE_DEVICE_TYPE(ISA8_RTL8019AS, isa8_rtl8019as_device)

#endif // MAME_BUS_ISA_RTL8019AS_H
