// license:BSD-3-Clause
// copyright-holders:Roman Boykov, Dmitry
/***************************************************************************

    Sprinter-WiFi / Sprinter-ESP ISA card

    The card decodes COM3 (0x03e8-0x03ef) and uses a TL16C550C UART clocked
    by 14.7456 MHz.  ESPKit programs divisor 8 to get 115200 bit/s.

***************************************************************************/

#include "emu.h"
#include "sprinter_esp.h"

#include "bus/rs232/null_modem.h"
#include "bus/rs232/rs232.h"
#include "bus/rs232/terminal.h"
#include "machine/ins8250.h"


namespace {

static void sprinter_esp_rs232_devices(device_slot_interface &device)
{
	device.option_add("null_modem", NULL_MODEM);
	device.option_add("terminal", SERIAL_TERMINAL);
}

static DEVICE_INPUT_DEFAULTS_START(sprinter_esp_rs232_defaults)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_8)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END


class isa8_sprinter_esp_device : public device_t, public device_isa8_card_interface
{
public:
	isa8_sprinter_esp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		device_t(mconfig, ISA8_SPRINTER_ESP, tag, owner, clock),
		device_isa8_card_interface(mconfig, *this)
	{
	}

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
};


void isa8_sprinter_esp_device::device_add_mconfig(machine_config &config)
{
	ns16550_device &uart(NS16550(config, "uart", XTAL(14'745'600)));
	uart.out_tx_callback().set("esp", FUNC(rs232_port_device::write_txd));
	uart.out_dtr_callback().set("esp", FUNC(rs232_port_device::write_dtr));
	uart.out_rts_callback().set("esp", FUNC(rs232_port_device::write_rts));

	rs232_port_device &esp(RS232_PORT(config, "esp", sprinter_esp_rs232_devices, "null_modem"));
	esp.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(sprinter_esp_rs232_defaults));
	esp.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(sprinter_esp_rs232_defaults));
	esp.rxd_handler().set(uart, FUNC(ins8250_uart_device::rx_w));
	esp.dcd_handler().set(uart, FUNC(ins8250_uart_device::dcd_w));
	esp.dsr_handler().set(uart, FUNC(ins8250_uart_device::dsr_w));
	esp.ri_handler().set(uart, FUNC(ins8250_uart_device::ri_w));
	esp.cts_handler().set(uart, FUNC(ins8250_uart_device::cts_w));
}


void isa8_sprinter_esp_device::device_start()
{
	set_isa_device();
	m_isa->install_device(0x03e8, 0x03ef,
			read8sm_delegate(*subdevice<ins8250_uart_device>("uart"), FUNC(ins8250_device::ins8250_r)),
			write8sm_delegate(*subdevice<ins8250_uart_device>("uart"), FUNC(ins8250_device::ins8250_w)));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(ISA8_SPRINTER_ESP, device_isa8_card_interface, isa8_sprinter_esp_device, "sprinter_esp", "Sprinter-WiFi ISA ESP8266")
