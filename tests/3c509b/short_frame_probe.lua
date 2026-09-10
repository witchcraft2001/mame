local t = dofile("tests/3c509b/common.lua")
local base = 0x300

t.finish("pcap-short-frame", function()
	t.expect_eq(t.read_word(base), 0x6d50, "auto-activated Window 0")
	-- Auto-activation plus Immediate recovery is selected by the runner.
	-- Enabling link beat makes the external receive path ready.
	t.select_window(base, 4)
	t.write_word(base + 0x0a, 0x0080)
	t.expect((t.read_word(base + 0x0a) & 0x0800) ~= 0, "pcap link is not ready")
	t.select_window(base, 2)
	t.expect_eq(t.read_word(base + 0x00), 0x6002, "station bytes 0/1")
	t.expect_eq(t.read_word(base + 0x02), 0xde8c, "station bytes 2/3")
	t.expect_eq(t.read_word(base + 0x04), 0x42ad, "station bytes 4/5")

	-- Receive only our individual address so unrelated broadcasts on the
	-- host interface cannot satisfy the probe.
	t.command(base, 0x8001) -- SetRxFilter: individual
	emu.wait(0.000003)
	t.command(base, 0x2000) -- RxEnable
	emu.wait(0.000003)
	t.select_window(base, 5)
	t.expect_eq(t.read_word(base + 0x08), 1, "individual RX filter")
	emu.wait(5.0)

	t.select_window(base, 1)
	local status = t.read_word(base + 0x08)
	t.expect_eq(status, 60, "unpadded host frame RX status")

	local expected = {
		0x02, 0x60, 0x8c, 0xde, 0xad, 0x42,
		0x66, 0x65, 0x74, 0x68, 0x00, 0x01,
		0x08, 0x06,
		0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x02,
		0x66, 0x65, 0x74, 0x68, 0x00, 0x01,
		0xc0, 0xa8, 0x07, 0x01,
		0x02, 0x60, 0x8c, 0xde, 0xad, 0x42,
		0xc0, 0xa8, 0x07, 0x81,
	}
	for offset, value in ipairs(expected) do
		t.expect_eq(t.io:read_u8(base), value, string.format("ARP byte %d", offset - 1))
	end
	for offset = #expected, 59 do
		t.expect_eq(t.io:read_u8(base), 0, string.format("wire padding byte %d", offset))
	end
end)
