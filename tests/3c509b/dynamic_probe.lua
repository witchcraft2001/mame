local t = dofile("tests/3c509b/common.lua")

t.finish("dynamic", function()
	local words = t.discover()
	t.expect_eq(words[1], 0x0260, "configured MAC word 00")
	t.expect_eq(words[2], 0x8c12, "configured MAC word 01")
	t.expect_eq(words[3], 0x3456, "configured MAC word 02")
	t.expect_eq(words[0x0a + 1], 0x0260, "OEM MAC word 0A")
	t.expect_eq(words[0x0b + 1], 0x8c12, "OEM MAC word 0B")
	t.expect_eq(words[0x0c + 1], 0x3456, "OEM MAC word 0C")
	t.expect_eq(words[0x1a + 1], 0x3456, "PnP serial word 1A")
	t.expect_eq(words[0x1b + 1], 0x8c12, "PnP serial word 1B")
	t.expect_eq(words[0x1c + 1] & 0xff, 0x35, "PnP serial checksum")
	t.expect_eq(words[0x1c + 1] & 0xff, t.pnp_serial_checksum(words), "calculated PnP serial checksum")
	t.expect(t.validate_primary(words), "dynamic primary checksum rejected")
	t.expect(t.validate_secondary(words), "dynamic secondary checksum rejected")

	-- Apart from the MAC-derived serial words, the physical resource stream
	-- must be copied byte-for-byte into the dynamic profile.
	local physical_pnp = {
		[0x18] = 0x6d50, [0x19] = 0x9550, [0x1d] = 0x1010, [0x1e] = 0x1982, [0x1f] = 0x3300,
		[0x20] = 0x6f43, [0x21] = 0x206d, [0x22] = 0x4333, [0x23] = 0x3035,
		[0x24] = 0x4239, [0x25] = 0x4520, [0x26] = 0x6874, [0x27] = 0x7265,
		[0x28] = 0x694c, [0x29] = 0x6b6e, [0x2a] = 0x4920, [0x2b] = 0x4949,
		[0x2c] = 0x5015, [0x2d] = 0x506d, [0x2e] = 0x0295, [0x2f] = 0x411c,
		[0x30] = 0x80d0, [0x31] = 0x22f7, [0x32] = 0x9ea8, [0x33] = 0x0147,
		[0x34] = 0x0210, [0x35] = 0x03e0, [0x36] = 0x1010, [0x37] = 0x3779,
		[0x38] = 0x0000, [0x39] = 0x0000, [0x3a] = 0x0000, [0x3b] = 0x0000,
		[0x3c] = 0x0000, [0x3d] = 0x0000, [0x3e] = 0x0000, [0x3f] = 0x0000,
	}
	for address, expected in pairs(physical_pnp) do
		t.expect_eq(words[address + 1], expected, string.format("PnP resource word %02X", address))
	end
	t.expect_eq(words[0x1c + 1] & 0xff00, 0x0a00, "PnP resource stream first byte")

	t.select_window(0x300, 2)
	t.expect_eq(t.read_word(0x300), 0x6002, "Window 2 Address0/1")
	t.expect_eq(t.read_word(0x302), 0x128c, "Window 2 Address2/3")
	t.expect_eq(t.read_word(0x304), 0x5634, "Window 2 Address4/5")

	-- Corrupt word 18 in the writable dynamic profile without updating 17.
	t.select_window(0x300, 0)
	t.write_word(0x30a, 0x0030) -- EWEN
	emu.wait(0.000061)
	t.write_word(0x30c, 0x6d40)
	t.write_word(0x30a, 0x0058) -- write word 18
	emu.wait(0.0111)
	local corrupted = t.discover()
	t.expect_eq(corrupted[0x18 + 1], 0x6d40, "corrupted PnP word")
	t.expect(not t.validate_secondary(corrupted), "secondary checksum missed word 18 corruption")
end)
