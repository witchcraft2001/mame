local t = dofile("tests/3c509b/common.lua")

local expected = {
	0x0020, 0xaf5d, 0x698b, 0x9550, 0xb434, 0x0041, 0x4a41, 0x6d50,
	0x0010, 0x3000, 0x0020, 0xaf5d, 0x698b, 0x1310, 0x0000, 0x3223,
	0x2083, 0x0000, 0x0000, 0x0004, 0x0001, 0x0000, 0x0000, 0x0205,
	0x6d50, 0x9550, 0x698b, 0xaf5d, 0x0a5b, 0x1010, 0x1982, 0x3300,
	0x6f43, 0x206d, 0x4333, 0x3035, 0x4239, 0x4520, 0x6874, 0x7265,
	0x694c, 0x6b6e, 0x4920, 0x4949, 0x5015, 0x506d, 0x0295, 0x411c,
	0x80d0, 0x22f7, 0x9ea8, 0x0147, 0x0210, 0x03e0, 0x1010, 0x3779,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
}

t.finish("physical", function()
	local first = t.discover()
	for i = 1, 64 do t.expect_eq(first[i], expected[i], string.format("EEPROM[%02X]", i - 1)) end
	t.expect(t.validate_primary(first), "physical primary checksum rejected")
	t.expect(t.validate_secondary(first), "physical secondary checksum rejected")
	t.expect_eq(first[0x1c + 1] & 0xff, t.pnp_serial_checksum(first), "physical PnP serial checksum")
	t.expect_eq(first[0x0f + 1], 0x3223, "primary checksum")
	t.expect_eq(first[0x17 + 1], 0x0205, "secondary checksum")

	t.select_window(0x300, 2)
	t.expect_eq(t.read_word(0x300), 0x2000, "Window 2 Address0/1")
	t.expect_eq(t.read_word(0x302), 0x5daf, "Window 2 Address2/3")
	t.expect_eq(t.read_word(0x304), 0x8b69, "Window 2 Address4/5")

	-- The fixture is a strict read-only regression image.  Exercise the
	-- complete EWEN/write sequence and prove that even a clearing write is
	-- ignored rather than merely rebuilt by a reset.
	t.select_window(0x300, 0)
	t.write_word(0x30a, 0x0030) -- EWEN
	emu.wait(0.000061)
	t.write_word(0x30c, 0x0000)
	t.write_word(0x30a, 0x0058) -- write word 18
	emu.wait(0.0111)

	local second = t.discover()
	for i = 1, 64 do t.expect_eq(second[i], first[i], string.format("second EEPROM[%02X]", i - 1)) end
end)
