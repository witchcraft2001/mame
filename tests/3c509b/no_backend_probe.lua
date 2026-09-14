local t = dofile("tests/3c509b/common.lua")
local base = 0x300

t.finish("no-backend", function()
	t.expect_eq(t.read_word(base), 0x6d50, "auto-activated Window 0")
	t.select_window(base, 4)
	t.write_word(base + 0x0a, 0x0080)
	-- Immediate mode has already made the TPO path READY, but without an
	-- opened MAME network device the read-only Valid Link Beat bit stays low.
	t.expect((t.read_word(base + 0x0a) & 0x0800) == 0, "link beat visible without a backend")
end)
