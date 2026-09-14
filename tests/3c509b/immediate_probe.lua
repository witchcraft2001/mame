local t = dofile("tests/3c509b/common.lua")
local base = 0x300

t.finish("immediate", function()
	t.expect_eq(t.read_word(base), 0x6d50, "auto-activated Window 0")
	t.select_window(base, 4)
	t.write_word(base + 0x0a, 0x0080)
	t.expect((t.read_word(base + 0x0a) & 0x0800) ~= 0, "Immediate recovery left link down")

	-- TPO selection is a recovery prerequisite independent of activation
	-- and link-beat enable.  Leaving TPO invalidates READY; selecting it
	-- again must notice the already-satisfied prerequisites.
	t.select_window(base, 0)
	t.write_word(base + 0x06, 0x4010)
	t.select_window(base, 4)
	t.expect((t.read_word(base + 0x0a) & 0x0800) == 0, "non-TPO selection retained link beat")
	t.select_window(base, 0)
	t.write_word(base + 0x06, 0x0010)
	t.select_window(base, 4)
	t.expect((t.read_word(base + 0x0a) & 0x0800) ~= 0, "TPO re-selection did not start recovery")
end)
