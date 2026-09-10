local t = dofile("tests/3c509b/common.lua")

t.finish("bases-" .. t.slot, function()
	for _, base in ipairs({ 0x200, 0x300, 0x3e0 }) do
		local command = 0xe0 | ((base - 0x200) >> 4)
		local words = t.discover(command)
		t.expect_eq(words[0x03 + 1], 0x9550, string.format("product at base %03X", base))
		t.expect_eq(t.read_word(base), 0x6d50, string.format("Window 0 at base %03X", base))
	end
end)
