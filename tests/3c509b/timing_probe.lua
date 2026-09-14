local t = dofile("tests/3c509b/common.lua")
local base = 0x300

local function media_status()
	t.select_window(base, 4)
	return t.read_word(base + 0x0a)
end

local function enable_link_beat()
	t.select_window(base, 4)
	t.write_word(base + 0x0a, 0x0080)
end

t.finish("timing", function()
	t.expect_eq(t.read_word(base), 0x6d50, "auto-activated Window 0")
	enable_link_beat()
	local recovery_start = manager.machine.time
	t.expect((media_status() & 0x0800) == 0, "link beat visible at recovery start")

	-- A pre-recovery TX must complete normally in the local FIFO/status
	-- machinery, but must not remain queued for replay when the link rises.
	t.command(base, 0x4800) -- TxEnable
	emu.wait(0.000003)
	t.select_window(base, 1)
	for _, byte in ipairs({ 0x3c, 0x80, 0x00, 0x00 }) do t.io:write_u8(base, byte) end
	for _ = 1, 60 do t.io:write_u8(base, 0) end
	emu.wait(0.001)
	t.expect_eq(t.io:read_u8(base + 0x0b), 0xc0, "early TX completion")
	t.io:write_u8(base + 0x0b, 0)
	t.expect_eq(t.read_word(base + 0x0c), 0x4000, "early TX FIFO release")

	-- Re-running discovery and activating a tag-zero card must not restart
	-- the recovery deadline.
	local rediscovered = t.discover()
	t.expect_eq(rediscovered[0x03 + 1], 0x9550, "rediscovery product")
	local elapsed = (manager.machine.time - recovery_start):as_double()
	if elapsed < 1.000 then emu.wait(1.000 - elapsed) end
	t.command(base, 0x2800) -- RxReset
	emu.wait(0.000011)
	t.command(base, 0x5800) -- TxReset
	emu.wait(0.000011)
	elapsed = (manager.machine.time - recovery_start):as_double()
	if elapsed < 2.990 then emu.wait(2.990 - elapsed) end
	t.expect((media_status() & 0x0800) == 0, "link beat rose before 3 seconds")
	elapsed = (manager.machine.time - recovery_start):as_double()
	if elapsed < 3.010 then emu.wait(3.010 - elapsed) end
	t.expect((media_status() & 0x0800) ~= 0, "link beat did not rise after 3 seconds")
	t.select_window(base, 1)
	t.expect_eq(t.io:read_u8(base + 0x0b), 0x00, "early TX replayed after recovery")

	-- Digital AUTOINIT and analogue link recovery have independent bounds.
	t.command(base, 0x0000) -- Global Reset
	emu.wait(0.000300)
	t.expect_eq(t.io:read_u8(base), 0xff, "AUTOINIT ended before 310 us")
	emu.wait(0.000012)
	t.expect_eq(t.read_word(base), 0x6d50, "AUTOINIT did not end after 310 us")
	enable_link_beat()
	emu.wait(2.990)
	t.expect((media_status() & 0x0800) == 0, "post-reset link beat rose early")
	emu.wait(0.020)
	t.expect((media_status() & 0x0800) ~= 0, "post-reset link recovery did not finish")
end)
