local t = dofile("tests/3c509b/common.lua")
local base = 0x300

local function wait_flag(predicate, timeout, message)
	local start = manager.machine.time
	while not predicate() do
		if (manager.machine.time - start):as_double() > timeout then error(message, 2) end
		emu.wait(0.000001)
	end
end

local function media_status()
	t.select_window(base, 4)
	return t.read_word(base + 0x0a)
end

t.finish("savestate", function()
	-- Save while a Window 0 EEPROM read is still in progress.
	t.write_word(base + 0x0a, 0x0083)
	emu.wait(0.000040)
	local saved = false
	local loaded = false
	local save_subscription = emu.add_machine_pre_save_notifier(function() saved = true end)
	local load_subscription = emu.add_machine_post_load_notifier(function() loaded = true end)
	manager.machine:save("3c509b-eeprom")
	wait_flag(function() return saved end, 0.001, "EEPROM state save timed out")
	emu.wait(0.000200)
	t.expect((t.read_word(base + 0x0a) & 0x8000) == 0, "EEPROM read never completed")
	manager.machine:load("3c509b-eeprom")
	wait_flag(function() return loaded end, 0.010, "EEPROM state load timed out")
	t.expect((t.read_word(base + 0x0a) & 0x8000) ~= 0, "EEPROM busy phase was not restored")
	emu.wait(0.000170)
	t.expect((t.read_word(base + 0x0a) & 0x8000) == 0, "restored EEPROM read did not complete")
	t.expect_eq(t.read_word(base + 0x0c), 0x9550, "restored EEPROM data")

	-- Save one second into link recovery.  Loading the state must restore
	-- the remaining two seconds rather than completing or restarting it.
	t.select_window(base, 4)
	t.write_word(base + 0x0a, 0x0080)
	emu.wait(1.000)
	saved = false
	loaded = false
	manager.machine:save("3c509b-link")
	wait_flag(function() return saved end, 0.010, "link state save timed out")
	emu.wait(1.000)
	manager.machine:load("3c509b-link")
	wait_flag(function() return loaded end, 0.010, "link state load timed out")
	emu.wait(1.950)
	t.expect((media_status() & 0x0800) == 0, "restored link timer completed early")
	emu.wait(0.100)
	t.expect((media_status() & 0x0800) ~= 0, "restored link timer did not complete")

	-- Keep notifier subscriptions live until the checks have completed.
	t.expect(save_subscription ~= nil and load_subscription ~= nil, "save/load notifiers unavailable")
end)
