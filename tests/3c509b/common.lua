local M = {}

M.slot = os.getenv("MAME_3C509B_SLOT") or "isa1"
M.tag = ":" .. M.slot .. ":3c509b"
M.id_port = 0x110
M.io = manager.machine.devices[M.slot == "isa0" and ":isa80" or ":isa81"].spaces["io8"]

-- The probes drive the ISA bus directly.  Halt the guest CPU so firmware
-- cannot reset/reconfigure the same bus while a timed test is in progress.
local cpu_state = manager.machine.devices[":maincpu"].state
cpu_state["IFF1"].value = 0
cpu_state["IFF2"].value = 0
cpu_state["HALT"].value = 1

-- Autoboot scripts are launched as the machine enters RUNNING.  Leave one
-- scheduler slice for reset/configuration side effects before touching ISA.
emu.wait(0.001)

function M.expect(condition, message)
	if not condition then error(message, 2) end
end

function M.expect_eq(actual, expected, message)
	if actual ~= expected then
		error(string.format("%s: got %04X, expected %04X", message, actual, expected), 2)
	end
end

function M.read_word(port)
	local lo = M.io:read_u8(port)
	return lo | (M.io:read_u8(port + 1) << 8)
end

function M.write_word(port, value)
	M.io:write_u8(port, value & 0xff)
	M.io:write_u8(port + 1, (value >> 8) & 0xff)
end

function M.command(base, value)
	M.write_word(base + 0x0e, value)
end

function M.select_window(base, window)
	M.command(base, 0x0800 | window)
	emu.wait(0.000003)
end

local function next_id_lfsr(value)
	if (value & 0x80) ~= 0 then return ((value << 1) ~ 0xcf) & 0xff end
	return (value << 1) & 0xff
end

function M.id_sequence()
	M.io:write_u8(M.id_port, 0)
	M.io:write_u8(M.id_port, 0)
	local value = 0xff
	for _ = 1, 255 do
		M.io:write_u8(M.id_port, value)
		value = next_id_lfsr(value)
	end
end

function M.id_read_word(address)
	M.io:write_u8(M.id_port, 0x80 | address)
	emu.wait(0.000163)
	local value = 0
	for _ = 1, 16 do value = (value << 1) | (M.io:read_u8(M.id_port) & 1) end
	return value
end

function M.discover(activate_command)
	M.id_sequence()
	M.io:write_u8(M.id_port, 0xd0) -- tag zero remains eligible next time
	local words = {}
	for address = 0, 0x3f do words[address + 1] = M.id_read_word(address) end
	M.io:write_u8(M.id_port, activate_command or 0xff)
	return words
end

function M.xor_bytes(words, ranges)
	local value = 0
	for _, range in ipairs(ranges) do
		for address = range[1], range[2] do
			local word = words[address + 1]
			value = value ~ (word & 0xff) ~ ((word >> 8) & 0xff)
		end
	end
	return value & 0xff
end

function M.validate_secondary(words)
	local vital = M.xor_bytes(words, { { 0x10, 0x12 }, { 0x18, 0x3f } })
	local configurable = M.xor_bytes(words, { { 0x13, 0x16 } })
	return words[0x17 + 1] == ((vital << 8) | configurable)
end

function M.validate_primary(words)
	local vital = M.xor_bytes(words, { { 0x00, 0x07 }, { 0x0a, 0x0c }, { 0x0e, 0x0e } })
	local configurable = M.xor_bytes(words, { { 0x08, 0x09 }, { 0x0d, 0x0d } })
	return words[0x0f + 1] == ((vital << 8) | configurable)
end

function M.pnp_serial_checksum(words)
	local lfsr = 0x6a
	for address = 0x18, 0x1b do
		local word = words[address + 1]
		for byte = 0, 1 do
			local data = (word >> (byte * 8)) & 0xff
			for bit = 0, 7 do
				local feedback = ((lfsr >> 1) ~ lfsr ~ (data >> bit)) & 1
				lfsr = ((lfsr >> 1) | (feedback << 7)) & 0xff
			end
		end
	end
	return lfsr
end

function M.finish(name, body)
	local ok, message = pcall(body)
	if ok then print("3C509B " .. name .. " PASS")
	else print("3C509B " .. name .. " FAIL: " .. tostring(message)) end
	manager.machine:exit()
end

return M
