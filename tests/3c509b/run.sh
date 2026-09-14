#!/bin/sh
set -eu

MAME_BIN=${MAME_BIN:-./mame}
MAME_ROMS=${MAME_ROMS:-roms}
MAME_NETWORK_INDEX=${MAME_NETWORK_INDEX:-0}
MAME_3C509B_PCAP_PEER=${MAME_3C509B_PCAP_PEER:-}

case "$MAME_BIN" in
/*) ;;
*) MAME_BIN="$(pwd)/${MAME_BIN#./}" ;;
esac
case "$MAME_ROMS" in
/*) ;;
*) MAME_ROMS="$(pwd)/$MAME_ROMS" ;;
esac

test_tmp=$(mktemp -d "${TMPDIR:-/tmp}/mame-3c509b.XXXXXX")
trap 'rm -r "$test_tmp"' EXIT HUP INT TERM

make_config()
{
	slot=$1 profile=$2 activate=$3 link=$4 interface=$5 mac=$6 destination=$7
	sed -e "s/@SLOT@/$slot/g" \
		-e "s/@PROFILE@/$profile/g" \
		-e "s/@ACTIVATE@/$activate/g" \
		-e "s/@LINK@/$link/g" \
		-e "s/@INTERFACE@/$interface/g" \
		-e "s/@MAC@/$mac/g" \
		tests/3c509b/profile.cfg.in > "$destination/sprinter.cfg"
}

run_probe()
{
	name=$1 slot=$2 profile=$3 activate=$4 link=$5 interface=$6 mac=$7 provider=$8
	cfg="$test_tmp/$name"
	mkdir -p "$cfg"
	make_config "$slot" "$profile" "$activate" "$link" "$interface" "$mac" "$cfg"
	log="$cfg/output.log"
	if ! MAME_3C509B_SLOT="$slot" SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
		"$MAME_BIN" sprinter "-$slot" 3c509b \
		-networkprovider "$provider" -video soft -sound none -skip_gameinfo -nothrottle \
		-keyboardprovider none -mouseprovider none -joystickprovider none \
		-rompath "$MAME_ROMS" -cfg_directory "$cfg" -nvram_directory "$cfg" -state_directory "$cfg" \
		-autoboot_delay 1 -autoboot_script "tests/3c509b/$name.lua" -seconds_to_run 12 >"$log" 2>&1
	then
		cat "$log"
		return 1
	fi
	if ! grep -q "3C509B .* PASS" "$log"; then
		cat "$log"
		return 1
	fi
	if ! grep -q "mac=\"$mac\"" "$cfg/sprinter.cfg"; then
		echo "$name: saved network-interface MAC does not match $mac"
		return 1
	fi
	grep "3C509B .* PASS" "$log"
}

run_short_frame_probe()
{
	name=short_frame_probe
	slot=isa1
	cfg="$test_tmp/$name"
	mkdir -p "$cfg"
	make_config "$slot" 0 128 1 "$MAME_NETWORK_INDEX" 02:60:8c:de:ad:42 "$cfg"
	log="$cfg/output.log"
	MAME_3C509B_SLOT="$slot" SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
		"$MAME_BIN" sprinter "-$slot" 3c509b \
		-networkprovider pcap -video soft -sound none -skip_gameinfo -throttle \
		-keyboardprovider none -mouseprovider none -joystickprovider none \
		-rompath "$MAME_ROMS" -cfg_directory "$cfg" -nvram_directory "$cfg" -state_directory "$cfg" \
		-autoboot_delay 1 -autoboot_script "tests/3c509b/$name.lua" -seconds_to_run 9 >"$log" 2>&1 &
	mame_pid=$!
	sleep 1
	if ! python3 tests/3c509b/inject_short_frame.py "$MAME_3C509B_PCAP_PEER"; then
		kill "$mame_pid" 2>/dev/null || true
		wait "$mame_pid" 2>/dev/null || true
		return 1
	fi
	if ! wait "$mame_pid"; then
		cat "$log"
		return 1
	fi
	if ! grep -q "3C509B pcap-short-frame PASS" "$log"; then
		cat "$log"
		return 1
	fi
	grep "3C509B pcap-short-frame PASS" "$log"
}

run_probe physical_probe isa1 1 0 1 -1 00:20:af:5d:69:8b none
run_probe dynamic_probe isa1 0 0 1 -1 02:60:8c:12:34:56 none
run_probe base_probe isa0 0 0 1 -1 02:60:8c:12:34:56 none
run_probe base_probe isa1 0 0 1 -1 02:60:8c:12:34:56 none
run_probe no_backend_probe isa1 0 128 1 -1 02:60:8c:12:34:56 none
run_probe immediate_probe isa1 0 128 1 "$MAME_NETWORK_INDEX" 02:60:8c:12:34:56 pcap
run_probe timing_probe isa1 0 128 0 "$MAME_NETWORK_INDEX" 02:60:8c:12:34:56 pcap
run_probe savestate_probe isa1 0 128 0 "$MAME_NETWORK_INDEX" 02:60:8c:12:34:56 pcap
if [ -n "$MAME_3C509B_PCAP_PEER" ]; then
	run_short_frame_probe
else
	echo "3C509B pcap-short-frame SKIP (set MAME_3C509B_PCAP_PEER to the host-side peer)"
fi

echo "3C509B headless suite PASS"
