# 3C509B headless regression probes

Run from the MAME source root after building:

```sh
MAME_ROMS=/path/to/roms MAME_NETWORK_INDEX=0 tests/3c509b/run.sh
```

`MAME_BIN` may select another binary. `MAME_NETWORK_INDEX` selects a pcap
interface used only to make backend-presence visible to Media Status; the
timing probes do not intentionally transmit a frame after recovery. The suite
covers the physical EEPROM golden vector, dynamic/configured MAC byte order,
read-only fixture enforcement, PnP resource preservation and serial/checksums,
deliberate corruption of word `18h`, repeat discovery with tag zero, both ISA
slots and bases `0200h`/`0300h`/`03E0h`, TPO selection ordering, independent
AUTOINIT and link-recovery timing, recovery stability across Rx/Tx Reset,
backend-dependent Media Status, early-TX disposal, and save/load in the middle
of EEPROM read and link recovery.
Both `Realistic (3 seconds)` and `Immediate` recovery settings are exercised.

On a host with a writable raw packet interface, set
`MAME_3C509B_PCAP_PEER` to the peer of the interface selected by
`MAME_NETWORK_INDEX`.  This enables an end-to-end regression that injects a
42-byte unpadded ARP reply and verifies that the pcap boundary presents it to
the emulated controller as a valid, zero-padded 60-byte Ethernet frame.  For
the macOS `feth0`/`feth1` setup:

```sh
MAME_ROMS=/path/to/roms MAME_NETWORK_INDEX=8 \
MAME_3C509B_PCAP_PEER=feth1 tests/3c509b/run.sh
```
