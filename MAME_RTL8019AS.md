# RTL8019AS для Sprinter в MAME

Эта сборка добавляет ISA8-карту `rtl8019as` для машины `sprinter`. Карта
использует существующее ядро `RTL8019A`/DP8390 MAME и 8-битную ISA-обвязку,
подходящую для будущих DSS-утилит `NICINFO.EXE`, `NICRAM.EXE`, `PING.EXE`,
`WGET.EXE`, `NTP.EXE`, `FTP.EXE` и `TFTP.EXE`.

## Запуск

Базовая команда:

```bash
../mame/mame sprinter -isa1 rtl8019as
```

Helper-скрипт:

```bash
./run_sprinter_rtl8019as.sh
./run_sprinter_rtl8019as.sh -verbose
RTL8019AS_VERBOSE=1 ./run_sprinter_rtl8019as.sh
SPRINTER_DEBUG=0 ./run_sprinter_rtl8019as.sh
```

Карту можно ставить в ISA-слоты Sprinter через `-isa0 rtl8019as` или
`-isa1 rtl8019as`. Типичная конфигурация оставляет `-isa0 zxbus_adapter` и
ставит Ethernet-карту в `-isa1`.

## Параметры карты

Default I/O base: `0x300`.

I/O base в первом этапе фиксирован: `0x300`.

Default IRQ: `3`.

Поддержанные IRQ:

```text
IRQ2/9, IRQ3, IRQ4, IRQ5
```

На этом этапе IRQ реализован как MAME input port, а I/O base фиксирован в
устройстве. Скрипт печатает переменные `RTL8019AS_IOBASE`, `RTL8019AS_IRQ`,
`RTL8019AS_MAC` и
`RTL8019AS_NETDEV` для диагностики, но не применяет их как ложные command-line
override. Изменение input ports выполняется стандартными средствами MAME
configuration/UI, а дополнительные аргументы можно передать в конец команды
скрипта.

MAC генерируется устройством при старте в локально администрируемом диапазоне:

```text
02:80:19:xx:xx:xx
```

Фактический MAC печатается в log при запуске:

```text
rtl8019as: start io=0300 irq=3 mac=02:80:19:xx:xx:xx
```

## Программная модель для DSS

```text
BASE+0x00..BASE+0x0f  DP8390/RTL8019A registers
BASE+0x10             8-bit remote DMA data port
BASE+0x11..BASE+0x1e  reserved, invalid access is logged
BASE+0x1f             reset port
default BASE          0x300
default TX page       0x40
default RX ring       0x46..0x80
```

Reset port:

```text
read  BASE+0x1f  clear DP8390 reset
write BASE+0x1f  assert DP8390 reset
```

Packet RAM:

```text
PROM/MAC area       0x0000..0x001f
packet RAM          0x4000..0x7fff
packet RAM size     16 KiB
PROM[0..5]          MAC
PROM[6..31]         0x57
```

Для RTL8019A page 3 ID ранняя диагностика может выбрать CR page 3 и читать:

```text
reg 0x0a -> 'P'
reg 0x0b -> 'p'
```

## Диагностика

Проверка слотов:

```bash
../mame/mame sprinter -listslots
```

Проверка дочерних устройств:

```bash
../mame/mame sprinter -isa1 rtl8019as -listdevices
```

Verbose-запуск:

```bash
../mame/mame sprinter -isa1 rtl8019as -verbose
./run_sprinter_rtl8019as.sh -verbose
RTL8019AS_VERBOSE=1 ./run_sprinter_rtl8019as.sh
```

Ожидаемые log-строки:

```text
rtl8019as: start io=0300 irq=3 mac=...
rtl8019as: reset assert
rtl8019as: reset clear
rtl8019as: irq3 state=1
rtl8019as: irq3 state=0
rtl8019as: invalid read off=12
rtl8019as: invalid write off=12 data=xx
```

Побайтный I/O и remote DMA log в исходнике вынесены в `LOG_IO` и `LOG_DMA` и
по умолчанию не включены, чтобы `-verbose` не создавал постоянный поток строк.

## Отличие от sprinter_esp

`sprinter_esp` эмулирует UART-карту для ESP-AT/serial-сценариев. `rtl8019as`
эмулирует Ethernet NIC на базе DP8390/RTL8019A и не запускает Jesperl, `socat`
или serial bridge.

## Ограничения первого этапа

Хостовая сеть зависит от MAME network provider и окружения пользователя. В
этом этапе нет отдельного TAP/helper-скрипта и нет TCP/IP стека внутри MAME:
кадры передаются через стандартный `device_network_interface`.

При репорте проблемы передайте команду запуска, фрагмент log вокруг
`rtl8019as`, вывод `-listslots`/`-listdevices` и скриншот ошибки, если MAME
упал или завис.
