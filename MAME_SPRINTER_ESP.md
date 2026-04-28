# MAME: эмуляция Sprinter-WiFi / Sprinter-ESP через ISA

Эта настройка эмулирует не штатный `rs232` порт Sprinter для мыши, а реальную схему Sprinter-WiFi: ISA-8 карта, на ней UART `TL16C550C`, к UART подключен ESP8266/ESP-12F.

## Что добавлено в MAME

В исходниках `../mame` добавлена ISA-карта:

- устройство MAME: `sprinter_esp`;
- слот подключения: `-isa1 sprinter_esp`;
- UART: NS16550/TL16C550-совместимый;
- диапазон ISA I/O: `0x03e8-0x03ef`, то есть COM3;
- частота UART: `14.7456 MHz`;
- дочерний RS232-порт карты: `:isa1:sprinter_esp:esp`;
- по умолчанию к нему подключен `null_modem` и MAME `bitbanger`.

Основание:

- в ESPKit `PORT_UART` задан как `0x03E8`, регистры UART занимают offsets `0..7`;
- в ESPKit скорость `115200` получается делителем `8` от `14.7456 MHz`;
- в схеме SprinterESP указаны `TL16C550C`, `COM3 3E8-3EF`, `14.7456MHz`, `ESP-12F`.

Измененные файлы MAME:

- `../mame/src/devices/bus/isa/sprinter_esp.cpp`
- `../mame/src/devices/bus/isa/sprinter_esp.h`
- `../mame/src/devices/bus/isa/isa_cards.cpp`
- `../mame/scripts/src/bus.lua`
- `../mame/src/mame/sinclair/sprinter.cpp`

В `sprinter.cpp` также подключен ISA `RESET`: ESPKit дергает reset через порт ISA, и теперь этот бит сбрасывает обе ISA-шины MAME.

## Сборка MAME

После этих изменений нужен бинарник из `../mame`; старый release-бинарник `mame_release_v306_25.05.2025/mame` не знает опцию `sprinter_esp`.

```bash
cd /Users/dmitry/dev/zx/sprinter/mame
make -j4
```

Проверка, что карта появилась:

```bash
/Users/dmitry/dev/zx/sprinter/mame/mame sprinter -listslots | grep -A20 isa1
```

В списке для `isa1` должна быть опция:

```text
sprinter_esp
```

Проверка медиа-устройства для ESP:

```bash
/Users/dmitry/dev/zx/sprinter/mame/mame sprinter -isa1 sprinter_esp -isa1:sprinter_esp:esp null_modem -listmedia
```

Ожидаемый смысл вывода: должен появиться `bitbanger`, через который `null_modem` обменивается байтами с внешним процессом.

## Готовый запуск

В этом каталоге создан скрипт:

```bash
./run_sprinter_esp.sh
```

Он запускает тот же набор образов, что `_306.sh`, но добавляет SprinterESP ISA-карту:

```bash
-isa1 sprinter_esp
-isa1:sprinter_esp:esp null_modem
-bitb socket.127.0.0.1:25232
```

По умолчанию скрипт берет бинарник `../mame/mame`, если он собран. Если нужно указать бинарник явно:

```bash
MAME_BIN=/Users/dmitry/dev/zx/sprinter/mame/mame ./run_sprinter_esp.sh
```

Параметры сокета:

```bash
SPRINTER_ESP_HOST=127.0.0.1 SPRINTER_ESP_PORT=25232 ./run_sprinter_esp.sh
```

Отключить debugger:

```bash
SPRINTER_DEBUG=0 ./run_sprinter_esp.sh
```

## Подключение Jesperl

Jesperl: <https://sourceforge.net/projects/jesperl/files/>

Идея такая: MAME эмулирует ISA-карту и UART, а вместо физического ESP8266 за `null_modem` сидит процесс Jesperl, который отвечает на ESP AT-команды.

Порядок:

1. Запусти Jesperl. Скачанная версия `JESPERL_1_007` слушает фиксированный адрес `localhost:1234`.
2. Запусти:

```bash
SPRINTER_ESP_PORT=1234 ./run_sprinter_esp.sh
```

Если Jesperl работает как сервер, MAME подключится к нему. Если конкретная сборка Jesperl работает как клиент, сначала запускай MAME: `bitbanger` с адресом `socket.127.0.0.1:25232` сможет ждать входящего подключения.

Важно: совместимость зависит от набора AT-команд, которые реализует Jesperl. Софт SprinterESP использует ESP8266 AT-прошивку, поэтому Jesperl должен покрывать хотя бы эти команды.

## Подключение физического ESP

Физический ESP12/ESP8266 можно пробросить через тот же `bitbanger` сокет. Сначала запускается MAME:

```bash
./run_sprinter_esp.sh
```

Затем TCP-сокет соединяется с USB-UART адаптером:

```bash
socat -d -d TCP:127.0.0.1:25232 FILE:/dev/cu.usbserial-XXXX,raw,b115200,cs8,parenb=0,cstopb=0,ixon=0,ixoff=0
```

Заменить `/dev/cu.usbserial-XXXX` на реальный порт. На macOS список обычно виден так:

```bash
ls /dev/cu.*
```

## Почему не `-rs232 null_modem`

У Sprinter в `sprinter.cpp` уже есть устройство `rs232`, но оно подключено к каналу B встроенного `Z84C015` и по умолчанию используется как порт мыши. SprinterESP по исходникам и схеме находится на ISA-шине как COM3-карта с отдельным UART. Поэтому правильная конфигурация MAME должна добавлять ISA-карту `sprinter_esp`, а не заменять корневой `-rs232`.
