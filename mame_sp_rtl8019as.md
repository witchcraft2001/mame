# План доработки MAME: ISA8 RTL8019AS для Sprinter

Документ предназначен как промпт для AI-агента, который должен реализовать
поддержку ISA8 Ethernet-карты на базе Realtek RTL8019AS в MAME для машины
`sprinter`. Доработка MAME должна быть завершена за один этап: агент должен
сразу добавить устройство, включить его в список ISA-слотов Sprinter, собрать
MAME и дать проверяемый результат.

## Цель

Добавить в MAME ISA8-карту `rtl8019as`, доступную для Sprinter через параметры
вида:

```bash
../mame/mame sprinter -isa1 rtl8019as
```

Устройство должно эмулировать программную модель RTL8019AS/NE2000-совместимого
адаптера на 8-битной ISA-шине, чтобы на стороне Sprinter DSS можно было писать
собственный драйвер и сетевые утилиты: `PING.EXE`, `WGET.EXE`, `NTP.EXE`,
`FTP.EXE`, `TFTP.EXE` и другие.

## Исходный контекст

Рабочий репозиторий:

```text
/Users/dmitry/dev/zx/sprinter/mame_esp
```

Соседнее дерево MAME:

```text
/Users/dmitry/dev/zx/sprinter/mame
```

В MAME уже есть полезные заготовки:

```text
../mame/src/devices/machine/dp8390.cpp
../mame/src/devices/machine/dp8390.h
../mame/src/devices/bus/isa/ne1000.cpp
../mame/src/devices/bus/isa/ne1000.h
../mame/src/devices/bus/isa/ne2000.cpp
../mame/src/devices/bus/isa/ne2000.h
../mame/src/devices/bus/isa/isa_cards.cpp
../mame/scripts/src/bus.lua
```

В `dp8390.*` уже объявлен и реализован тип:

```cpp
RTL8019A(config, m_dp8390, 0);
```

Класс `rtl8019a_device` наследует общий `dp8390_device`, который уже умеет:

- передавать кадры через `device_network_interface`;
- принимать кадры через `recv_cb`;
- обслуживать DP8390-регистры через `cs_read/cs_write`;
- выполнять remote DMA через `remote_read/remote_write`;
- выдавать IRQ через callback;
- частично отдавать RTL8019A page 3 ID-регистры, включая `P`/`p`.

Текущие ISA-адаптеры `ne1000` и `ne2000` являются ориентирами, но напрямую не
подходят:

- `ne1000` уже ISA8, но использует `DP8390D` и 8 КБ RAM;
- `ne2000` использует 16-битную ISA-шину;
- целевое устройство должно быть ISA8 RTL8019AS с 16 КБ packet RAM и
  8-битным data port.

## Обязательный результат

После работы агента должны выполняться проверки:

```bash
make -C ../mame -j4
../mame/mame sprinter -listslots
../mame/mame sprinter -isa1 rtl8019as -listdevices
bash -n run_sprinter_rtl8019as.sh
```

Ожидаемый результат:

- MAME собирается без ошибок;
- `rtl8019as` отображается среди доступных ISA-устройств для `sprinter`;
- `-listdevices` показывает дочерний `rtl8019a`/DP8390 Ethernet controller;
- в текущем каталоге создан скрипт запуска `run_sprinter_rtl8019as.sh`;
- в текущем каталоге создана инструкция `MAME_RTL8019AS.md`;
- существующие устройства `sprinter_esp`, `com`, `zxbus_adapter` не исчезают;
- запуск Sprinter с `-isa1 rtl8019as` не ломает клавиатуру, мышь и базовый старт
  машины.

## Архитектура реализации

Добавить новое ISA8-устройство:

```text
../mame/src/devices/bus/isa/rtl8019as.cpp
../mame/src/devices/bus/isa/rtl8019as.h
```

Рекомендуемая структура:

```cpp
class isa8_rtl8019as_device :
	public device_t,
	public device_isa8_card_interface
{
public:
	isa8_rtl8019as_device(
		const machine_config &mconfig,
		const char *tag,
		device_t *owner,
		uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual ioport_constructor device_input_ports() const override;

private:
	required_device<rtl8019a_device> m_dp8390;
	uint8_t m_prom[32];
	uint8_t m_board_ram[16 * 1024];
	uint8_t m_irq;
	uint16_t m_iobase;

	uint8_t port_r(offs_t offset);
	void port_w(offs_t offset, uint8_t data);
	uint8_t mem_read(offs_t offset);
	void mem_write(offs_t offset, uint8_t data);
	void irq_w(int state);
};
```

Имя device type:

```cpp
DECLARE_DEVICE_TYPE(ISA8_RTL8019AS, isa8_rtl8019as_device)
```

Идентификатор слота:

```cpp
device.option_add("rtl8019as", ISA8_RTL8019AS);
```

## Подключение к сборке MAME

Агент должен обновить:

```text
../mame/scripts/src/bus.lua
```

Добавить новые файлы рядом с `sprinter_esp.cpp/h`, `ne1000.cpp/h`,
`ne2000.cpp/h`.

Агент должен обновить:

```text
../mame/src/devices/bus/isa/isa_cards.cpp
```

Добавить include:

```cpp
#include "rtl8019as.h"
```

И добавить устройство в активный список ISA8-слотов, доступный для `sprinter`.
Не переносить карту только в большой закомментированный список PC ISA-карт:
она должна реально появиться в `sprinter -listslots`.

## Карта портов

Начальная реализация должна использовать понятную NE2000-совместимую карту
портов. Базовый порт по умолчанию:

```text
I/O base: 0x300
range:    0x300..0x31f
```

Минимально обязательная карта:

```text
base+0x00..base+0x0f  DP8390 register window, page selected by CR.PS0/PS1
base+0x10             remote DMA data port, 8-bit access
base+0x11..base+0x1e  reserved or aliases; log unexpected access
base+0x1f             board reset port
```

Поведение reset port принять совместимым с существующими `ne1000/ne2000`:

- read `base+0x1f`: снять reset через `dp8390_reset(CLEAR_LINE)`;
- write `base+0x1f`: установить reset через `dp8390_reset(ASSERT_LINE)`;
- логировать read/write reset port.

На первом этапе можно зафиксировать `0x300`, но лучше сразу добавить DIP/input
configuration:

```text
0x300, 0x320, 0x340, 0x360
```

Если добавляется выбор базы, драйвер Sprinter DSS должен печатать найденную
базу в диагностике. По умолчанию использовать `0x300`.

## IRQ

Минимальный набор:

```text
IRQ2/9, IRQ3, IRQ4, IRQ5
```

Рекомендуемый default: `IRQ3` или `IRQ5`. Для раннего Sprinter-драйвера
принять polling-first подход: драйвер должен уметь работать без включения IMR и
без зависимости от IRQ. IRQ нужен для последующих этапов и должен быть
реализован в MAME сразу.

Callback `rtl8019as_irq_w` должен вызывать соответствующую линию ISA:

```text
irq2_w, irq3_w, irq4_w, irq5_w
```

Обязательно логировать:

```text
rtl8019as: irq3 state=1
rtl8019as: irq3 state=0
```

## Packet RAM и PROM

RTL8019AS содержит 16 КБ packet RAM. Для совместимости с NE2000-подобной
моделью использовать страницы:

```text
page size: 256 bytes
TX page:   0x40
RX start:  0x46
RX stop:   0x80
RAM range: 0x4000..0x7fff
```

Рекомендуемая схема:

```text
remote memory offset < 32          PROM/MAC area
remote memory 0x4000..0x7fff       16 KB board RAM
остальное                          0xff + log unexpected access
```

Для первого устойчивого варианта важно, чтобы MAME и будущий DSS-драйвер
согласованно трактовали PROM. Принять такой минимальный формат:

```text
PROM[0..5]   MAC address
PROM[6..31]  0x57
```

MAC должен быть deterministic enough для отладки. Рекомендуемый вариант:

```text
02:80:19:xx:xx:xx
```

Где младшие байты можно получить из `machine().rand()` или из tag/instance.
При старте устройства обязательно печатать MAC в log:

```text
rtl8019as: io=0300 irq=3 mac=02:80:19:12:34:56
```

Если агент решит реализовать классический NE2000 PROM layout с дублированием
байтов для 16-битного режима, это надо явно описать в комментариях и в
`sprinter_rtp8019_soft.md`. Для ISA8 Sprinter проще и предпочтительнее прямой
8-битный PROM.

## DP8390/RTL8019AS программная модель

Драйвер на стороне Sprinter будет использовать эти регистры. MAME-обвязка
должна отдавать их через `m_dp8390->cs_read(offset)` и принимать через
`m_dp8390->cs_write(offset, data)`.

Page 0, основные регистры:

```text
0x00 CR     Command Register
0x01 PSTART RX ring start page
0x02 PSTOP  RX ring stop page
0x03 BNRY   RX boundary page
0x04 TPSR   TX page start
0x05 TBCR0  TX byte count low
0x06 TBCR1  TX byte count high
0x07 ISR    Interrupt Status Register
0x08 RSAR0  Remote DMA start low
0x09 RSAR1  Remote DMA start high
0x0a RBCR0  Remote DMA byte count low
0x0b RBCR1  Remote DMA byte count high
0x0c RCR    Receive Configuration Register
0x0d TCR    Transmit Configuration Register
0x0e DCR    Data Configuration Register
0x0f IMR    Interrupt Mask Register
```

Page 1:

```text
0x01..0x06 PAR0..PAR5 MAC address
0x07       CURR current RX page
0x08..0x0f MAR0..MAR7 multicast hash
```

Командный регистр `CR`:

```text
bit 0 STP   stop
bit 1 STA   start
bit 2 TXP   transmit packet
bits 3..5   remote DMA command
bits 6..7   page select
```

`ISR` важные биты:

```text
0x01 PRX  packet received
0x02 PTX  packet transmitted
0x04 RXE  receive error
0x08 TXE  transmit error
0x10 OVW  overwrite warning
0x20 CNT  counter overflow
0x40 RDC  remote DMA complete
0x80 RST  reset status
```

`ISR` сбрасывается записью единиц в соответствующие биты.

`DCR` должен работать в 8-битном режиме: `WTS=0`. Это критично для Sprinter.

RTL8019AS page 3 ID:

```text
CR page select = 3
reg 0x0a -> 'P'
reg 0x0b -> 'p'
```

Эта пара должна использоваться ранним `NICINFO.EXE` для дополнительной
идентификации карты.

## Сетевое подключение хоста

Устройство должно использовать MAME `device_network_interface`, как уже делает
`dp8390_device`. Агент не должен писать отдельный TCP/IP стек внутри MAME.

Достаточно, чтобы DP8390 мог:

- отправить Ethernet frame наружу через `send`;
- получить Ethernet frame через `recv_cb`;
- положить входящий frame в RX ring;
- выставить ISR/IRQ.

Хостовая настройка сети зависит от MAME и окружения пользователя. В этом этапе
не нужно реализовывать TAP-скрипты, но нужно документировать, что проверка
реального обмена кадрами зависит от выбранного MAME network provider.

## Локальные артефакты в `mame_esp`

Кроме изменений в `../mame`, агент обязан создать в текущем каталоге:

```text
run_sprinter_rtl8019as.sh
MAME_RTL8019AS.md
```

### `run_sprinter_rtl8019as.sh`

Скрипт запуска должен быть сделан по стилю существующих helper-скриптов,
особенно `run_sprinter_esp.sh` и `run_sprinter_esp_with_serial.sh`:

- `#!/usr/bin/env bash`;
- `set -euo pipefail`;
- вычисление `SCRIPT_DIR`;
- поддержка `MAME_BIN`;
- поддержка `IMG_DIR`;
- поддержка `SPRINTER_DEBUG=0/1`;
- передача дополнительных аргументов MAME через `"$@"`;
- проверка синтаксиса через `bash -n`.

Смысл скрипта: запустить Sprinter с ISA Ethernet-картой RTL8019AS вместо
SprinterESP:

```bash
./run_sprinter_rtl8019as.sh
./run_sprinter_rtl8019as.sh -verbose
SPRINTER_DEBUG=0 ./run_sprinter_rtl8019as.sh
```

Базовая команда внутри скрипта должна использовать:

```text
-isa1 rtl8019as
```

Если в MAME-устройстве реализованы конфигурационные параметры через slot/input
settings, скрипт должен дать способ прокинуть их без редактирования файла.
Рекомендуемые переменные окружения:

```text
RTL8019AS_IOBASE=0x300
RTL8019AS_IRQ=3
RTL8019AS_MAC=02:80:19:12:34:56
RTL8019AS_NETDEV=<mame-network-provider-or-interface>
RTL8019AS_VERBOSE=0
```

Если конкретная версия MAME не позволяет менять эти параметры напрямую из
командной строки, скрипт всё равно должен:

- документировать фактические default values;
- печатать их перед запуском;
- передавать дополнительные аргументы MAME через `"$@"`, чтобы пользователь мог
  добавить нужные опции вручную;
- не создавать ложного ощущения, что переменная окружения работает, если MAME её
  пока не использует.

Пример стартового вывода скрипта:

```text
Starting Sprinter with RTL8019AS
MAME: /Users/dmitry/dev/zx/sprinter/mame/mame
I/O base: 0x300
IRQ: 3
MAC: auto
Network backend: default MAME network provider
```

Скрипт не должен запускать Jesperl, `socat` или serial bridge. Это отдельная
конфигурация: здесь тестируется Ethernet-карта, а не ESP-AT UART.

### `MAME_RTL8019AS.md`

Отдельный Markdown-файл должен описывать возможности и принципы конфигурации
эмулируемой сетевой карты.

Минимальное содержание:

- что такое `rtl8019as` в этой сборке MAME;
- как запустить Sprinter с картой;
- какие ISA-слоты можно использовать;
- default I/O base;
- возможные I/O base values, если реализован выбор;
- default IRQ;
- возможные IRQ values;
- MAC address: auto/default/configurable;
- packet RAM layout;
- DP8390/RTL8019AS register map;
- reset port;
- remote DMA data port;
- отличие от `sprinter_esp`;
- ограничения первого этапа;
- как включать `-verbose` и какие log-строки ожидать;
- как запускать будущие DSS-тесты `NICINFO.EXE`, `NICRAM.EXE`, `PING.EXE`;
- как пользователь должен репортить проблемы агенту.

В инструкции обязательно указать, что базовая программная модель для DSS:

```text
BASE+0x00..BASE+0x0f  DP8390 registers
BASE+0x10             8-bit remote DMA data port
BASE+0x1f             reset port
default BASE          0x300
default TX page       0x40
default RX ring       0x46..0x80
```

Если агент реализует дополнительные параметры MAME, например input ports или
command-line overrides, они должны быть описаны с точными командами.

## Логирование для диагностики

Добавить диагностические сообщения, чтобы по `-verbose` log или скриншоту
можно было понять, что произошло.

Минимальные сообщения:

```text
rtl8019as: start io=0300 irq=3 mac=02:80:19:12:34:56
rtl8019as: reset assert
rtl8019as: reset clear
rtl8019as: read port=0300 off=00 data=21
rtl8019as: write port=030e off=0e data=48
rtl8019as: remote read addr=4000 data=xx
rtl8019as: remote write addr=4000 data=xx
rtl8019as: tx len=60
rtl8019as: rx len=60
rtl8019as: irq state=1
rtl8019as: invalid read off=12
rtl8019as: invalid write off=12 data=xx
```

Не нужно включать побайтный log по умолчанию, иначе MAME станет слишком шумным.
Сделать logmask или compile-time флаги:

```cpp
#define LOG_IO    (1U << 1)
#define LOG_IRQ   (1U << 2)
#define LOG_DMA   (1U << 3)
#define LOG_NET   (1U << 4)
```

По умолчанию включить только старт, reset, ошибки и сетевые события. Побайтный
remote DMA log включать только при явном debug-флаге.

## Один этап реализации MAME

Агент должен выполнить всё ниже в одном этапе.

1. Изучить `ne1000.cpp/h`, `ne2000.cpp/h`, `dp8390.cpp/h`,
   `isa_cards.cpp`, `bus.lua`.
2. Создать `rtl8019as.cpp/h`.
3. Использовать `RTL8019A(config, m_dp8390, 0)`, а не `DP8390D`.
4. Реализовать ISA8 port handlers на `0x300..0x31f`.
5. Реализовать 16 КБ RAM и 8-битный remote DMA data port.
6. Реализовать PROM/MAC и лог MAC при старте.
7. Реализовать IRQ callback.
8. Добавить input ports для IRQ и, желательно, I/O base.
9. Подключить файлы к `bus.lua`.
10. Добавить `rtl8019as` в активные ISA options для Sprinter.
11. Создать `run_sprinter_rtl8019as.sh` в текущем каталоге `mame_esp`.
12. Создать `MAME_RTL8019AS.md` в текущем каталоге `mame_esp`.
13. Собрать MAME.
14. Проверить `-listslots` и `-listdevices`.
15. Проверить `bash -n run_sprinter_rtl8019as.sh`.
16. Запустить Sprinter с картой через новый скрипт и убедиться, что базовая
    машина стартует.
17. Описать изменения, команды проверки и остаточные ограничения.

## Команды проверки

Из каталога:

```text
/Users/dmitry/dev/zx/sprinter/mame_esp
```

Выполнить:

```bash
make -C ../mame -j4
../mame/mame sprinter -listslots
../mame/mame sprinter -isa1 rtl8019as -listdevices
../mame/mame sprinter -isa1 rtl8019as -verbose
bash -n run_sprinter_rtl8019as.sh
./run_sprinter_rtl8019as.sh -verbose
```

Ожидаемые признаки успеха:

```text
rtl8019as
RTL8019AS
RTL8019A Ethernet Controller
rtl8019as: start io=0300 irq=...
Starting Sprinter with RTL8019AS
```

Если `-listslots` не показывает `rtl8019as`, значит карта не добавлена в
правильный slot interface.

Если `-listdevices` не показывает дочерний `rtl8019a`, значит устройство
создано без правильного `device_add_mconfig`.

Если MAME падает при старте, проверить:

- конфликт I/O range;
- неправильный ISA interface type;
- отсутствие `set_isa_device()`;
- неправильный lifetime `required_device`;
- ошибка в `bus.lua`.

## Тестер notes для пользователя

После реализации MAME пользователь проверяет:

```bash
../mame/mame sprinter -listslots
```

Ожидаемо в списке ISA-слотов есть:

```text
rtl8019as
```

Потом:

```bash
../mame/mame sprinter -isa1 rtl8019as -listdevices
```

Ожидаемо виден дочерний RTL8019A/DP8390 controller.

Потом обычный запуск:

```bash
./run_sprinter_rtl8019as.sh -verbose
```

Ожидаемо:

- Sprinter загружается;
- клавиатура работает;
- мышь, если была включена до этого, не ломается;
- в log есть строка старта `rtl8019as`;
- нет постоянного потока invalid port access до запуска тестовой утилиты.
- `MAME_RTL8019AS.md` содержит фактические параметры карты и команды запуска.

Пользователь передаёт агенту:

- команду запуска;
- полный фрагмент log вокруг `rtl8019as`;
- скриншот ошибки, если MAME упал или завис;
- вывод `-listslots` и `-listdevices`, если карта не появилась.

Агент после отчёта пользователя сначала чинит найденную проблему, затем
повторяет этот же набор проверок.

## Критерии готовности MAME-этапа

Этап считается завершённым, когда:

- MAME собирается;
- `rtl8019as` доступен в ISA-слотах Sprinter;
- карта стартует без падения MAME;
- DP8390-регистры доступны по `0x300..0x30f`;
- remote DMA data port доступен по `0x310`;
- reset port доступен по `0x31f`;
- page 3 ID `P`/`p` читается через RTL8019A core;
- создан и проходит `bash -n` скрипт `run_sprinter_rtl8019as.sh`;
- создана инструкция `MAME_RTL8019AS.md` по запуску, базовому адресу, IRQ,
  MAC, register map и ограничениям;
- log содержит достаточно информации для диагностики;
- будущая DSS-утилита `NICINFO.EXE` сможет обнаружить карту, прочитать ID,
  MAC/PROM и базовые регистры.

## Ограничения первого MAME-этапа

Допустимые ограничения:

- фиксированный I/O base `0x300`, если выбор базы не успели сделать;
- polling-first проверка без обязательного подтверждения IRQ из DSS;
- отсутствие отдельного UI для MAC address;
- отсутствие хостовых TAP/helper-скриптов.

Недопустимые ограничения:

- карта не видна в `sprinter -listslots`;
- карта реализована только как ISA16;
- data port требует 16-битных операций;
- драйвер должен использовать сторонний x86 packet driver;
- неизвестные port access приводят к падению MAME;
- добавление карты ломает существующие Sprinter ISA-устройства.
