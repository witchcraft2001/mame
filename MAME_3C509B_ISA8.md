# План доведения эмуляции 3Com 3C509B-TPO до рабочего ISA8-уровня в MAME

## 1. Назначение документа

Этот документ описывает доработки MAME, необходимые для аппаратно-достоверной
эмуляции сетевой карты 3Com EtherLink III 3C509B-TPO в восьмибитном ISA-слоте
компьютера Sprinter.

Целевой клиент — проект
[`../sprinter-3C509B`](../sprinter-3C509B/README.md). Его
[`specs.md`](../sprinter-3C509B/specs.md) задаёт обязательные сценарии
обнаружения, инициализации, polling TX/RX, loopback и последующей работы
сетевого стека. Эмулятор должен воспроизводить поведение регистров, команд и
FIFO физической карты. Нельзя подменять карту специальным API, виртуальным
пакетным драйвером или Sprinter-специфичным перехватом сетевых функций.

Документ составлен по состоянию дерева MAME на 2026-08-29. В дереве уже есть
начальная модель:

- [`src/devices/bus/isa/3c509b.cpp`](src/devices/bus/isa/3c509b.cpp);
- [`src/devices/bus/isa/3c509b.h`](src/devices/bus/isa/3c509b.h);
- регистрация `3c509b` в
  [`src/devices/bus/isa/isa_cards.cpp`](src/devices/bus/isa/isa_cards.cpp);
- подключение исходников в [`scripts/src/bus.lua`](scripts/src/bus.lua).

Существующая модель уже собирается и видна как `-isa1 3c509b`, но пока не
может считаться эталоном для разработки драйвера: несколько её упрощений
меняют наблюдаемое программой поведение карты.

## 2. Источники и границы clean-room

Приоритет источников:

1. [3Com EtherLink III Drivers Technical Reference, 09-0398-002B](https://www.ardent-tool.com/NIC/3c5x9b_Technical_Reference.pdf).
2. Дампы и трассы реальной 3C509B-TPO в ISA8-слоте Sprinter.
3. Спецификация клиента
   [`../sprinter-3C509B/specs.md`](../sprinter-3C509B/specs.md).
4. [Linux 3c509 driver](https://github.com/torvalds/linux/blob/master/drivers/net/ethernet/3com/3c509.c)
   и зафиксированный в клиентском проекте Nestor DOS packet driver — только
   как clean-room-источники наблюдаемой последовательности операций.

GPL-код Linux/Nestor нельзя копировать в BSD-3-Clause реализацию MAME. Из него
разрешено вынести таблицу поведения или псевдокод, после чего самостоятельно
реализовать модель по руководству 3Com.

Версия 1 модели ограничена одной 3C509B-TPO, classic 3Com ID activation,
10BASE-T, half duplex и polling-драйвером Sprinter. Не требуются BNC/AUI,
boot ROM, полноценный ISA PnP manager и запись EEPROM.

## 3. Правильная граница между Sprinter и картой

Эмуляция должна оставаться двухслойной:

```text
Z80, адресное пространство Sprinter
        |
        | окно 0xC000..0xFFFF
        v
sprinter_state::isa_r/isa_w
        |
        | один 8-битный ISA I/O cycle
        v
isa8_device::AS_ISA_IO выбранного слота
        |
        | ID port или base+0x00..base+0x0F
        v
isa8_3c509b_device
        |
        | Ethernet frame без прикладного TCP/IP
        v
device_network_interface / pcap
```

Текущий Sprinter уже отображает ISA I/O `0x0000..0x3FFF` в память Z80
`0xC000..0xFFFF` и выполняет отдельный восьмибитный цикл для каждого чтения или
записи. Поэтому:

- регистры 3C509B устанавливаются только в `AS_ISA_IO`;
- в `sprinter.cpp` не добавляются прямые обработчики регистров 3C509B;
- низкий и высокий байты 16-битного регистра видны клиенту как два
  последовательных обращения по адресам `0xC000 + port` и
  `0xC000 + port + 1`;
- TX/RX FIFO остаются байтовыми потоками карты;
- TCP/IP, ARP, DHCP и прочие протоколы исполняются на Z80, а не внутри MAME.

Обе ISA-шины Sprinter уже являются `isa8_device`. Карта должна одинаково
работать в `-isa0` и `-isa1`; типичный тест использует `-isa1`, поскольку в
`-isa0` по умолчанию стоит `zxbus_adapter`.

## 4. IRQ: намеренно не подключать к CPU Sprinter

Sprinter-клиент работает только polling-методом. Отсутствие связи IRQ карты с
CPU — правильная особенность платформы.

- Не подключать `irq*_callback` ISA-шины Sprinter к `m_irqs` или Z80.
- Не добавлять IRQ в конфигурацию клиента.
- Внутри 3C509B всё равно моделировать `Interrupt Mask`, `Read Zero Mask`,
  `Interrupt Latch` и причины событий: polling-код читает их через Status.
- Универсальная модель карты может выставлять соответствующую линию на
  `isa8_device`; у Sprinter эта линия должна оставаться неподключённой.
- Значение IRQ из EEPROM сохраняется как характеристика карты, но не влияет на
  успешность её обнаружения и работы на Sprinter.

## 5. Аудит текущей реализации

| Область | Сейчас | Требуемое поведение | Приоритет |
|---|---|---|:---:|
| Регистрация устройства | Карта включена в сборку и ISA8 slot options | Сохранить, проверить `-listslots` и `-listdevices` | P0 |
| Диапазон I/O base | Таблица из 8 адресов, часть значений пропущена | Все 31 базы `0x200..0x3E0`, шаг `0x10` | P0 |
| EEPROM word `08h` | База кодируется сокращённым индексом и добавляется неверный `0x2000` | Bits 4..0 = `(base-0x200)/0x10`; TPO XCVR = `00`, ROM выключен | P0 |
| Configuration Control | После reset `0x4F00`, запись уничтожает read-only POR bits | Для ISA 3C509B-TPO POR-поля читаются корректно; изменяются только writable bits | P0 |
| EEPROM timing | Busy всегда ноль, чтение мгновенное | `EBY` и окончание read не раньше документированного цикла | P0 |
| Internal Configuration | Только 16 бит, после reset ноль | 32-битный регистр из EEPROM words `12h/13h`, включая RAM size/partition и classic activation | P0 |
| ID state machine | Базовая LFSR есть, tag учитывается частично | Два нуля, 255 байт, EEPROM delay, tag suppression, activate/reset semantics | P0 |
| ISA8 word access | Общий low/high latch уже есть | Уточнить исключения и side effects; command выполняется записью high byte | P0 |
| FIFO offsets | Потоком считаются offsets `00..03` | Поток только через младший байт `+00` или `+02`; `+01/+03` недопустимы | P0 |
| Window 2 station address | Байты в каждой паре переставлены | `base+0 = Address 0`, `base+1 = Address 1` и т. д. | P0 |
| Window 5 | Значения привязаны к неверным offsets | Привести offsets к Figure 5-6 руководства | P0 |
| Net Diagnostic | Reset-значение `0x2000` выглядит как включённый controller loopback | Writable loopback bits = 0, ASIC revision = 2 в bits 5..1 | P0 |
| TX окончание загрузки | Packet data округляется до WORD | Preamble + data обязательно округляются до DWORD | P0 |
| TX completion | Пакет и статус завершаются синхронно при последней записи | Завершение по 10 Mbps timer/callback, корректные TX Free и status | P0 |
| TX Status | Чтение удаляет entry, глубина 16 | Чтение не удаляет; запись в `base+0x0B` pop; ровно 31 entry | P0 |
| RX lifetime | Последний прочитанный байт автоматически удаляет пакет | Пакет остаётся head до команды `RX_DISCARD` | P0 |
| Back-to-back RX | Может потеряться следующий пакет из-за auto-pop/double discard | Один discard удаляет ровно один head packet | P0 |
| RX Early | Сравнение выполнено в обратную сторону | Событие при количестве байт, превышающем threshold; >1792 означает disable | P0 |
| CIP/reset | Команды выполняются мгновенно, bit 12 не виден | Асинхронные команды и EEPROM обслуживаются таймерами | P0 |
| FIFO capacity | TX/RX Free возвращают константу `0x07FF` | Учёт 32 KiB SRAM, partition, занятых байт и thresholds | P1 |
| Window 6 bytes | RX/TX byte counters поменяны местами | `+0A` = RX bytes, `+0C` = TX bytes | P1 |
| RX filter | Group и broadcast трактуются независимо | Group reception также разрешает broadcast; promiscuous включает всё | P1 |
| Link/media | Link beat заявлен постоянно | Связать media status с backend/loopback или явным тестовым состоянием | P1 |
| Interrupt latch | Автоматически очищается при исчезновении причины | Latch держится до `AckIntr(IntLatch)` | P1 |
| Save state | Переменные сохранены частично, `std::queue` и cursor теряются | Сохраняемое фиксированное представление FIFO и все таймерные состояния | P1 |

P0 — необходимо для этапов 3–6 клиентского проекта. P1 — требуется до
признания модели аппаратно-достоверной и перед длительными сетевыми тестами.

## 6. Детальные требования к модели

### 6.1. I/O decode и конфигурация экземпляра

Операционное окно всегда занимает 16 портов. Для ISA mode:

```text
index = AddressConfiguration & 0x1F
index 0x00..0x1E: base = 0x200 + index * 0x10
index 0x1F: EISA mode, для Sprinter не активировать
```

Установленный сейчас общий handler `0x0200..0x03FF` можно сохранить и
фильтровать доступ по текущей базе. Это не должно превращаться в ответ карты на
весь диапазон: вне активных `base..base+0x0F` чтение возвращает pull-up
`0xFF`, запись не имеет эффекта.

MAME input setting должен позволять выбрать все 31 значения или хранить
5-битный base index с корректными строками UI. Default для стенда — `0x300`,
то есть index `0x10`, а не `0x04`.

Опция `Auto-activate at reset` допустима только как отладочное расширение.
Основной acceptance path обязан использовать `Real (ID sequence required)`.

### 6.2. MAC и EEPROM

EEPROM остаётся read-only с точки зрения версии 1. Не добавлять NVRAM-запись и
не делать изменение постоянной конфигурации условием теста.

Минимально согласованный образ EEPROM:

| Word | Содержимое |
|---:|---|
| `00h..02h` | Factory MAC, два сетевых байта на слово |
| `03h` | Product ID `0x9550` для 3C509B-TPO |
| `07h` | Manufacturer ID `0x6D50` |
| `08h` | Address Configuration с XCVR=TPO и правильным 5-bit base index |
| `09h` | Resource Configuration; IRQ только справочный для Sprinter |
| `0Ah..0Ch` | OEM MAC |
| `0Dh` | Software Information, согласованный с link beat policy |
| `0Eh` | Compatibility Word |
| `0Fh` | Primary checksum |
| `10h` | Capabilities `0x2083` |
| `12h` | Internal Configuration low word, 32 KiB SRAM |
| `13h` | Internal Configuration high word: допустимая partition и classic activation |
| `14h` | Secondary Software Information, revision 1 для B |
| `17h` | Secondary checksums |

Для 32 KiB byte-wide SRAM нижнее слово Internal Configuration должно отражать
RAM SIZE `010b`. Partition выбирается документированным значением; практичный
default — 1:1. Чтобы не рекламировать неэмулированный PnP activation path,
поле ISA Activation Select следует поставить в `ISA contention only`.

Default MAC генерируется один раз при старте экземпляра. Затем `build_eeprom()`
должен брать `get_mac()`, а не безусловно перезаписывать его. Это сохраняет
совместимость с MAME `Configure Network Devices`, где пользователь может
назначить MAC. EEPROM, station address и network interface обязаны показывать
одно и то же значение после reset.

Обычное EEPROM-чтение через Window 0:

1. Запись команды `0x80 | address` в `base+0x0A/0x0B`.
2. Установка `EBY` (bit 15 EEPROM Command).
3. Таймер не менее 162 мкс.
4. Заполнение EEPROM Data и снятие `EBY`.

ID-port EEPROM read также не должен отдавать новое слово до окончания цикла.
Клиент ждёт с конечным тайм-аутом; мгновенное завершение скроет ошибки в его
polling-коде.

Команды erase/write в версии 1 можно безопасно игнорировать после корректного
Busy/command decode. Они не должны изменять EEPROM исподволь.

### 6.3. Classic ID-port activation

Поддерживаемые ID ports: `0x100..0x1F0`, шаг `0x10`; default клиента `0x110`.

Последовательность состояния:

```text
POR/AUTOINIT --310 us--> ID_WAIT
ID_WAIT --two zero writes + 255-byte LFSR--> ID_CMD
ID_CMD --EEPROM read--> shift 16 bits MSB first on data bit 0
ID_CMD --E0..FE--> set base index, ACTIVE, ID_WAIT
ID_CMD --FF--> ACTIVE at EEPROM base, ID_WAIT
```

LFSR начинается с `0xFF`, использует полином `0xCF`, содержит 255 выходных
байтов и заканчивается `0x98`. Любой неверный байт возвращает приём
последовательности к ожидаемому `0xFF`. Ноль в LFSR не встречается.

Нужно исправить tag semantics:

- `D0..D7` задаёт tag;
- карта с ненулевым tag не отвечает на contention reads;
- ранее найденная карта игнорирует предназначенные для новых карт tag/test
  команды согласно руководству;
- для одного экземпляра Sprinter это всё равно проверяется отрицательными
  тестами, чтобы ID state machine не была только «магической разблокировкой».

После аппаратного/global reset должен пройти AUTOINIT delay. В этот период
операционное окно невидимо, а преждевременный ID sequence не принимается.

### 6.4. ISA8 low-byte/high-byte frontend

Общее правило 3C509B для 16-битного регистра:

```text
read  base+even       -> low byte, зафиксировать согласованный high byte
read  base+even+1     -> high byte

write base+even       -> зафиксировать low byte
write base+even+1     -> high byte, применить полное 16-bit значение
```

Между парными циклами не должно быть другого доступа к карте. Новый доступ к
другому регистру инвалидирует незавершённую пару. Command word в
`base+0x0E/0x0F` выполняется только после записи high byte.

Исключения:

- Status разрешает отдельное 8-битное чтение любого байта;
- Timer (`Window 1/+0A`) — отдельный 8-битный read;
- TX Status (`Window 1/+0B`) — отдельный 8-битный read/write;
- Window 6 counters `+00..+08` — отдельные байтовые регистры;
- TX/RX PIO — поток, а не парный регистр.

Для PIO на ISA8 допустимы младшие байты `base+0x00` и `base+0x02`. Доступы к
`base+0x01` и `base+0x03` запрещены: чтение возвращает `0xFF`, запись
игнорируется и может логироваться в отладочном режиме. Клиент должен
использовать `base+0x00` как единый байтовый FIFO port.

### 6.5. Карта окон и регистров

Минимальный набор, который обязан работать до запуска клиента:

| Window | Offset | Register | Требуемое поведение |
|:---:|:---:|---|---|
| all | `0E` | Command/Status | low/high command, window+CIP+event status |
| 0 | `00` | Manufacturer ID | `0x6D50` |
| 0 | `02` | Product ID | `0x9550` |
| 0 | `04` | Configuration Control | read-only POR bits, writable ENA/RST |
| 0 | `06` | Address Configuration | полные 5 bits базы |
| 0 | `08` | Resource Configuration | IRQ/SRDY fields |
| 0 | `0A` | EEPROM Command | EBY и read command |
| 0 | `0C` | EEPROM Data | результат после Busy |
| 1 | `00`,`02` | RX/TX PIO | только младшие байтовые offsets |
| 1 | `08` | RX Status | head packet, length/error/incomplete |
| 1 | `0A` | Timer | 3.2 мкс на tick, saturation `0xFF` |
| 1 | `0B` | TX Status | read head; write pops one entry |
| 1 | `0C` | TX Free | свободные байты, округлённые до DWORD |
| 2 | `00..05` | Station Address | Address 0 в low byte первой пары |
| 3 | `00`,`02` | Internal Configuration | low/high 16-bit halves 32-bit регистра |
| 3 | `05` | ROM Control | 8-bit; boot ROM может оставаться disabled |
| 3 | `0A` | RX Free | реальная свободная ёмкость RX FIFO |
| 3 | `0C` | TX Free | точное, неокруглённое значение |
| 4 | `04` | FIFO Diagnostic | хотя бы overrun/underrun/reset-required flags |
| 4 | `06` | Net Diagnostic | loopback, TX/RX state, ASIC revision 2 |
| 4 | `08` | Ethernet Status | минимальные TX/RX error/state bits |
| 4 | `0A` | Media Status | TPO, link, link beat, jabber, CRC strip |
| 5 | `00` | TX Start Threshold | readback команды |
| 5 | `02` | TX Available Threshold | readback команды |
| 5 | `06` | RX Early Threshold | readback команды |
| 5 | `08` | RX Filter | lower 4 bits |
| 5 | `0A` | Interrupt Mask | readback команды |
| 5 | `0C` | Read Zero Mask | readback команды |
| 6 | `00..08` | Byte statistics | read-and-zero только при disabled stats |
| 6 | `0A` | RX bytes | 16-bit read-and-zero |
| 6 | `0C` | TX bytes | 16-bit read-and-zero |

Текущую Window 5 следует перестроить целиком: сейчас `RX filter`, thresholds и
часть readback registers находятся не на документированных offsets.

Station Address в Window 2 хранится в сетевом порядке как массив байтов, но
регистровая пара little-endian относительно ISA cycles:

```text
word at +00 = Address1 << 8 | Address0
word at +02 = Address3 << 8 | Address2
word at +04 = Address5 << 8 | Address4
```

### 6.6. Команды, Status и Command-in-Progress

Нужно разделить:

- физические причины событий;
- видимость причины через Read Zero Mask;
- разрешение IRQ через Interrupt Mask;
- latched IRQ state;
- занятую командную машину (`CIP`, Status bit 12).

`Interrupt Latch` нельзя автоматически очищать только потому, что исчезла
причина. Он снимается `AckIntr(IntLatch)`. Для Sprinter это проверяется чтением
Status, даже если IRQ line не подключена.

Минимально обязательные команды:

- Global Reset;
- Select Window;
- RX Disable/Enable/Reset/Discard;
- TX Disable/Enable/Reset;
- Request/Acknowledge Interrupt;
- Set Interrupt Mask;
- Set Read Zero Mask;
- Set RX Filter;
- Set RX Early Threshold;
- Set TX Available Threshold;
- Set TX Start Threshold;
- Statistics Enable/Disable.

Во время CIP новые команды не принимаются. Для операций без опубликованной
точной длительности используется короткий ненулевой детерминированный timer,
но не мгновенное изменение. Обязательные ориентиры:

- AUTOINIT/перечитывание EEPROM — около 250–310 мкс;
- EEPROM read — не менее 162 мкс;
- RX Discard выставляет CIP до перехода к следующему RX packet;
- сетевой TX завершается по таймеру `device_network_interface` на скорости
  10 Mbit/s, а не внутри последней FIFO write.

Драйвер клиента всё равно использует тайм-ауты, поэтому тесты должны покрывать
и нормальное снятие CIP, и искусственно зависший fault-injection режим.

### 6.7. TX FIFO и TX Status

Формат одного TX packet:

```text
word 0: bit15 Notify, bit13 Disable CRC Generation, bits10..0 Length
word 1: 0 для совместимости
data:   Length bytes
pad:    до следующей DWORD boundary
```

Запуск передачи происходит после получения
`4 + align4(length)` байтов, а не `4 + align2(length)`. Pad bytes не входят в
передаваемую Ethernet length. Кадр короче 60 байт физическая карта умеет
дополнить автоматически; клиентский проект дополнительно нормализует такие
кадры до 60 байт до записи preamble, и модель должна корректно поддерживать
оба случая.

`TX Free` должен уменьшаться при загрузке FIFO и восстанавливаться после
фактического завершения. В Window 1 значение округляется вниз до DWORD, в
Window 3 возвращается точное значение. Нельзя всегда возвращать `0x07FF`.

TX completion:

1. Передать frame через `device_network_interface::send()`.
2. Дождаться `send_complete_cb`.
3. Освободить соответствующее место FIFO.
4. Для успешного пакета создать TX Status только если в preamble был Notify.
5. Для ошибки создать TX Status независимо от Notify и выключить transmitter
   там, где этого требует контроллер.
6. Обновить TX Available threshold/event и statistics.

TX Status stack содержит ровно 31 entry. Read `base+0x0B` только показывает
текущую entry. Write `base+0x0B` удаляет ровно одну ранее прочитанную complete
entry. Когда stack пуст, `TX_COMPLETE` в общем Status снимается. При overflow
выставляется соответствующий bit и transmitter отключается.

Для успешного Notify completion TX Status должен содержать как минимум bits
`Complete` и `Interrupt on Successful Transmission Requested`, а не только
`Complete`.

Jabber и underrun требуют состояния `TX Reset required`, TX disable и
последовательности recovery `TX Reset` → окончание команды → `TX Enable`.
Нужен управляемый fault injection для этих ветвей без зависимости от реальной
ошибки pcap.

### 6.8. RX FIFO и явный RX Discard

Ключевой инвариант: чтение последнего байта кадра не удаляет пакет.

Для каждого принятого packet хранить:

- исходную длину без FIFO padding;
- data с padding до DWORD;
- RX Status error/type;
- текущий byte cursor;
- занятое место FIFO.

RX Status показывает только head packet. Поле RX Bytes уменьшается по мере
чтения фактических данных. Чтение допустимого padding может перевести 11-bit
count в отрицательные значения, но чтение за пределами padding вызывает RX
underrun/Adapter Failure.

После чтения данных head packet остаётся на месте. Только команда
`RX_DISCARD`:

1. выставляет CIP;
2. отбрасывает остаток data/padding текущего packet;
3. удаляет ровно одну RX Status entry;
4. освобождает место RX FIFO;
5. делает видимым следующий packet;
6. обновляет `RX_COMPLETE` и RX Early;
7. снимает CIP.

Это обязательно проверяется двумя и десятью кадрами подряд. Сценарий
«прочитать первый → RX_DISCARD → прочитать второй» не должен терять второй
кадр. Двойной discard должен удалить два кадра — именно поэтому клиент после
успешного `READ_FRAME` не вызывает дополнительный `DISCARD_FRAME`.

RX FIFO ограничен выбранной partition 32 KiB SRAM. При нехватке места нужно
учитывать dropped/overrun statistics, не раздувать неограниченный
`std::queue`.

Приём от `device_network_interface` желательно разделить на
`recv_start_cb`/`recv_complete_cb`: packet становится complete после
моделируемого времени передачи. Упрощённый P0-вариант может публиковать только
полностью принятые кадры, но не должен ошибочно генерировать RX Early для уже
complete packet.

### 6.9. Filters, TPO media и loopback

RX Filter bits:

- individual — только текущий Station Address;
- group — multicast и, согласно 3Com, также broadcast;
- broadcast — broadcast;
- promiscuous — все адреса.

После reset filter равен нулю. Клиент устанавливает `individual + broadcast`
до `RX Enable`.

Для 3C509B-TPO:

- XCVR сообщает twisted pair;
- Media Status writable bits 7/6 управляют link beat и jabber protection;
- CRC Strip Disable остаётся writable capability B-revision;
- read-only Link Beat не должен быть безусловной константой, если network
  backend отключён;
- отсутствие link не должно зависать: TX/RX-код клиента заканчивается по
  тайм-ауту и получает диагностируемое состояние.

Net Diagnostic должен возвращать ASIC revision 2 в bits 5..1. Writable
loopback bits 15..12 хранятся отдельно от read-only TX/RX state bits.

Для `EL3LB` минимум необходим FIFO/controller loopback: переданный кадр
возвращается в обычный RX FIFO и затем подчиняется тем же RX Status,
byte-stream и RX Discard правилам. `set_loopback()` MAME можно использовать как
транспорт, но регистровое состояние, filtering, completion и статистику всё
равно обслуживает 3C509B device.

### 6.10. Statistics и диагностические состояния

Statistics собираются только после `Statistics Enable`. Перед чтением клиент
выдаёт `Statistics Disable`. Чтение сбрасывает конкретный counter.

Исправить порядок двух 16-битных counters:

```text
Window 6/+0A = Total bytes received successfully
Window 6/+0C = Total bytes transmitted successfully
```

Для диагностики как минимум моделировать:

- TX/RX frames OK;
- TX/RX bytes OK;
- RX FIFO overrun/drop;
- FIFO RX underrun и TX overrun;
- TX reset required;
- Update Statistics при достижении порога counter.

## 7. Представление состояния и save state

Переменные фиксированного размера предпочтительнее несохраняемого
`std::queue<rx_packet>`. Возможная структура:

```text
RX SRAM/FIFO byte array
RX descriptor array: offset, data_length, padded_length, status
head, tail, packet_count, cursor, used_bytes

TX SRAM/FIFO byte array or bounded packet descriptors
current preamble/data cursor, queued/active descriptors, used_bytes

TX status[31], stack depth
```

Сохраняются:

- EEPROM и pending EEPROM operation;
- ID state/LFSR/position/tag/selected port;
- low/high latches;
- window/config/status/masks/thresholds;
- TX/RX enable и loopback/media state;
- FIFO bytes/descriptors/cursors;
- TX Status stack;
- statistics;
- состояние и остаток device timers.

После load состояние IRQ/status пересчитывается без потери latch. Network
backend может потерять внешний кадр, который ещё не начал попадать в карту, но
уже принятые в SRAM кадры обязаны сохраниться.

## 8. Логирование и fault injection

Добавить раздельные log masks, оставив побайтовый FIFO log выключенным по
умолчанию:

- ID/LFSR/activation;
- command/CIP/reset;
- register I/O;
- TX/RX packet summary;
- FIFO/error;
- IRQ/status.

Packet summary содержит slot tag, base, length, status и причину drop. Не
логировать содержимое пользовательских пакетов по умолчанию.

Для воспроизводимой проверки negative paths полезны debug input settings или
закрытый test hook:

- hold CIP;
- force TX underrun;
- force jabber;
- force bad RX CRC/runt/oversize;
- ограничить видимую ёмкость FIFO;
- принудительно link down.

Fault injection выключен по умолчанию и не меняет normal profile.

## 9. Последовательность реализации

### Шаг A. Исправить discovery/configuration

- Полные 31 I/O bases и формульный decode.
- Согласованный MAC/EEPROM и checksums.
- 32-bit Internal Configuration.
- AUTOINIT и EEPROM Busy timers.
- Полная classic ID state machine.

Результат: будущий `EL3INFO.EXE` читает `6D50`, `9550`, MAC и base одинаково в
MAME и на реальной карте.

### Шаг B. Исправить ISA8 register frontend

- Low/high semantics и исключения.
- FIFO только `+00/+02` low-byte ports.
- Правильные Window 2/3/4/5/6 offsets.
- Command/Status/CIP и reset defaults.

Результат: `EL3REG.EXE` получает согласованный snapshot окон 0–6.

### Шаг C. Перестроить FIFO datapath

- 32 KiB SRAM/partition и free-byte accounting.
- TX DWORD padding и asynchronous completion.
- 31-entry TX Status с write-pop.
- RX packet lifetime только до RX Discard.
- Back-to-back RX.

Результат: `EL3LB.EXE` проходит длины 60, 61, 62, 63 и 1514 с несколькими
кадрами подряд.

### Шаг D. Media/network integration

- TPO media/link state.
- pcap TX/RX через `device_network_interface`.
- Link-down и TX/RX error recovery.
- Statistics и threshold events.

Результат: `EL3TX.EXE`/`EL3RX.EXE` обмениваются Ethernet II кадрами с host.

### Шаг E. Save state и регрессии

- Сохраняемое представление FIFO.
- Unit/headless tests и fault injection.
- Длительный polling stress.

Результат: save/load с queued RX/TX не ломает следующий packet, 100 циклов
INIT/DONE и длительный обмен проходят без зависания.

## 10. Матрица обязательных регрессий

### 10.1. Host/C++ tests

- 255 LFSR bytes, первый `FF`, последний `98`, повтор после ошибки.
- Encode/decode всех bases `0x200..0x3E0` и reject index `0x1F`.
- EEPROM golden vector, MAC byte order, primary/secondary checksums.
- Trace 16-bit ISA8 access: low строго перед high; command только на high.
- Отдельные byte accesses Status/Timer/TX Status/statistics.
- FIFO aliases `+00/+02` и отсутствие side effects у `+01/+03`.
- TX required bytes: `4 + align4(length)` для 14, 42, 59, 60, 61, 62, 63,
  1514.
- TX Status: read/read не pop; read/write pop; overflow на entry 32.
- RX: чтение data не pop; один discard — один packet; 2 и 10 packet queues.
- RX Early boundary: threshold-1, threshold, threshold+1, disabled >1792.
- EEPROM Busy и CIP видны хотя бы на одном emulated tick.
- Window 2 station byte order и Window 5 exact offsets.
- Save/load с partially read RX и непустым TX Status stack.

### 10.2. MAME/Sprinter tests

```bash
./mame sprinter -listslots
./mame sprinter -isa1 3c509b -listdevices
./mame sprinter -isa0 3c509b -listdevices
./mame -listnetwork
```

Запуск classic activation profile:

```bash
./mame sprinter \
  -isa1 3c509b \
  -flop1 ../sprinter-3C509B/distr/sprinter-3c509b.img \
  -networkprovider pcap \
  -verbose
```

Конкретный host interface выбирается через меню MAME `Configure Network
Devices` и сохраняется в локальном `cfg/sprinter.cfg`. Имя вроде `feth0` не
зашивается в исходник или общий тест.

Выполнить последовательно по мере появления приложений клиента:

1. `EL3INFO` — ID sequence, EEPROM, activation.
2. `EL3REG` — окна, byte order, masks, media.
3. `EL3LB` — FIFO/controller loopback и back-to-back packets.
4. `EL3TX`/`EL3RX` — физические Ethernet II frames через pcap.
5. `PING` — end-to-end polling ARP/IPv4/ICMP.

Каждый test log должен содержать версию MAME/EXE, slot, ID port, base, MAC,
последние Status/RX Status/TX Status и однозначный `RESULT OK`/`RESULT FAIL`.

### 10.3. Сравнение с реальной картой

Для одинаковых операций снять компактные access traces:

```text
operation, slot, ISA port, R/W, byte value, elapsed ticks
```

Сравниваются:

- EEPROM words `00..17` и checksums;
- reset values окон 0, 3, 4 и 5;
- длительность EEPROM Busy/CIP в допустимых пределах;
- TX Free/RX Free после reset и после одного packet;
- TX Status read/write-pop;
- RX Status до/после data read и RX Discard;
- loopback lengths 60/61/62/63/1514;
- два packet подряд.

Не следует требовать точного совпадения внутренней ёмкости/overhead, если
реальная ревизия EEPROM отличается. Драйвер не должен зависеть от exact empty
TX/RX Free; обязательны совпадение порядка состояний и достаточная ёмкость.

## 11. Definition of Done

Модель готова для разработки и проверки сетевого комплекта Sprinter только
когда одновременно выполнены условия:

- карта доступна в обоих ISA8 slots и не требует изменений `sprinter.cpp` для
  своей регистровой модели;
- classic ID sequence и EEPROM работают с реальными задержками и тайм-аутами;
- поддержаны все 31 ISA base;
- каждое 16-bit обращение клиента действительно является двумя 8-bit cycles;
- high FIFO offsets не двигают FIFO;
- TX использует DWORD padding, конечный TX Free и asynchronous completion;
- TX Status читается без pop и удаляется записью, stack имеет 31 entry;
- RX data read не удаляет packet, один RX Discard удаляет ровно один packet;
- back-to-back RX не теряет следующий кадр;
- reset/CIP/threshold/filter/media semantics проходят регрессии;
- IRQ не подключён к CPU Sprinter и polling остаётся единственным обязательным
  режимом;
- pcap TX/RX и internal loopback работают через ту же hardware model;
- save/load не повреждает сохранённые FIFO/status states;
- `EL3INFO`, `EL3REG`, `EL3LB`, `EL3TX`, `EL3RX` и затем `PING` завершаются с
  `RESULT OK`;
- результаты MAME сопоставлены с трассами реальной 3C509B-TPO.

До выполнения этих пунктов MAME полезен как экспериментальная заготовка, но не
как источник истины для поведения драйвера Sprinter.
