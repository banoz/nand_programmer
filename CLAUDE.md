# NANDO — Agent Operating Rules & Project Knowledge

---

# Part 1: Operating Rules

## CRITICAL: Memory and Context Constraints
- **Context Ceiling Alert:** We are running on a local hardware engine with tight context restraints. You MUST minimize token usage on every turn.
- **Do Not Ingest Broadly:** NEVER use sweeping repository searches, large grep passes, or read entire multi-thousand-line source files in a single step.
- **Aggressive Memory Clearing:** If your task requires a sequence of tool calls or file reviews, you MUST explicitly advise the user to run `/clear` between sub-tasks to flush the active token cache and protect the hardware buffer.

## Workflow: Sequence-Driven Task Splitting
When assigned any complex directive, implementation task, or architectural summary, follow this protocol:

1. **Phase 1: Discovery & Scoping (Max 1-2 file reads)**
   - Read ONLY high-level entry points (`CMakeLists.txt`, `Makefile`, `main.c`, device descriptors).
   - Stop and output a bulleted list of the exact sub-units needed.

2. **Phase 2: Micro-Chunk Execution**
   - Execute exactly ONE small atomic unit at a time.
   - Never chain more than 2 tool/shell calls in a single turn.

3. **Phase 3: Intermittent Verification & Handoff**
   - After each atomic change, trigger a targeted compilation check for just that build object.
   - Stop, report status, and request permission before the next block.

## File Inspection Constraints
- Any C source/header over 150 lines (especially generated STM32CubeHAL files) MUST be read in targeted line-range slices. Never view the whole file.
- Use `.gitignore`/`.claudeignore` to block build artifacts (`build/`, `obj/`, `.elf`, `.bin`, `.hex`, `.map`, `.o`, `.d`).

## Agent Policy
1. **Immutability by Default** — do not modify files unless explicitly requested and confirmed. Propose patches and await approval.
2. **Single Source of Truth** — always read current files from disk. Never assume the code matches an earlier snapshot.
3. **Minimal Prompt Context** — include only relevant snippets or explicit struct definitions.

## Runtime Checklist
1. Read the latest files relevant to the task.
2. Build context from current contents, not prior snapshots.
3. Do not modify files unless explicitly requested.
4. Provide patches or full file contents and await approval.
5. Include file paths when summarizing context.

## C & STM32 Firmware Guidelines
- **Build:** standard embedded toolchains (`make`, `cmake`). Always monitor `-Wall -Wextra`.
- **Code Placement Isolation:** in CubeMX-managed files, write only inside `/* USER CODE BEGIN ... */` markers.
- **Peripheral Layering:** keep HAL/LL hardware APIs distinct from application logic.
- **Defensive Resource Handling:** fixed-width types from `<stdint.h>`, `volatile` for hardware registers and ISR flags, explicit clock enables, bounds-checked register pools.
- Keep changes tightly scoped to avoid Flash/SRAM footprint growth.

## Bash Guidelines
- macOS-compatible Bash.
- No permission needed for read-only commands (builds, static analysis, file lookups).
- **NEVER use `rm`/`rm -rf`** — use `trash`. (Note: `trash` is not installed on this machine; move unwanted files to `$TMPDIR` instead.)

## Skill Registry and Layout
Skills live in `~/.claude/skills/` or `.claude/skills/`, one directory per slash command, each containing an uppercase `SKILL.md`:

```
.claude/skills/
└── <skill-name>/          # invoked as /<skill-name>
    └── SKILL.md
```

Each `SKILL.md` documents: **Description**, **Inputs & Deliverables**, **Execution Workflow**.

## Start-Up Behavior
Before non-trivial work, read `.github/analysis/lessons.md` and this file. Treat the workspace as immutable until changes are approved.

---

# Part 2: Project Knowledge

NANDO is an open-source flash programmer: an STM32F103 (HD, LQFP100) board with a
TSOP-48 ZIF socket, driven by a Qt5 desktop application over USB CDC.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/programmer/` | Main application firmware (the programmer itself) |
| `firmware/bootloader/` | Bootloader |
| `firmware/libs/spl/` | ST Standard Peripheral Library + `startup_stm32f10x_hd.s` |
| `firmware/usb_cdc/` | USB CDC stack |
| `qt/` | Qt5 host application and the chip databases (`nando_*_chip_db.csv`) |
| `kicad/` | Schematic (`.sch`, KiCad 5) and PCB; `nand_programmator.net` is the netlist |
| `test/` | Host-native driver tests (no hardware needed) |
| `sim/` | Renode full-system simulation (Docker) |

## Firmware architecture

`nand_programmer.c` is the protocol engine. It receives command packets over USB
CDC and dispatches through a HAL vtable:

```c
static flash_hal_t *hal[] = { &hal_fsmc, &hal_spi, &hal_parallel_nor };
```

The index comes from the host (`conf_cmd->hal`) and **must** match
`ChipInfo::CHIP_HAL_*` in `qt/chip_info.h`:

| Index | HAL | Driver | Chip database |
|---|---|---|---|
| 0 | `CHIP_HAL_PARALLEL` | `fsmc_nand.c` | `nando_parallel_chip_db.csv` |
| 1 | `CHIP_HAL_SPI` | `spi_flash.c` | `nando_spi_chip_db.csv` |
| 2 | `CHIP_HAL_PARALLEL_SERIAL` | `parallel_nor_flash.c` | `nando_parallel_serial_chip_db.csv` |

`flash_hal_t` (in `flash_hal.h`) is the contract every driver implements.

### HAL contract rules (violating these causes real bugs)

- **`read_status` must be non-blocking.** It is polled from the main loop while a
  write is in flight; spinning inside it stalls the USB pump. Every driver keeps
  a *separate* blocking helper for the synchronous erase path
  (`nand_get_status`, `pnor_get_status`, `spi_flash_read_status`).
- **Return `FLASH_STATUS_*` from `flash_hal.h`**, never driver-local constants.
  `READY=0, BUSY=1, ERROR=2, TIMEOUT=3` — a local `TIMEOUT=2` silently becomes
  `ERROR` and gets reported to the host as a bad block.
- **`0xFF` means "command not supported."** Guard every configurable opcode.
- **`read_spare_data` returning `FLASH_STATUS_INVALID_CMD` is normal** for 2 KB-page
  NAND. `np_read_bad_block_info_from_page()` falls back to a full
  `page_size + spare_size` read and indexes `buf[page_size + bb_mark_off]`.
- `chip_id_t` has **five** ID bytes. Populate all of them.

### Host ↔ firmware wire structs

Each driver has a `__attribute__((packed))` config struct whose field order must
match its Qt counterpart **exactly**:

| Firmware | Qt |
|---|---|
| `fsmc_conf_t` in `fsmc_nand.c` | `parallel_chip_info.cpp` |
| `spi_conf_t` in `spi_flash.c` | `spi_chip_info.cpp` |
| `parallel_nor_conf_t` in `parallel_nor_flash.c` | `parallel_serial_chip_info.cpp` |

Prefer all-`uint8_t` structs. A wider member in the middle of a packed struct
forces alignment differences across the MSVC/MinGW ↔ ARM-GCC boundary.

## Hardware facts (from `kicad/nand_programmator.net`)

The ZIF socket exposes only `FSMC_D0..D15`, `FSMC_CLE`, `FSMC_ALE`, `FSMC_NCE2`,
`FSMC_NOE`, `FSMC_NWE`, `FSMC_NWAIT`. **There is no address bus**, so every
supported parallel device must speak a CLE/ALE-multiplexed protocol.

- FSMC **NAND bank 2** at `0x70000000`; `A16` = CLE, `A17` = ALE:
  - `0x70000000` data · `0x70010000` command · `0x70020000` address
- Data pins: `PD14,PD15,PD0,PD1` (D0–D3) and `PE7–PE10` (D4–D7); 8-bit only
- Control: `PD11` CLE, `PD12` ALE, `PD4` NOE, `PD5` NWE, `PD7` NCE2, `PD6` NWAIT
- USART1 debug console on `PA9`/`PA10` at 115200 (`uart.c`, `printf` routes here)
- Application 1 links at **`0x08004000`** (after the bootloader) — see
  `stm32_flash_1.ld`

## Chip database CSV format

Plain CSV with a `#` header row. **`-` means "not defined"**
(`ChipDb::paramNotDefValue`); the firmware sees `0xFF`. Used both for absent
commands and for "stop comparing here" in ID matching (e.g. a 4-byte-ID part
sets `ID5` to `-`).

`row cycles` / `col cycles` are literal counts of address bytes clocked out —
they are read straight into a `switch` in `fsmc_nand.c`. Getting them wrong
corrupts every read, program and erase. Always take them from the datasheet's
address-cycle map, and note that within one part family the 1 Gb device often
needs fewer row cycles than its 2/4 Gb siblings.

## Flashing the programmer's own firmware

Over SWD with an ST-Link:

```bash
cd firmware && make -f Makefile.linux
st-flash read backup.bin 0x08000000 0x40000      # always back up first
st-flash --reset write obj/nando_fw.bin 0x08000000
```

**Then physically unplug and replug the board's USB cable.** This is not
optional and not a nicety:

- `usb_init()` in `firmware/programmer/usb.c` busy-waits on
  `USB_IsDeviceConfigured()` — i.e. `bDeviceState == CONFIGURED` — *before*
  `cdc_init()` and before the `while (1) np_handler()` loop is ever reached.
- The board's D+ pull-up is the fixed 1.5k R5, not a software-controlled one,
  so the device never electrically detaches. `st-flash --reset` does an AIRCR
  software reset (NRST is not wired), the host therefore never re-enumerates
  and never sends SET_CONFIGURATION, and the firmware spins in `usb_init()`
  forever.

The failure looks alarming and is entirely benign: USB still shows the device
(the descriptors were served before the reset), `/dev/ttyACM*` still exists, and
every command times out. It is indistinguishable from a bricked flash unless you
know to power-cycle. Verified by halting the core over SWD: the PC sits at
`usb_init+0x12/0x16`, the `beq.n` back-edge of that wait loop.

Two further notes from doing this on hardware:

- Replug *slowly*. A disconnect ~3 s after a connect left the device enumerated
  but unresponsive; a clean power cycle fixed it with no reflash.
- `/dev/ttyACM*` numbering is not stable across replugs — the ST-Link's own VCP
  and the programmer swap places. Match on VID/PID `0483:5740`, never on the
  node name.

To confirm which firmware is actually running, use behaviour rather than the
version string, which has stayed `3.5.0` across these changes: on a chip with
more than 20 bad blocks, `read_bad_blocks` fails in about a second with
`NP_ERR_BBT_OVERFLOW` on stock firmware and completes with the bitmap.

## Build commands

```bash
# Firmware. Build from firmware/, which defines CFLAGS and the toolchain and
# passes both down; it falls back to a system arm-none-eabi- when
# ../../compiler is absent. syscalls.c now provides _exit/_kill/_getpid, so
# --specs=nosys.specs is no longer required.
cd firmware && make -f Makefile.linux
```

`CFLAGS` in `firmware/Makefile.linux` carries `-Wno-error=format`: newer newlib
ships an `inttypes.h` built without the C99 format macros (`PRIx64`), which
`-Werror` would otherwise reject. Every other warning stays fatal.

```bash
# Qt host app. CMake is the macOS path; qt.pro is used by the Linux/Windows
# release workflow. BOTH must list new sources — CMake globs, qmake does not.
cd qt && cmake -B build && cmake --build build
```

On macOS `qt.pro` has no Boost include path and `-Werror` trips over Boost's
deprecated `sprintf`; use the CMake build there.

## Testing

```bash
cd test && make          # host-native, no hardware
```

Drivers are compiled natively against behavioural chip models. Every bus access
goes through `*_bus_read`/`*_bus_write` primitives — inline volatile
dereferences on target, redirected to the model under `-DPNOR_HOST_TEST` /
`-DNAND_HOST_TEST`. `spl_stub.h/.c` stands in for `<stm32f10x.h>` and *captures*
what the driver configures, so FSMC settings are assertable.

The models enforce the protocol (address-cycle counts, write-enable ordering,
NOR/NAND program-only-clears-bits) and report violations rather than returning
plausible data. `test_fsmc_nand.c` reads its config from the real
`nando_parallel_chip_db.csv`, so it tests the shipped database row.

```bash
cd sim && docker-compose up    # Renode, writes to sim/out/ (readable from the repo)
```

Renode notes learned the hard way:
- The stock `platforms/cpus/stm32f103.repl` maps **FSMC bank 1 only**; bank 2 at
  `0x70000000` must be added (`sim/nando.repl`).
- There is **no RCC peripheral**. `RCC_CR` has a platform tag but `RCC_CFGR` does
  not, so `SystemInit()` spins waiting for `SWS`. Tag it with `0x0000000A`.
- `cpu VectorTableOffset` must be `0x08004000`, not `0x08000000` — the first LOAD
  segment starts at file offset 0, so `0x08000000` holds the ELF header.
- `Python.PythonPeripheral` request members are **PascalCase**: `IsInit`,
  `IsRead`, `IsWrite`, `Offset`, `Value`, `Length`. `Init()` runs lazily from
  `EnsureInit()` during the first access.
- Honour `request.Length`: the 8-bit FSMC turns a 32-bit CPU read into four
  consecutive byte fetches (`nand_read_id` depends on this).

## Known issues, not yet addressed

- `fsmc_nand.c` calls `nand_fsmc_init(fsmc_conf)` but the function is declared
  with empty parens and takes no argument. Harmless today (it reads the global),
  GCC is silent, clang warns, invalid under C2x.
- `nand_uninit()` is still a `TODO`, so switching away from NAND leaves FSMC
  bank 2 configured.
- ~~Several 1 Gb entries specify 3 row cycles while 16-bit row addressing
  suggests 2.~~ Audited: every row in `nando_parallel_chip_db.csv` now agrees
  with `ceil(ceil(log2(total_size / page_size)) / 8)`. All thirteen 1 Gb
  2 KB-page parts are on 2 row cycles and all ten 2 Gb parts on 3. Note the
  table corroborated itself before the change — seven of the 1 Gb entries
  already said 2, including `W29N01HVSINA` while its sibling `W29N01GV` said 3.
- `H27UBG8T2A` is the one deliberate exception: it keeps 4 row cycles where the
  formula wants 3. Its `total size` is `4294967295`, a clamped `2^32 - 1` rather
  than the real 4 GiB, so the derived page count is wrong by construction and
  the geometry cannot settle the question. It is a stacked-die part; confirm
  against the datasheet before touching either field.
- The parallel-serial (`CHIP_HAL_PARALLEL_SERIAL`) path passes host tests but has
  **never been run on real hardware**, and its chip database is empty.
- **The four ST/Numonyx `NAND*W3*` rows all declare the wrong `total size`,**
  and their own device-ID bytes prove it. `NAND01GW3B` (`20 F1`, where `F1` is
  the JEDEC code for 1 Gbit), `NAND02GW3B` (`20 DA`, 2 Gbit) and `NAND128W3A`
  (`20 73`, 128 Mbit) every one declare `1073741824` — 8 Gbit. It looks like one
  value pasted across the group. A too-large `total size` lets a full read or
  write address past the end of the part, so this is the dangerous direction.
  `NAND128W3A` is worse still: at 16 KB blocks its declared size implies 65536
  blocks, over `NAND_BBT_MAX_BLOCKS`, so `nand_bad_block_table_init()` refuses it
  and the chip cannot be configured at all. Correcting `total size` alone is not
  enough — the row cycles then change too (`NAND01GW3B` would want 2, not 3), and
  `NAND128W3A` has 512-byte pages but claims 2 column cycles where the other
  small-page rows correctly use 1. These want a datasheet each, not arithmetic.
- `NAND512W3A2C` carries the same five ID bytes as `S34ML04G1`
  (`01 DC 90 95 54`) and sits after it in the file, so `chipInfoGetByChipId()`
  — which returns the first match — can never select it. Its geometry is also
  the S34ML04G1 row's (2 KB page, 512 MiB) under a part number that is a
  512 Mbit device with 512-byte pages, and whose maker code should be `20`
  (ST), not `01`. Selecting it by hand would write 2 KB pages to a 512-byte-page
  part. Left alone pending a datasheet: the row needs correcting or deleting,
  not guessing.

## Socket capability limit: single chip enable

`kicad/nand_programmator.net` wires **pin 9 (`FSMC_NCE2`) as the only chip
enable**, with pin 7 as `FSMC_NWAIT` (R/B#). **Socket pins 1-6 are
unconnected**, and so is pin 19 (WP#).

This matters only for parts that expose more than one chip enable. Samsung
encodes that in position 9 of the part number, the "Mode" field of its official
NAND part number decoder: `0` = Normal (single nCE, single R/nB), `1` = Dual nCE
& Dual R/nB, `3` = Tri, `4`/`5` = Quad. **A `U1`/`U3`/`U4`/`U5` part can only
reach its first chip enable on this board; a `U0` part is fully reachable.**

Die stacking is a separate field — position 3 — and does *not* imply multiple
chip enables. `F`/`G` are single-die, `K` is an SLC die stack, `L` an MLC DDP,
`W` an SLC 4-die stack. So `K9K8G08U0D` is two dies behind **one** CE and is
fully addressable here at its whole 8 Gbit; `K9K8G08U1D` is the same density with
two CEs and would present only half. `K9WAG08U1D` decodes as 16 Gbit / 4-die
stack / dual CE, i.e. 8 Gbit per CE, which is consistent with how such parts are
built.

`K9K8G08U0D` is therefore in the database at its full **1073741824 bytes**. Note
that lands on exactly 8192 blocks, which is precisely `NAND_BBT_MAX_BLOCKS`: it
fits, with nothing spare. A larger part, or this one with smaller blocks, would
be refused by `nand_bad_block_table_init()`.

Its ID is `EC D3 51 95`, verified against the SUNXI NFC MTD driver's chip table,
which carries `{0xec, 0xd3, 0x51, 0x95}` for K9K8G08 with `id_len: 4` — a
production driver matching this part on four bytes, which is what the row's
`ID5 = -` does here too. The 3rd byte's die-count field (bits 1:0) was
cross-checked against the family naming over seven entries in that table:
`K9F8G08` `0x50` and `K9G8G08` `0x14` decode to one die, `K9K8G08` `0x51` and
`K9L8G08` `0x55` to two, matching F/G single-die and K/L two-die. It also
confirms the existing `K9G8G08U0A` (`EC D3 14 A5`) and `K9G8G08U0M`
(`EC D3 14 25`) rows byte for byte.

**Confirmed on hardware.** A K9K8G08U0D in the socket reports
`ec d3 51 95 58` — the predicted four bytes, plus a 5th of `0x58` (the
package-level plane encoding: 4 planes x 2 Gbit = 8 Gbit). The row keeps
`ID5 = -` deliberately, matching on four bytes as the SUNXI driver does, so it
also covers sibling generations whose 5th byte differs.

The full 1 GiB is genuinely reachable: comparing page *k* against page
*k + 262144* across five populated offsets gave distinct content every time,
where a part limited to its first die would have mirrored them. The upper-half
pages read exactly `+0x10` from their lower-half counterparts in all five cases —
deterministic, address-derived, not wrapped. Sequential pages advance the same
pattern coherently, which also exercises `row cycles = 3`.

Caution about *that* particular chip: it holds a full-chip test pattern covering
the spare area as well as the main area, so its bad block markers are overwritten
— 23 of the first 24 blocks read a non-`0xFF` marker byte. `read_bad_blocks` on
it fails with `NP_ERR_BBT_OVERFLOW` almost immediately on stock 3.5.0 firmware,
whose table holds 20 entries. The bitmap makes the table big enough to hold the
answer but cannot make the answer meaningful: this part's factory bad block list
is gone. See also the ECC design doc's warning that writing a foreign dump can
mark good blocks bad.
