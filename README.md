# M5Cardputer Loader

A resident **firmware launcher** for the [M5Cardputer](https://docs.m5stack.com/en/core/Cardputer)
(ESP32‑S3, 8 MB). It lives in on‑board flash, keeps a library of firmwares as
`.bin` files on the microSD card, and boots into any of them from a simple list
UI — then **a tap of RESET returns you to the loader, for any firmware** (even
ones that self‑validate their OTA image, like Bruce).

Inspired by [bmorcelli/Launcher](https://github.com/bmorcelli/Launcher) and
[tobozo/M5Stack-SD-Updater](https://github.com/tobozo/M5Stack-SD-Updater).

---

## What it does

- **Main screen** lists installed firmwares (bright orange) at the top, then the
  functions (`-  OTA Install  -`, `-  WebUI Upload  -`, `-  Power  -`,
  `-  Settings  -`, `-  About  -`, `-  Reboot  -`). Firmware names are cleaned up
  for display (`Bruce.1.2.3.3.tar.bin` → `Bruce 1.2.3.3`).
- **Launch / Rename / Delete** any firmware. Launch copies it into `ota_0` and
  reboots into it.
- **RESET returns to the loader** — see *Return mechanism* below.
- **OTA Install** — pulls the live **M5Burner catalog** (Cardputer category,
  ~550 firmwares) and/or installs from a **GitHub release**, downloads over
  WiFi, and saves a launchable `.bin` to the SD card (with a progress bar).
- **WebUI Upload** — hosts a web page; upload/delete `.bin`s from a browser over WiFi.
- **Per‑firmware persistence** — each firmware keeps its own internal state
  (NVS + SPIFFS) across runs; files apps write to the SD card persist naturally.
- **Settings** — WiFi (SSID/pass in an isolated NVS), WiFi test, brightness, SD rescan.
- **Power** — live battery voltage/%/charge‑state, a header battery glyph (fill =
  level, colour = green > 50 % / orange 30–50 % / red < 30 %, ⚡ when charging),
  two‑stage idle blanking (**dim**, then **screen off**, both timeouts configurable),
  power off / deep sleep. The battery readout updates live (~1 s) while a screen is
  shown. See *Power & charging* below.

### Controls
`;` up · `.` down · `ENTER` select · `` ` `` or `Backspace` back.

---

## Hardware (research summary)
ESP32‑S3FN8, dual‑core LX7 @240 MHz, **8 MB flash, no PSRAM**; ST7789V2 240×135;
56‑key keyboard (Cardputer **ADV** = TCA8418 I2C @0x34 on G8/G9; original = GPIO
matrix); microSD on SPI **CS=G12 MOSI=G14 CLK=G40 MISO=G39**; battery via ADC on
**G10** (×2.0, calibrated); G0 = the user button (also the boot strapping pin —
*don't hold it at power‑on*).

---

## Power & charging

The Cardputer/ADV has **no PMIC**. The charger is a **TP4057** — a dumb standalone
Li‑Po charger with **no I2C, no enable pin, and no status output**. So:

- **Firmware cannot enable, set, or read charging.** `M5Unified` reports the board
  as `pmic_adc`; `isCharging()` returns *unknown* and `setChargeCurrent()` etc. are
  no‑ops. The only battery signal is the **ADC voltage** (G10).
- **Charge current is hardware‑fixed at ~62 mA** and the **switch must be ON** to
  charge (off physically disconnects the cell).
- Powered on, the device draws **~300 mA**, so on a weak supply (hub / PC port /
  thin cable) the system load eats everything and the battery **nets zero** — looks
  like "not charging." Measured: a **laptop USB port does not net‑charge it even in
  low‑power mode**; a **5 V ≥1 A wall charger does** (~+120 mV/hr while idle).

What the loader does to help: idles the CPU at **80 MHz** (boosts to 240 MHz only
for WiFi/downloads), and at the screen‑off idle stage drops to **40 MHz** with the
backlight off — minimising the load that competes with the 62 mA charge.

**To charge:** use a real wall charger, switch ON, and **leave it idle** so the
screen blanks. Watching it (screen on) keeps it from gaining. The charge indicator
infers state from a 90 s voltage trend (the only method possible on this hardware),
so ⚡ appears within a minute or two of a genuine rise.

---

## Partition table ([`partitions.csv`](partitions.csv))

```
nvs_ldr   0x009000  16 KB   loader's OWN config (isolated from apps)
bootflag  0x00d000   4 KB   one-shot boot flag (read by the bootloader hook)
otadata   0x00e000   8 KB   boot selection
factory   0x010000 1.4 MB   the LOADER (this project)
ota_0     0x170000 4.5 MB   runtime slot (selected firmware runs here)
spiffs    0x5f0000 1.5 MB   app FS     (snapshotted per-firmware)
nvs       0x770000  24 KB   app NVS    (snapshotted per-firmware)
ota_1     0x780000 448 KB   STUB — must exist or ESP32Marauder smashes its stack
coredump  0x7f0000  64 KB
```

`ota_1` is never launched into; it exists only because ESP32Marauder's boot‑time
OTA‑partition lookup needs a second app slot (see *Known limitation*). Launches
still target `ota_0` (`esp_ota_get_next_update_partition()` from `factory` returns it).

---

## Return mechanism (the important part)

A launched app becomes the boot target, and apps like Bruce mark their own image
"valid" — so plain OTA rollback can't bring you back. The fix is a **custom
bootloader hook** (`bootloader_components/loader_boot/loader_boot.c`):

- On every boot it reads the `bootflag` sector:
  - **magic set** → clear it, keep otadata → boot `ota_0` (the chosen firmware) **once**.
  - **not set** → wipe otadata → **force‑boot the loader** (`factory`).
- The loader's `armBootOnce()` writes the magic right before launching a firmware.

Because the bootloader runs *before* the app and rewrites the boot selection
itself, **RESET always returns to the loader regardless of what the app did.**

> pioarduino's `framework = arduino` always flashes a *prebuilt* bootloader and
> never compiles one, so the hook can't go through the normal build. It's built
> standalone (see below) and flashed at `0x0`.

> **Not a security boundary.** `bootflag`, `otadata` and `factory` live in plain
> writable flash (no Secure Boot / flash encryption), and the hook doesn't check
> *who* set the flag. A launched firmware runs fully privileged and can forge the
> boot flag to keep booting itself, or erase `factory` to disable the loader — so
> the guarantee above holds for **well-behaved** apps; it's a convenience
> mechanism, not containment against a hostile image.

---

## Building & flashing

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`).

```bash
# 1) App (fast, ~11s)
pio run -e cardputer

# 2) Hooked bootloader (only needed once, or when the hook/partitions change)
./build-bootloader.sh            # -> bl_build/bootloader.bin (verifies the hook is linked)

# 3) Flash
./flash-all.sh full              # bootloader + partitions + (wifi creds) + app, first time
./flash-all.sh app               # JUST the app at 0x10000, for day-to-day code changes
```

`build-bootloader.sh` builds the ESP‑IDF bootloader sub‑project directly (it
honors the project's `bootloader_components/`), using the toolchain + IDF venv
that pioarduino already installed. `flash-all.sh full` also clears
`bootflag`/`otadata` so the device boots cleanly into the loader.

Optional WiFi creds: generate a 16 KB NVS image (namespace `loader`, keys
`ssid`/`pass`) and place it at `/tmp/wifi_nvs_4000.bin`; `flash-all.sh full`
writes it to `nvs_ldr` (0x9000). Or just set WiFi on‑device in Settings.

---

## SD card

Format **FAT32, ≤ 32 GB, MBR**. Firmwares live in `/firmware/*.bin`. The loader
manages it: `/firmware/_m5cat.tsv` (cached M5Burner index) and
`/firmware/data/<fw>/` (per‑firmware NVS/SPIFFS snapshots) are created automatically.

---

## Known limitation

Firmwares that need their **own large data partition** at a fixed offset (e.g.
UIFlow, or builds whose bundled SPIFFS is larger than our 1.5 MB `spiffs`) may not
run when launched from `ota_0`, because the live partition table is the loader's.
Self‑contained apps (Bruce, NEMO, Evil‑Cardputer, ESP32Marauder, most tools) work
fine. A future "partition‑manager" mode could rewrite the table per‑firmware.

> **ESP32Marauder note:** Marauder's boot‑time OTA‑partition lookup hard‑requires
> a *second* app slot to exist, or it smashes its own stack on boot (it crashes
> identically even flashed standalone with a single‑OTA table). The partition
> table therefore keeps a small stub `ota_1` (448 KB) purely so that lookup
> succeeds — we never launch into it, and `esp_ota_get_next_update_partition()`
> from the loader still resolves to `ota_0`, so launches are unaffected. Install
> the **`m5cardputer_adv`** asset (the I2C‑keyboard build); the plain
> `m5cardputer` build targets the original GPIO‑matrix keyboard and its keys
> won't respond on the ADV.

---

## Project layout

```
src/main.cpp                              the loader (UI, SD, OTA, WebUI, Power, persistence)
partitions.csv                            8MB table (nvs_ldr/bootflag/factory/ota_0/nvs/spiffs)
bootloader_components/loader_boot/        the boot-once bootloader hook
bl_sdkconfig.defaults                     sdkconfig defaults for the standalone bootloader build
build-bootloader.sh                       builds bl_build/bootloader.bin (with the hook)
flash-all.sh                              flash helper (app | full)
platformio.ini                            app build config
```

### Sources
- [M5Cardputer hardware docs](https://docs.m5stack.com/en/core/Cardputer)
- [bmorcelli/Launcher](https://github.com/bmorcelli/Launcher)
- [tobozo/M5Stack-SD-Updater](https://github.com/tobozo/M5Stack-SD-Updater)

---

## License

MIT — see [LICENSE](LICENSE). © 2026 p0rtm0lester.

## Acknowledgements

Inspired by [bmorcelli/Launcher](https://github.com/bmorcelli/Launcher) (MIT,
© 2023 shikarunochi) and [tobozo/M5Stack-SD-Updater](https://github.com/tobozo/M5Stack-SD-Updater).
Third-party license notices are reproduced in
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). Build-time libraries
(M5Unified, M5GFX, M5Cardputer, arduino-esp32, ESP-IDF) are pulled by PlatformIO
and keep their own licenses.
