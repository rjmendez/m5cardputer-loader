// SPDX-License-Identifier: MIT
// Copyright (c) 2026 p0rtm0lester
// =====================================================================
//  M5Cardputer Loader
//  ---------------------------------------------------------------------
//  A resident firmware launcher for the M5Cardputer (ESP32-S3, 8MB).
//
//  Features
//    * Simple scrollable list UI driven by the Cardputer keyboard.
//    * Launch firmwares stored as .bin files on the microSD card
//      (/firmware/*.bin). The chosen file is streamed into the ota_0
//      partition and the device reboots into it.
//    * OTA installer: download a .bin from a URL (read from
//      /firmware/ota.txt, or typed in) straight into ota_0 over WiFi.
//    * Settings: WiFi credentials (saved to NVS), screen brightness,
//      SD rescan, and info.
//
//  Controls (Cardputer keyboard)
//    ;  = up        .  = down        ENTER = select
//    `  = back/exit (also BACKSPACE)
//
//  See README.md for the partition scheme and the return-to-loader
//  mechanism.
// =====================================================================

#include <M5Cardputer.h>
#include "esp32-hal-cpu.h"   // setCpuFrequencyMhz (lower idle draw so the charger wins)
#include <SD.h>
#include <SPI.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <vector>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "fwutil.h"      // pure helpers: lipoPct, prettyName, extractVersion, safeName, fwIdFromPath

// ---- M5Cardputer microSD pins (from the hardware spec) --------------
static const int SD_SCK  = 40;   // G40 CLK
static const int SD_MISO = 39;   // G39 MISO
static const int SD_MOSI = 14;   // G14 MOSI
static const int SD_CS   = 12;   // G12 CS

// ---- Display geometry ------------------------------------------------
static const int SCR_W = 240;
static const int SCR_H = 135;
static const int HDR_H = 18;
static const int ROW_H = 16;
static const int FTR_H = 14;
static const int LIST_Y = HDR_H + 2;
static const int VISIBLE_ROWS = (SCR_H - HDR_H - FTR_H - 2) / ROW_H;  // ~6 rows at font size 1

// ---- Colors ----------------------------------------------------------
#define COL_BG     TFT_BLACK
#define COL_HDR    TFT_BLACK   // header/footer bar bg (black; set off by a green rule)
#define COL_HDRTXT 0x07E0      // bright terminal green (header/footer text)
#define COL_RULE   0x05C0      // green divider rule under header / over footer
#define COL_TXT    TFT_WHITE
#define COL_DIM    TFT_DARKGREY
#define COL_SEL    0xFD20      // amber (selection highlight bar) — pairs with green header
#define COL_SELTXT TFT_BLACK
#define COL_FW     0xFD60      // bright orange (firmware list text)
#define COL_OK     TFT_GREEN
#define COL_ERR    TFT_RED
#define COL_WARN   0xFD20      // orange (battery 30-50%)

// Battery % color: >50 green, 30-50 orange, <30 red.
static inline uint16_t battColor(int lvl) {
    return lvl > 50 ? COL_OK : (lvl >= 30 ? COL_WARN : COL_ERR);
}

// ---- Battery / charging tracking -----------------------------------------
// The Cardputer (ADV) has NO PMIC: battery is read as a calibrated ADC voltage
// and charging is done by a fixed hardware charger with no status line. So we
// can't ask "am I charging?" — we infer it from the voltage trend (rising =
// charging, pinned high = charged) and map voltage -> % through a Li-Po curve
// (more accurate + far less jumpy than the library's single-sample linear %).
static int      g_battMv   = 0;     // smoothed battery mV (displayed value)
static int      g_battBase = 0;     // trend baseline mV (rolled each window)
static int      g_battPct  = -1;    // state-of-charge %
static int      g_chgState = 0;     // 0 = discharging, 1 = charging, 2 = charged/full
static uint32_t g_battNext = 0;     // next sample time (ms)
static uint32_t g_baseNext = 0;     // next trend-evaluation time (ms)

// lipoPct() moved to fwutil.h

// Sample (averaged) battery voltage, smooth it, and infer charge state from trend.
static void updatePower(bool force = false) {
    uint32_t now = millis();
    if (!force && (int32_t)(now - g_battNext) < 0) return;
    g_battNext = now + 2000;                          // sample every 2s
    int acc = 0, n = 0;
    for (int i = 0; i < 16; i++) { int v = M5Cardputer.Power.getBatteryVoltage(); if (v > 0) { acc += v; n++; } }
    if (!n) return;
    int mv = acc / n;
    if (g_battMv == 0) { g_battMv = g_battBase = mv; g_baseNext = now + 90000; }  // prime
    g_battMv  = (g_battMv * 3 + mv) / 4;              // light smoothing for the displayed value
    g_battPct = lipoPct(g_battMv);
    // Charge trend over a fixed ~90s window (truncation-free): the charger is so
    // slow (~2-3 mV/min) that we compare the smoothed voltage to a baseline rolled
    // every 90s rather than an EMA delta (which lost sub-1mV steps to int rounding).
    if ((int32_t)(now - g_baseNext) >= 0) {
        int delta = g_battMv - g_battBase;
        if (g_battMv >= 4180 && delta >= -2) g_chgState = 2;   // pinned high -> charged
        else if (delta >  2) g_chgState = 1;                   // rose -> charging
        else if (delta < -2) g_chgState = 0;                   // fell -> discharging
        // |delta|<=2: flat (e.g. laptop holding it steady) -> keep last state
        g_battBase = g_battMv;
        g_baseNext = now + 90000;
    }
}

#define FW_DIR   "/firmware"
#define CAT_TSV  "/firmware/_m5cat.tsv"     // cached slim M5Burner catalog (name<TAB>version<TAB>file)
#define M5B_LIST "https://m5burner-api.m5stack.com/api/firmware"
#define M5B_CDN  "https://m5burner-cdn.m5stack.com/firmware/"
#define M5B_CAT  "cardputer"                 // M5Burner category to filter on

static Preferences prefs;
static auto& dsp = M5Cardputer.Display;
static M5Canvas cv(&dsp);          // off-screen frame buffer (double buffering -> no flicker)
static bool sdOK = false;

// Font sizes: back to the original small-but-readable size 1 everywhere.
static const int FONT_BIG = 1;
static const int FONT_SMALL = 1;


// Power / auto-dim state
static uint32_t lastActivity = 0;
static uint8_t  dimState = 0;       // 0 = normal, 1 = dimmed, 2 = screen off (launcher idle)
static uint32_t g_dimMs = 120000;   // idle -> dim         (0 = never)
static uint32_t g_offMs = 300000;   // idle -> screen off  (0 = never)
static uint8_t  g_bright = 128;

// =====================================================================
//  Navigation input
// =====================================================================
enum Nav { N_NONE, N_UP, N_DOWN, N_ENTER, N_BACK };

static Nav pollNav() {
    M5Cardputer.update();
    updatePower();                          // throttled battery/charge sampling
    bool changed = M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed();
    if (changed) {
        lastActivity = millis();
        if (dimState) { setCpuFrequencyMhz(80); dsp.setBrightness(g_bright); dimState = 0; return N_NONE; }  // wake screen + clock
    } else {
        uint32_t idle = millis() - lastActivity;        // two-stage idle blanking (launcher only)
        if (g_offMs && idle > g_offMs) {                // screen off + deep-idle 40MHz (lowest draw -> charges fastest)
            if (dimState != 2) { dsp.setBrightness(0); setCpuFrequencyMhz(40); dimState = 2; }
        } else if (g_dimMs && idle > g_dimMs) {         // dim
            if (dimState != 1) { dsp.setBrightness(8); dimState = 1; }
        }
        return N_NONE;
    }
    auto st = M5Cardputer.Keyboard.keysState();
    if (st.enter) return N_ENTER;
    if (st.del)   return N_BACK;            // backspace = back
    for (char c : st.word) {
        switch (c) {
            case ';': return N_UP;          // up arrow key
            case '.': return N_DOWN;        // down arrow key
            case '`': return N_BACK;        // esc / back
        }
    }
    return N_NONE;
}

// =====================================================================
//  Low-level drawing helpers
// =====================================================================
// All helpers compose into the off-screen canvas `cv`; callers blit once
// with cv.pushSprite(0,0). This double-buffering removes the flicker that
// came from clearing the live display every frame.
static void drawHeader(const char* title) {
    cv.fillRect(0, 0, SCR_W, HDR_H, COL_HDR);
    cv.setTextColor(COL_HDRTXT, COL_HDR);
    cv.setTextDatum(middle_left);
    cv.setTextSize(FONT_BIG);
    cv.drawString(title, 4, HDR_H / 2);
    // battery indicator: glyph (fill = %, color = level) + bolt when charging.
    updatePower();                                   // throttled; keeps the reading fresh
    int pct = g_battPct;
    if (pct >= 0) {
        uint16_t col = (g_chgState ? COL_OK : battColor(pct));   // green while on charger
        const int bw = 18, bh = 10, nub = 2;
        int bx = SCR_W - 3 - nub - bw;               // battery body, right-aligned
        int by = (HDR_H - bh) / 2;
        cv.drawRect(bx, by, bw, bh, col);
        cv.fillRect(bx + bw, by + 3, nub, bh - 6, col);          // + terminal nub
        int fw = (bw - 4) * pct / 100; if (fw > 0) cv.fillRect(bx + 2, by + 2, fw, bh - 4, col);
        if (g_chgState == 1) {                       // charging -> lightning bolt
            int cx = bx + bw / 2, cy = by + bh / 2;
            cv.fillTriangle(cx + 2, by + 1, cx - 3, cy + 1, cx + 1, cy, TFT_YELLOW);
            cv.fillTriangle(cx - 1, cy, cx + 3, cy - 1, cx - 2, by + bh - 1, TFT_YELLOW);
        }
        cv.setTextDatum(middle_right);               // % to the left of the icon
        cv.setTextSize(FONT_SMALL);
        cv.setTextColor(col, COL_HDR);
        cv.drawString(String(pct) + "%", bx - 3, HDR_H / 2);
    }
    cv.drawFastHLine(0, HDR_H - 1, SCR_W, COL_RULE); // green rule separates header from body
}

static void drawFooter(const char* hint) {
    cv.fillRect(0, SCR_H - FTR_H, SCR_W, FTR_H, COL_HDR);
    cv.drawFastHLine(0, SCR_H - FTR_H, SCR_W, COL_RULE);  // green rule over footer
    cv.setTextColor(COL_HDRTXT, COL_HDR);
    cv.setTextDatum(middle_left);
    cv.setTextSize(FONT_SMALL);
    cv.drawString(hint, 4, SCR_H - FTR_H / 2 - 1);
}

// Centered status message screen; optional pause.
static void screenMsg(const char* line1, const char* line2 = nullptr,
                      uint16_t color = COL_TXT, uint32_t holdMs = 0) {
    cv.fillScreen(COL_BG);
    cv.setTextColor(color, COL_BG);
    cv.setTextDatum(middle_center);
    cv.setTextSize(FONT_BIG);
    cv.drawString(line1, SCR_W / 2, SCR_H / 2 - (line2 ? 12 : 0));
    if (line2) { cv.setTextSize(FONT_SMALL); cv.drawString(line2, SCR_W / 2, SCR_H / 2 + 14); }
    cv.pushSprite(0, 0);
    if (holdMs) delay(holdMs);
}

static void drawProgressBar(const char* label, size_t cur, size_t total) {
    static int lastPct = -1;
    int pct = total ? (int)((cur * 100) / total) : 0;
    if (pct == lastPct) return;
    lastPct = pct;
    cv.fillScreen(COL_BG);
    cv.setTextColor(COL_TXT, COL_BG);
    cv.setTextDatum(middle_center);
    cv.setTextSize(FONT_BIG);
    cv.drawString(label, SCR_W / 2, 36);
    int bx = 20, by = 66, bw = SCR_W - 40, bh = 20;
    cv.drawRect(bx, by, bw, bh, COL_TXT);
    cv.fillRect(bx + 2, by + 2, (bw - 4) * pct / 100, bh - 4, COL_SEL);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    cv.drawString(buf, SCR_W / 2, by + bh + 14);
    cv.pushSprite(0, 0);
}

// =====================================================================
//  Generic scrollable list menu.
//  Returns selected index, or -1 if the user pressed BACK.
// =====================================================================
// fwColor is used for the first `fwCount` (non-selected) rows — e.g. bright
// orange for the firmware entries on the main screen. Remaining rows use COL_TXT.
// `provider`, if given, is called on each (re)draw to rebuild the rows — used for
// live menus (e.g. Power shows updating battery/voltage). Static menus pass none.
static int listMenu(const char* title, const std::vector<String>& items,
                    const char* footer, int start = 0,
                    uint16_t fwColor = COL_TXT, int fwCount = 0,
                    std::vector<String> (*provider)() = nullptr) {
    int sel = start;
    int top = 0;
    bool redraw = true;
    std::vector<String> dyn;
    if (provider) dyn = provider();
    const std::vector<String>& L = provider ? dyn : items;   // live-refreshable source
    if (L.empty()) {
        screenMsg(title, "(empty) - press ` to go back");
        while (pollNav() != N_BACK) delay(10);
        return -1;
    }
    uint32_t nextTick = millis() + 1000;     // periodic redraw so live data (battery) updates
    for (;;) {
        if (sel < top) top = sel;
        if (sel >= top + VISIBLE_ROWS) top = sel - VISIBLE_ROWS + 1;

        if (redraw) {
            cv.fillScreen(COL_BG);
            drawHeader(title);
            cv.setTextSize(FONT_BIG);
            for (int i = 0; i < VISIBLE_ROWS && (top + i) < (int)L.size(); i++) {
                int idx = top + i;
                int y = LIST_Y + i * ROW_H;
                bool on = (idx == sel);
                if (on) cv.fillRect(0, y, SCR_W, ROW_H, COL_SEL);
                uint16_t fg = (idx < fwCount) ? fwColor : COL_TXT;
                cv.setTextColor(on ? COL_SELTXT : fg, on ? COL_SEL : COL_BG);
                cv.setTextDatum(middle_left);
                String row = L[idx];
                if (row.length() > 38) row = row.substring(0, 37) + "~";   // ~40 chars fit at size 1
                cv.drawString(row, 6, y + ROW_H / 2);
            }
            // scroll indicator
            if ((int)L.size() > VISIBLE_ROWS) {
                int trackY = LIST_Y, trackH = VISIBLE_ROWS * ROW_H;
                int knobH = trackH * VISIBLE_ROWS / L.size();
                int knobY = trackY + trackH * top / L.size();
                cv.fillRect(SCR_W - 3, trackY, 3, trackH, COL_DIM);
                cv.fillRect(SCR_W - 3, knobY, 3, knobH, COL_SEL);
            }
            drawFooter(footer);
            cv.pushSprite(0, 0);
            redraw = false;
        }

        Nav n = pollNav();
        switch (n) {
            case N_UP:    sel = (sel - 1 + L.size()) % L.size(); redraw = true; break;
            case N_DOWN:  sel = (sel + 1) % L.size();            redraw = true; break;
            case N_ENTER: return sel;
            case N_BACK:  return -1;
            default: break;
        }
        // periodic live refresh (battery icon + provider rows) while idle; skip when screen off
        if (dimState != 2 && (int32_t)(millis() - nextTick) >= 0) {
            nextTick = millis() + 1000;
            if (provider) { dyn = provider(); if (sel >= (int)dyn.size()) sel = (int)dyn.size() - 1; }
            redraw = true;
        }
        delay(8);
    }
}

// =====================================================================
//  On-screen text entry (for WiFi credentials / URLs).
//  Returns false if cancelled (BACK on empty).
// =====================================================================
static bool textInput(const char* title, String& out, bool mask = false) {
    String buf = out;
    for (;;) {
        cv.fillScreen(COL_BG);
        drawHeader(title);
        cv.setTextColor(COL_TXT, COL_BG);
        cv.setTextDatum(top_left);
        cv.setTextSize(FONT_BIG);
        String shown = buf;
        if (mask) { shown = ""; for (size_t i = 0; i < buf.length(); i++) shown += '*'; }
        // wrap manually (char cell ~6x8 at size 1)
        int x = 6, y = LIST_Y + 4;
        for (int i = 0; i < (int)shown.length(); i++) {
            cv.drawChar(shown[i], x, y);
            x += 6;
            if (x > SCR_W - 10) { x = 6; y += 11; }
        }
        cv.fillRect(x, y, 5, 8, COL_SEL);  // cursor
        drawFooter("type  ENTER=ok  `=cancel");
        cv.pushSprite(0, 0);

        // read keys
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            auto st = M5Cardputer.Keyboard.keysState();
            if (st.enter) { out = buf; return true; }
            if (st.del && buf.length()) buf.remove(buf.length() - 1);
            for (char c : st.word) {
                if (c == '`') { return false; }       // cancel
                if (c >= 32 && c < 127) buf += c;
            }
        }
        delay(10);
    }
}

// =====================================================================
//  SD card
// =====================================================================
static SPIClass sdSPI(HSPI);

static bool mountSD() {
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdOK = SD.begin(SD_CS, sdSPI, 25000000);
    if (sdOK && !SD.exists(FW_DIR)) SD.mkdir(FW_DIR);
    return sdOK;
}

// FwAlias/FW_ALIASES, extractVersion(), prettyName() moved to fwutil.h

// Collect *.bin files under /firmware into `paths` (full path) / `names` (pretty).
static void scanFirmware(std::vector<String>& names, std::vector<String>& paths) {
    names.clear(); paths.clear();
    if (!sdOK && !mountSD()) return;
    File dir = SD.open(FW_DIR);
    if (!dir) return;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (f.isDirectory()) { f.close(); continue; }
        String name = f.name();           // basename on esp32 SD
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        if (name.startsWith(".")) { f.close(); continue; }   // skip hidden/temp (.dl.tmp etc.)
        String lower = name; lower.toLowerCase();
        if (lower.endsWith(".bin")) {
            names.push_back(prettyName(name));
            paths.push_back(String(FW_DIR) + "/" + name);
        }
        f.close();
    }
    dir.close();
}


// =====================================================================
//  Per-firmware persistence
//  ---------------------------------------------------------------------
//  Each firmware keeps its own internal-flash state (the app "nvs" + the
//  "spiffs" partitions) in /firmware/data/<fwId>/. On launch we restore
//  that firmware's snapshot into flash; on return (the loader boots after
//  a RESET/rollback) we snapshot the partitions back. Files apps write
//  straight to the SD card persist on their own (shared filesystem).
// =====================================================================
#define DATA_DIR "/firmware/data"

// fwIdFromPath() moved to fwutil.h
static String dataDirFor(const String& fwId) { return String(DATA_DIR) + "/" + fwId; }

static const esp_partition_t* dataPart(const char* label) {
    return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, label);
}

// Is the partition effectively empty (first 4KB all 0xFF)? Skip backing those up.
static bool partitionLooksEmpty(const esp_partition_t* p) {
    uint8_t buf[256];
    if (esp_partition_read(p, 0, buf, sizeof(buf)) != ESP_OK) return true;
    for (uint8_t b : buf) if (b != 0xFF) return false;
    return true;
}

static bool backupPartition(const char* label, const String& path) {
    const esp_partition_t* p = dataPart(label);
    if (!p) return false;
    if (partitionLooksEmpty(p)) { SD.remove(path); return true; }   // nothing to save
    File f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    static uint8_t buf[4096];
    for (uint32_t off = 0; off < p->size; off += sizeof(buf)) {
        uint32_t n = p->size - off; if (n > sizeof(buf)) n = sizeof(buf);
        if (esp_partition_read(p, off, buf, n) != ESP_OK) { f.close(); SD.remove(path); return false; }
        f.write(buf, n);
    }
    f.close();
    return true;
}

// Called at loader boot: snapshot the firmware that just ran back to SD.
// (Uses the live table, which is the one the launched app booted with.)
static void persistLastFirmware() {
    String last = prefs.getString("lastfw", "");
    if (last.isEmpty()) return;
    bool wasFull = prefs.getBool("lastfull", false);
    prefs.remove("lastfw");                             // clear first (avoid loops on failure)
    if (!sdOK && !mountSD()) return;
    SD.mkdir(DATA_DIR);
    String dir = dataDirFor(last);
    SD.mkdir(dir);
    backupPartition("nvs", dir + "/nvs.bin");            // app settings persist across runs
    if (!wasFull) backupPartition("spiffs", dir + "/spiffs.bin");  // full-image spiffs comes from the image
}

static uint32_t rd32f(const uint8_t* p) { return p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24); }

// Restore a per-firmware snapshot into its partition (esp_partition; clean if none).
static void restorePartition(const char* label, const String& path) {
    const esp_partition_t* p = dataPart(label);
    if (!p) return;
    esp_partition_erase_range(p, 0, p->size);
    File f = SD.open(path);
    if (!f) return;                                       // no snapshot -> erased/clean
    static uint8_t buf[4096]; uint32_t off = 0;
    while (off < p->size) {
        int g = f.read(buf, sizeof(buf)); if (g <= 0) break;
        while (g & 3) buf[g++] = 0xFF;
        if (esp_partition_write(p, off, buf, g) != ESP_OK) break;
        off += g;
    }
    f.close();
}

// Arm a single boot of ota_0 via the bootflag (Update.end already set otadata->ota_0).
static void armBootOnce() {
    const esp_partition_t* bf = dataPart("bootflag");
    if (!bf) return;
    uint32_t magic = 0xB007A001u;
    esp_partition_erase_range(bf, 0, bf->size);
    esp_partition_write(bf, 0, &magic, sizeof(magic));
}

// Write `len` bytes from SD file f (at src) into the named data partition.
static bool writeImagePartition(File& f, uint32_t src, uint32_t len, const char* label, const char* msg) {
    const esp_partition_t* p = dataPart(label);
    if (!p) return false;
    if (len > p->size) len = p->size;
    esp_partition_erase_range(p, 0, p->size);
    f.seek(src);
    static uint8_t buf[4096]; uint32_t done = 0;
    while (done < len) {
        uint32_t want = len - done; if (want > sizeof(buf)) want = sizeof(buf);
        int g = f.read(buf, want); if (g <= 0) return false;
        int wl = g; while (wl & 3) buf[wl++] = 0xFF;
        if (esp_partition_write(p, done, buf, wl) != ESP_OK) return false;
        done += g;
        if ((done & 0x3ffff) == 0 || done >= len) drawProgressBar(msg, done, len);
    }
    return true;
}

// =====================================================================
//  Launcher. Uses the proven Update/esp_partition APIs and a FIXED table
//  (raw esp_flash_* crashes under Arduino, so we don't rewrite the table).
//    full == true  : M5Burner full image -> write app into ota_0 AND its
//                     bundled data partitions (e.g. Marauder's SPIFFS).
//    full == false : raw app image -> write into ota_0.
//  RESET returns to the loader via the bootflag bootloader hook.
// =====================================================================
static bool buildAndLaunch(File& f, const String& fwId, bool full) {
    uint32_t appOff = 0, appLen = 0;
    struct { uint8_t sub; uint32_t off, sz; char lbl[17]; } data[6]; int nd = 0;

    screenMsg("Preparing launch...", full ? "full image" : "app image", COL_OK, 500);
    int dataTooBig = 0;
    if (full) {
        static uint8_t ft[0xc00]; f.seek(0x8000);
        if (f.read(ft, sizeof(ft)) != (int)sizeof(ft)) { screenMsg("Bad image (table)", nullptr, COL_ERR, 3000); return false; }
        for (int i = 0; i < (int)(sizeof(ft) / 32); i++) {
            const uint8_t* e = ft + i * 32; if (e[0] != 0xAA || e[1] != 0x50) break;
            uint8_t t = e[2], s = e[3]; uint32_t o = rd32f(e + 4), z = rd32f(e + 8);
            if (t == 0 && !appOff) appOff = o;
            if (t == 1 && (s == 0x81 || s == 0x82 || s == 0x83) && nd < 6) {   // fat/spiffs/littlefs
                data[nd].sub = s; data[nd].off = o; data[nd].sz = z;
                memset(data[nd].lbl, 0, 17); memcpy(data[nd].lbl, e + 12, 16); nd++;
            }
        }
        if (!appOff) { screenMsg("Bad image (no app)", nullptr, COL_ERR, 3000); return false; }
        // warn if a bundled data partition is bigger than ours (its FS won't mount)
        for (int i = 0; i < nd; i++) { const esp_partition_t* dp = dataPart(data[i].lbl); if (dp && data[i].sz > dp->size) dataTooBig = data[i].sz / 1024; }
        if (dataTooBig) screenMsg("Warning: data too big", (String(dataTooBig) + "K - may not run").c_str(), COL_ERR, 2500);
    }
    // app image length (walk ESP image header + segments)
    uint8_t hdr[24]; f.seek(appOff);
    if (f.read(hdr, 24) != 24 || hdr[0] != 0xE9) { screenMsg("Bad app image", nullptr, COL_ERR, 3000); return false; }
    int segs = hdr[1]; bool hashapp = (hdr[23] == 1); appLen = 24;
    for (int s = 0; s < segs; s++) { uint8_t sh[8]; f.seek(appOff + appLen); if (f.read(sh, 8) != 8) { screenMsg("Bad app (segs)", nullptr, COL_ERR, 3000); return false; } appLen += 8 + rd32f(sh + 4); }
    appLen += 1; if (appLen % 16) appLen += 16 - (appLen % 16); if (hashapp) appLen += 32;

    // 1) write the app into ota_0 via the Update API
    const esp_partition_t* ota = esp_ota_get_next_update_partition(NULL);
    if (!ota || appLen > ota->size) { screenMsg("App too big for ota_0", (String(appLen/1024)+"K > "+String((ota?ota->size:0)/1024)+"K").c_str(), COL_ERR, 4500); return false; }
    if (!Update.begin(appLen, U_FLASH)) { screenMsg("Update.begin failed", Update.errorString(), COL_ERR, 3500); return false; }
    Update.onProgress([](size_t c, size_t t) { drawProgressBar("Installing app", c, t); });
    f.seek(appOff);
    static uint8_t buf[4096]; uint32_t done = 0;     // static: keep the 4KB buffer off the 8KB task stack
    while (done < appLen) {
        uint32_t want = appLen - done; if (want > sizeof(buf)) want = sizeof(buf);
        int g = f.read(buf, want); if (g <= 0) break;
        if (Update.write(buf, g) != (size_t)g) { screenMsg("App write failed", Update.errorString(), COL_ERR, 3500); Update.abort(); return false; }
        done += g;
    }
    if (done != appLen || !Update.end(true)) { screenMsg("Finalize failed", Update.errorString(), COL_ERR, 3500); return false; }

    // 2) write bundled data partitions (e.g. Marauder's SPIFFS) into matching labels
    for (int i = 0; i < nd; i++) writeImagePartition(f, data[i].off, data[i].sz, data[i].lbl, "Installing data");

    // 3) per-firmware persistence: restore saved NVS (settings). For app-only also
    //    restore its SPIFFS snapshot; full images take SPIFFS from the image itself.
    String dir = dataDirFor(fwId);
    restorePartition("nvs", dir + "/nvs.bin");
    if (!full) restorePartition("spiffs", dir + "/spiffs.bin");

    prefs.putString("lastfw", fwId);
    prefs.putBool("lastfull", full);
    armBootOnce();
    screenMsg("Launching...", "RESET returns to loader", COL_OK, 1200);
    esp_restart();
    return true;
}

static void flashFromSD(const String& path) {
    String fwId = fwIdFromPath(path);
    File f = SD.open(path);
    if (!f) { screenMsg("Open failed", path.c_str(), COL_ERR, 2000); return; }
    size_t sz = f.size();
    if (sz < 0x2000) { screenMsg("File too small", nullptr, COL_ERR, 2000); f.close(); return; }
    bool full = false;                                    // full image? (partition table magic at 0x8000)
    if (sz > 0x9000) { uint8_t m[2]; f.seek(0x8000); if (f.read(m, 2) == 2 && m[0] == 0xAA && m[1] == 0x50) full = true; }
    bool ok = buildAndLaunch(f, fwId, full);              // reboots on success
    f.close();
    if (!ok) screenMsg("Launch failed", "incompatible image?", COL_ERR, 3000);
}

// =====================================================================
//  WiFi
// =====================================================================
static bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    setCpuFrequencyMhz(240);   // full speed for WiFi/TLS; loop() drops back to 80 when idle
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("pass", "");
    if (ssid.isEmpty()) { screenMsg("No WiFi set", "Settings > WiFi Setup", COL_ERR, 2500); return false; }
    screenMsg("Connecting WiFi...", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        M5Cardputer.update();
        delay(150);
    }
    if (WiFi.status() != WL_CONNECTED) { screenMsg("WiFi failed", nullptr, COL_ERR, 2500); return false; }
    return true;
}

// =====================================================================
//  M5Burner catalog
//  ---------------------------------------------------------------------
//  The M5Burner firmware list is a single ~2.2 MB JSON array — far bigger
//  than RAM. We stream it, split it into individual objects with a brace
//  counter, keep only category=="cardputer", and write a slim index
//  (name<TAB>version<TAB>file) to the SD card. The OTA menu then reads
//  that small file. (Algorithm validated against the live API.)
// =====================================================================

// Stream the M5Burner catalog and write the slim Cardputer index to SD.
static bool fetchCatalog() {
    if (!sdOK && !mountSD()) { screenMsg("No SD card", nullptr, COL_ERR, 2000); return false; }
    if (!connectWiFi()) return false;

    screenMsg("Fetching catalog...", "from M5Burner");
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, M5B_LIST)) { screenMsg("HTTP begin failed", nullptr, COL_ERR, 2500); return false; }
    http.setUserAgent("M5Burner");
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        screenMsg("Catalog HTTP error", String(code).c_str(), COL_ERR, 3000);
        http.end(); return false;
    }
    WiFiClient* s = http.getStreamPtr();

    SD.remove(CAT_TSV);
    File out = SD.open(CAT_TSV, FILE_WRITE);
    if (!out) { http.end(); screenMsg("SD write failed", nullptr, COL_ERR, 2500); return false; }

    // Streaming JSON tokenizer — no large buffer (avoids OOM under TLS).
    // We only track the fields we need. Strings are captured into a small
    // fixed token buffer; a string is a KEY if the next non-space char is
    // ':' else it's a VALUE for the most recent interesting key.
    int  depth = 0, found = 0, scanned = 0;
    bool instr = false, esc = false;
    char tok[96]; int tlen = 0;                 // current string token (truncated, that's fine)
    bool havePending = false;                   // a completed string awaits key/value classification
    int  curKey = 0;                            // 1=category 2=name 3=version 4=file (awaiting value)
    bool isCard = false;
    String curName, curVer, curFile;
    uint8_t rb[512];
    uint32_t lastUI = 0;

    auto classifyKey = [&](const char* k) {
        if (!strcmp(k, "category")) return 1;
        if (!strcmp(k, "name"))     return 2;
        if (!strcmp(k, "version"))  return 3;
        if (!strcmp(k, "file"))     return 4;
        return 0;
    };
    auto applyValue = [&](const char* v) {
        switch (curKey) {
            case 1: { String t = v; t.toLowerCase(); if (t == M5B_CAT) isCard = true; } break;
            case 2: curName = v; break;
            case 3: curVer  = v; break;          // last wins == latest version
            case 4: curFile = v; break;
        }
        curKey = 0;
    };

    while (http.connected() || s->available()) {
        int n = s->available() ? s->readBytes(rb, sizeof(rb)) : 0;
        if (n <= 0) { if (!http.connected()) break; delay(2); continue; }
        for (int i = 0; i < n; i++) {
            char c = (char)rb[i];
            if (instr) {                          // inside a string literal
                if (esc) { if (tlen < 95) tok[tlen++] = c; esc = false; }
                else if (c == '\\') esc = true;
                else if (c == '"') { tok[tlen] = 0; havePending = true; instr = false; }
                else if (tlen < 95) tok[tlen++] = c;
                continue;
            }
            switch (c) {
                case '"': tlen = 0; instr = true; break;
                case ':':                          // pending string was a KEY
                    if (havePending) { curKey = classifyKey(tok); havePending = false; }
                    break;
                case '{':
                    if (depth == 0) { isCard = false; curName = ""; curVer = ""; curFile = ""; curKey = 0; }
                    depth++; havePending = false; break;
                case ',':
                case ']':
                    if (havePending) { applyValue(tok); havePending = false; } else curKey = 0;
                    break;
                case '}':
                    if (havePending) { applyValue(tok); havePending = false; } else curKey = 0;
                    if (depth > 0) depth--;
                    if (depth == 0) {              // end of a top-level entry
                        scanned++;
                        if (isCard && curName.length() && curFile.endsWith(".bin")) {
                            curName.replace('\t', ' '); curName.replace('\n', ' ');
                            out.printf("%s\t%s\t%s\n", curName.c_str(), curVer.c_str(), curFile.c_str());
                            found++;
                        }
                    }
                    break;
            }
        }
        if (millis() - lastUI > 250) {            // live progress
            lastUI = millis();
            cv.fillScreen(COL_BG);
            cv.setTextColor(COL_TXT, COL_BG); cv.setTextDatum(middle_center); cv.setTextSize(FONT_BIG);
            cv.drawString("Fetching catalog", SCR_W / 2, 50);
            cv.drawString(String(found) + " cardputer", SCR_W / 2, 78);
            cv.setTextSize(FONT_SMALL); cv.drawString(String(scanned) + " scanned", SCR_W / 2, 104);
            cv.pushSprite(0, 0);
        }
        M5Cardputer.update();
    }
    out.close();
    http.end();
    screenMsg("Catalog updated", (String(found) + " firmwares").c_str(), COL_OK, 1500);
    return found > 0;
}

// safeName() moved to fwutil.h

// Download any URL to an SD file, following redirects, with a progress bar.
static bool downloadURLToFile(const String& url, const String& dst, const char* ua) {
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    http.setUserAgent(ua);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // GitHub assets redirect to a CDN
    if (!http.begin(client, url)) { screenMsg("HTTP begin failed", nullptr, COL_ERR, 2500); return false; }
    int code = http.GET();
    if (code != HTTP_CODE_OK) { screenMsg("HTTP error", String(code).c_str(), COL_ERR, 3000); http.end(); return false; }

    int total = http.getSize();
    WiFiClient* s = http.getStreamPtr();
    SD.remove(dst);
    File of = SD.open(dst, FILE_WRITE);
    if (!of) { http.end(); screenMsg("SD open failed", nullptr, COL_ERR, 2500); return false; }

    uint8_t buf[2048]; uint32_t got = 0; uint32_t lastUI = 0;
    while (http.connected() || s->available()) {
        int n = s->available() ? s->readBytes(buf, sizeof(buf)) : 0;
        if (n <= 0) { if (!http.connected()) break; delay(2); continue; }
        of.write(buf, n); got += n;
        if (millis() - lastUI > 200) { lastUI = millis(); drawProgressBar("Downloading", got, total > 0 ? total : 0); }
        M5Cardputer.update();
    }
    of.close(); http.end();
    if (total > 0 && (int)got < total) { SD.remove(dst); screenMsg("Download incomplete", nullptr, COL_ERR, 3000); return false; }
    return got > 0;
}

// Download a firmware URL to SD and keep the file AS-IS (full image or app).
// The partition manager (buildAndLaunch) handles either form at launch time —
// full images keep their bundled data partitions (e.g. Marauder's SPIFFS).
static void installFromURL(const String& url, const String& name, const char* ua) {
    if (!sdOK && !mountSD()) { screenMsg("No SD card", nullptr, COL_ERR, 2000); return; }
    if (!connectWiFi()) return;
    screenMsg("Downloading...", name.c_str());

    String path = String(FW_DIR) + "/" + safeName(name) + ".bin";
    String tmp  = String(FW_DIR) + "/.dl.tmp";
    if (!downloadURLToFile(url, tmp, ua)) return;
    SD.remove(path);
    if (SD.rename(tmp, path)) screenMsg("Saved to SD!", (safeName(name) + ".bin").c_str(), COL_OK, 2200);
    else { SD.remove(tmp); screenMsg("Install failed", nullptr, COL_ERR, 3000); }
}

// M5Burner download (full-flash image kept whole; partition-managed at launch).
static void downloadToSD(const String& file, const String& name) {
    installFromURL(String(M5B_CDN) + file, name, "M5Burner");
}

// ---- GitHub releases --------------------------------------------------
// Fetch a repo's latest release, list its .bin assets, install the chosen one.
static void githubInstall() {
    if (!sdOK && !mountSD()) { screenMsg("No SD card", nullptr, COL_ERR, 2000); return; }
    String repo = prefs.getString("ghrepo", "pr3y/Bruce");
    if (!textInput("GitHub owner/repo", repo)) return;
    prefs.putString("ghrepo", repo);
    if (!connectWiFi()) return;

    String api = "https://api.github.com/repos/" + repo + "/releases/latest";
    screenMsg("Fetching release...", repo.c_str());
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    http.setUserAgent("M5CardputerLoader");
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(client, api)) { screenMsg("HTTP begin failed", nullptr, COL_ERR, 2500); return; }
    int code = http.GET();
    if (code != HTTP_CODE_OK) { screenMsg("GitHub HTTP err", String(code).c_str(), COL_ERR, 3000); http.end(); return; }

    // Stream the release JSON, pulling out browser_download_url values ending in .bin
    WiFiClient* s = http.getStreamPtr();
    std::vector<String> urls, labels;
    String token; bool instr = false, esc = false; bool capUrl = false;
    const char* KEY = "browser_download_url";
    while ((http.connected() || s->available()) && urls.size() < 40) {
        int c = s->read();
        if (c < 0) { if (!http.connected()) break; delay(2); continue; }
        char ch = (char)c;
        if (instr) {
            if (esc) { token += ch; esc = false; }
            else if (ch == '\\') esc = true;
            else if (ch == '"') {
                instr = false;
                if (capUrl) { if (token.endsWith(".bin")) { urls.push_back(token); int sl = token.lastIndexOf('/'); labels.push_back(sl >= 0 ? token.substring(sl + 1) : token); } capUrl = false; }
                else if (token == KEY) capUrl = true;   // next string is the URL value
            } else if (token.length() < 256) token += ch;
        } else if (ch == '"') { token = ""; instr = true; }
    }
    http.end();

    if (urls.empty()) { screenMsg("No .bin assets", "in latest release", COL_ERR, 3000); return; }
    int sel = listMenu("GitHub assets", labels, "ENTER=install  `=back");
    if (sel < 0 || sel >= (int)urls.size()) return;
    installFromURL(urls[sel], labels[sel].substring(0, labels[sel].lastIndexOf('.')), "M5CardputerLoader");
}

// OTA menu: show the cached M5Burner Cardputer catalog from SD, refresh,
// and download the selected firmware (app-extracted) onto the SD card.
static String g_catFilter;          // session search term for the OTA catalog (empty = show all)

static void otaMenu() {
    if (!sdOK && !mountSD()) { screenMsg("No SD card", nullptr, COL_ERR, 2000); return; }
    if (!SD.exists(CAT_TSV)) { if (!fetchCatalog()) return; }

    for (;;) {
        std::vector<String> names, files;
        String flt = g_catFilter; flt.toLowerCase();
        int total = 0;
        File f = SD.open(CAT_TSV);
        if (f) {
            while (f.available()) {
                String line = f.readStringUntil('\n'); line.trim();
                if (line.isEmpty()) continue;
                int t1 = line.indexOf('\t'); int t2 = line.indexOf('\t', t1 + 1);
                if (t1 < 0 || t2 < 0) continue;
                String nm = line.substring(0, t1);
                String ver = line.substring(t1 + 1, t2);
                String fil = line.substring(t2 + 1);
                total++;
                if (flt.length()) { String hay = nm; hay.toLowerCase(); if (hay.indexOf(flt) < 0) continue; }
                names.push_back(nm + " " + ver);
                files.push_back(fil);
            }
            f.close();
        }

        std::vector<String> disp;
        disp.push_back("** Refresh catalog **");
        disp.push_back("** Install from GitHub **");
        disp.push_back(g_catFilter.length() ? String("** Filter: ") + g_catFilter + " **"
                                            : String("** Search... **"));
        const int HDR = 3;
        for (auto& n : names) disp.push_back(n);

        String title = g_catFilter.length() ? String(names.size()) + "/" + total + " match"
                                            : String("OTA -> SD (") + total + ")";
        int sel = listMenu(title.c_str(), disp, "ENTER=get  `=back");
        if (sel < 0) return;
        if (sel <= 1) {                                   // network actions (refresh / GitHub):
            // free the ~544-entry catalog RAM FIRST, else the TLS handshake OOMs (HTTP -1).
            std::vector<String>().swap(names);
            std::vector<String>().swap(files);
            std::vector<String>().swap(disp);
            if (sel == 0) fetchCatalog(); else githubInstall();
            continue;
        }
        if (sel == 2) {                                   // search/filter (blank = clear)
            String q = g_catFilter;
            if (textInput("Search (blank=all)", q)) { q.trim(); g_catFilter = q; }
            continue;
        }
        int idx = sel - HDR;
        if (idx < 0 || idx >= (int)files.size()) continue;

        std::vector<String> conf = { String("Download: ") + names[idx], "YES - save to SD", "NO - cancel" };
        int c = listMenu("Confirm", conf, "ENTER=choose  `=cancel", 1);
        if (c == 1) {
            // Free the big catalog vectors BEFORE the TLS download — otherwise
            // ~544 names in RAM + the canvas starve the mbedTLS handshake (HTTP -1).
            String selFile = files[idx], selName = names[idx];
            std::vector<String>().swap(names);
            std::vector<String>().swap(files);
            std::vector<String>().swap(disp);
            downloadToSD(selFile, selName);
        }
    }
}

// =====================================================================
//  Settings
// =====================================================================
static void wifiSetup() {
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("pass", "");
    if (!textInput("WiFi SSID", ssid)) return;
    if (!textInput("WiFi Password", pass, true)) return;
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    WiFi.disconnect(true);          // drop any stale session so the new creds are used
    screenMsg("WiFi saved", ssid.c_str(), COL_OK, 1500);
}

// Connect and verify connectivity; show IP + signal so you know it works.
static void wifiTest() {
    WiFi.disconnect(true);
    if (!connectWiFi()) return;     // shows its own "WiFi failed" on error
    std::vector<String> info = {
        "Connected!",
        "SSID: " + WiFi.SSID(),
        "IP:   " + WiFi.localIP().toString(),
        "RSSI: " + String(WiFi.RSSI()) + " dBm",
        "GW:   " + WiFi.gatewayIP().toString(),
    };
    listMenu("WiFi Test", info, "`=back");
}

static void settingsMenu() {
    for (;;) {
        int bright = prefs.getInt("bright", 128);
        String wifiState = (WiFi.status() == WL_CONNECTED) ? "connected" : "off";
        std::vector<String> items = {
            "WiFi Setup",
            "WiFi Test (" + wifiState + ")",
            "Brightness: " + String(bright),
            "Rescan SD card",
            String("SD: ") + (sdOK ? "mounted" : "NOT mounted"),
        };
        int sel = listMenu("Settings", items, ";.=move ENTER=set `=back");
        if (sel < 0) return;
        switch (sel) {
            case 0: wifiSetup(); break;
            case 1: wifiTest(); break;
            case 2: {
                bright = (bright + 32) % 288;       // 0..256 wrap
                if (bright > 255) bright = 32;
                prefs.putInt("bright", bright);
                g_bright = bright;
                dsp.setBrightness(bright);
                break;
            }
            case 3: sdOK = false; mountSD();
                    screenMsg("SD", sdOK ? "mounted" : "mount failed",
                              sdOK ? COL_OK : COL_ERR, 1200); break;
            default: break;
        }
    }
}

// =====================================================================
//  WebUI — host a tiny web page over WiFi to upload/delete SD firmwares
//  from a browser on your PC/phone. No card removal needed.
// =====================================================================
static void webUI() {
    if (!sdOK && !mountSD()) { screenMsg("No SD card", nullptr, COL_ERR, 2000); return; }
    if (!connectWiFi()) return;

    WebServer server(80);
    static File uploadFile;          // static: outlives the upload-callback invocations

    auto pageHTML = []() -> String {
        String h = "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
                   "<title>M5Cardputer Loader</title></head><body style='font-family:sans-serif;margin:1em'>"
                   "<h2>M5Cardputer Loader</h2>"
                   "<form method='POST' action='/upload' enctype='multipart/form-data'>"
                   "<input type=file name=f accept='.bin'> <input type=submit value=Upload></form><hr><h3>/firmware</h3><ul>";
        File dir = SD.open(FW_DIR);
        for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
            if (!f.isDirectory()) {
                String n = f.name(); int sl = n.lastIndexOf('/'); if (sl >= 0) n = n.substring(sl + 1);
                String low = n; low.toLowerCase();
                if (low.endsWith(".bin"))
                    h += "<li>" + n + " (" + String((uint32_t)(f.size() / 1024)) + "K) "
                         "<a href='/delete?f=" + n + "'>[delete]</a></li>";
            }
            f.close();
        }
        dir.close();
        h += "</ul><p>Upload app .bin images; they appear under SD Firmware.</p></body></html>";
        return h;
    };

    String ip = WiFi.localIP().toString();
    auto drawInfo = [&ip]() {
        std::vector<String> info = { "WebUI running", "http://" + ip, "Open in browser to", "upload .bin files", "` = stop" };
        cv.fillScreen(COL_BG); drawHeader("WebUI Upload");
        cv.setTextSize(FONT_BIG); cv.setTextColor(COL_TXT, COL_BG); cv.setTextDatum(top_left);
        int y = LIST_Y + 2; for (auto& l : info) { cv.drawString(l, 6, y); y += ROW_H; }
        cv.pushSprite(0, 0);
    };
    static bool uploadDone = false;

    server.on("/", HTTP_GET, [&server, pageHTML]() { server.send(200, "text/html", pageHTML()); });
    server.on("/delete", HTTP_GET, [&server]() {
        if (server.hasArg("f")) SD.remove(String(FW_DIR) + "/" + server.arg("f"));
        server.sendHeader("Location", "/"); server.send(303, "text/plain", "");
    });
    server.on("/upload", HTTP_POST,
        [&server]() { server.sendHeader("Location", "/"); server.send(303, "text/plain", ""); },
        [&server]() {
            HTTPUpload& up = server.upload();
            if (up.status == UPLOAD_FILE_START) {
                String fn = up.filename; int sl = fn.lastIndexOf('/'); if (sl >= 0) fn = fn.substring(sl + 1);
                String low = fn; low.toLowerCase(); if (!low.endsWith(".bin")) fn += ".bin";
                String p = String(FW_DIR) + "/" + fn;
                SD.remove(p); uploadFile = SD.open(p, FILE_WRITE);
                drawProgressBar("Uploading", 0, 0);
            } else if (up.status == UPLOAD_FILE_WRITE) {
                if (uploadFile) uploadFile.write(up.buf, up.currentSize);
                // Content-Length ~= file size (+ small multipart overhead) -> usable progress bar
                int total = server.header("Content-Length").toInt();
                drawProgressBar("Uploading", up.totalSize, total > 0 ? total : 0);
            } else if (up.status == UPLOAD_FILE_END) {
                if (uploadFile) uploadFile.close();
                uploadDone = true;
            }
        });
    const char* hdrKeys[] = { "Content-Length" };           // read it in the upload handler for progress
    server.collectHeaders(hdrKeys, 1);
    server.begin();

    drawInfo();
    for (;;) {
        server.handleClient();
        if (uploadDone) { uploadDone = false; screenMsg("Uploaded!", nullptr, COL_OK, 1000); drawInfo(); }
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            auto st = M5Cardputer.Keyboard.keysState();
            bool back = st.del; for (char c : st.word) if (c == '`') back = true;
            if (back) break;
        }
        delay(2);
    }
    server.stop();
    screenMsg("WebUI stopped", nullptr, COL_OK, 800);
}

// Format an idle timeout (ms) compactly: 0->"off", whole minutes->"Nm", else "Ns".
static String fmtIdle(uint32_t ms) {
    if (!ms) return "off";
    uint32_t s = ms / 1000;
    return (s % 60 == 0) ? String(s / 60) + "m" : String(s) + "s";
}

// Live Power-menu rows (re-pulled ~1x/sec by listMenu so battery/voltage update).
static std::vector<String> powerRows() {
    updatePower();
    const char* st = g_chgState == 1 ? "charging" : g_chgState == 2 ? "charged" : "on battery";
    return {
        String("Battery: ") + (g_battPct >= 0 ? String(g_battPct) + "%" : String("n/a")),
        String("Voltage: ") + String(g_battMv) + " mV",
        String("State:   ") + st,
        String("Dim after:  ") + fmtIdle(g_dimMs),
        String("Screen off: ") + fmtIdle(g_offMs),
        "Power Off",
        "Deep Sleep",
    };
}

static void powerMenu() {
    int start = 0;
    for (;;) {
        int sel = listMenu("Power", powerRows(), ";.=move ENTER=set `=back", start, COL_TXT, 0, powerRows);
        if (sel < 0) return;
        start = sel;                                     // keep position across setting changes
        if (sel == 3) {                                  // cycle idle -> dim timeout
            const uint32_t opts[] = { 0, 30, 60, 120, 180, 300 }; int cur = 0;
            for (int i = 0; i < 6; i++) if (opts[i] * 1000 == g_dimMs) cur = i;
            g_dimMs = opts[(cur + 1) % 6] * 1000;
            prefs.putUInt("dimms", g_dimMs);
            lastActivity = millis(); dimState = 0; dsp.setBrightness(g_bright);
        } else if (sel == 4) {                           // cycle idle -> screen-off timeout
            const uint32_t opts[] = { 0, 60, 120, 300, 600, 900 }; int cur = 0;
            for (int i = 0; i < 6; i++) if (opts[i] * 1000 == g_offMs) cur = i;
            g_offMs = opts[(cur + 1) % 6] * 1000;
            prefs.putUInt("offms", g_offMs);
            lastActivity = millis(); dimState = 0; dsp.setBrightness(g_bright);
        } else if (sel == 5) {
            screenMsg("Powering off...", nullptr, COL_OK, 800); M5Cardputer.Power.powerOff();
        } else if (sel == 6) {
            screenMsg("Deep sleep...", "power btn to wake", COL_OK, 800); M5Cardputer.Power.deepSleep();
        }
    }
}

static void aboutScreen() {
    const esp_partition_t* run = esp_ota_get_running_partition();
    std::vector<String> info = {
        "M5Cardputer Loader",
        "ESP32-S3FN8  8MB flash",
        "Display 240x135 ST7789V2",
        String("Running: ") + (run ? run->label : "?"),
        "SD: /firmware/*.bin",
        "OTA: M5Burner catalog",
        "Controls: ; up . down",
        "ENTER select  ` back",
#ifdef LOADER_ROLLBACK
        "Return: RESET (rollback)",
#else
        "Return: app loader_return.h",
#endif
    };
    listMenu("About", info, "`=back");
}

// =====================================================================
//  Main menu
// =====================================================================

// Action menu for one installed firmware: Launch / Rename / Delete.
static void firmwareAction(const String& name, const String& path) {
    std::vector<String> act = { name, "Launch", "Rename", "Delete", "Cancel" };
    int a = listMenu("Firmware", act, "ENTER=choose  `=back", 1);
    if (a == 1) { flashFromSD(path); }
    else if (a == 2) {                                       // rename
        String nn = path;                                    // derive editable base from the real filename
        int sl = nn.lastIndexOf('/'); if (sl >= 0) nn = nn.substring(sl + 1);
        int dot = nn.lastIndexOf('.'); if (dot > 0) nn = nn.substring(0, dot); // strip .bin
        if (textInput("New name", nn) && nn.length()) {
            String np = String(FW_DIR) + "/" + safeName(nn) + ".bin";
            if (SD.rename(path, np)) screenMsg("Renamed", nullptr, COL_OK, 1200);
            else screenMsg("Rename failed", nullptr, COL_ERR, 2000);
        }
    }
    else if (a == 3) {                                       // delete
        std::vector<String> conf = { String("Delete ") + name + "?", "YES - delete", "NO - keep" };
        int c = listMenu("Confirm", conf, "ENTER=choose  `=cancel", 1);
        if (c == 1) { SD.remove(path); screenMsg("Deleted", nullptr, COL_OK, 1000); }
    }
}

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    dsp.setRotation(1);
    dsp.setTextSize(1);
    cv.setColorDepth(8);             // 8-bit halves sprite RAM (32KB) -> more heap for TLS
    cv.createSprite(SCR_W, SCR_H);   // off-screen buffer for flicker-free drawing

    prefs.begin("loader", false, "nvs_ldr");   // loader's OWN nvs partition (apps can't clobber it)
    WiFi.persistent(false);                     // keep WiFi creds out of the app "nvs" partition
    g_bright = prefs.getInt("bright", 128);
    g_dimMs  = prefs.getUInt("dimms", 120000);   // dim after 2 min idle (configurable)
    g_offMs  = prefs.getUInt("offms", 300000);   // screen off after 5 min idle (configurable)
    lastActivity = millis();
    dsp.setBrightness(g_bright);
    setCpuFrequencyMhz(80);   // idle at 80MHz: the UI needs no more, and the lower draw
                              // lets the ~62mA hardware charger out-pace the system load
    updatePower(true);                          // prime battery reading before first draw

    // If we are an OTA app under a rollback build, the loader (factory)
    // is never rolled back; nothing to mark here. Loaded apps that omit
    // esp_ota_mark_app_valid_cancel_rollback() will roll back to us.

    screenMsg("M5Cardputer Loader", "starting...", COL_OK, 600);
    mountSD();
    persistLastFirmware();   // snapshot the firmware that just ran (per-firmware data)
}

void loop() {
    if (getCpuFrequencyMhz() != 80) setCpuFrequencyMhz(80);  // back to low-power idle after WiFi/downloads
    // Installed firmwares come first (each launchable), then the functions.
    std::vector<String> names, paths;
    scanFirmware(names, paths);
    int fwCount = names.size();

    std::vector<String> items = names;            // firmwares at the top (drawn bright orange)
    const char* funcs[] = { "-  OTA Install  -", "-  WebUI Upload  -", "-  Power  -",
                            "-  Settings  -", "-  About  -", "-  Reboot  -" };
    for (auto& fn : funcs) items.push_back(fn);

    uint64_t freeMB = sdOK ? (SD.totalBytes() - SD.usedBytes()) / (1024 * 1024) : 0;
    String footer = fwCount ? (String(fwCount) + " fw  " + String((uint32_t)freeMB) + "MB free")
                            : "no firmware on SD";

    int sel = listMenu("M5Cardputer Loader", items, footer.c_str(), 0, COL_FW, fwCount);
    if (sel < 0) return;
    if (sel < fwCount) { firmwareAction(names[sel], paths[sel]); return; }

    switch (sel - fwCount) {
        case 0: otaMenu(); break;
        case 1: webUI(); break;
        case 2: powerMenu(); break;
        case 3: settingsMenu(); break;
        case 4: aboutScreen(); break;
        case 5: screenMsg("Rebooting...", nullptr, COL_OK, 800); esp_restart(); break;
        default: break;
    }
}
