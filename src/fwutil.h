// =====================================================================
//  fwutil.h — pure, hardware-independent helpers extracted from main.cpp
//  ---------------------------------------------------------------------
//  These functions touch no hardware: they operate only on an Arduino-
//  compatible `String` (provided by the core on-device, or by the shim in
//  test/arduino_compat.h for host unit tests). Keep it that way — do NOT add
//  Arduino.h / M5 / SD / esp_* includes here, so the file stays testable with
//  a plain host compiler. Unit tests live in test/test_fwutil.cpp.
// =====================================================================
#pragma once
#include <ctype.h>
#include <stddef.h>

// Single-cell Li-Po voltage(mV) -> % (piecewise-linear; light-load curve).
static int lipoPct(int mv) {
    static const int V[] = {3300,3500,3600,3700,3750,3800,3850,3900,3950,4000,4100,4200};
    static const int P[] = {   0,  10,  20,  35,  45,  55,  62,  70,  78,  85,  95, 100};
    const int N = sizeof(V) / sizeof(V[0]);
    if (mv <= V[0])   return 0;
    if (mv >= V[N-1]) return 100;
    for (int i = 1; i < N; i++)
        if (mv < V[i]) return P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) / (V[i] - V[i-1]);
    return 100;
}

// Known firmwares: if the (lowercased) filename contains `key`, show `label`.
// Most specific keys first. Build/board/date tokens in the filename are ignored.
struct FwAlias { const char* key; const char* label; };
static const FwAlias FW_ALIASES[] = {
    {"marauder",       "Marauder"},
    {"ghostesp",       "GhostESP"},
    {"ghost",          "GhostESP"},
    {"porkchop",       "Porkchop"},
    {"bruce",          "Bruce"},
    {"nemo",           "NEMO"},
    {"evil",           "Evil-Cardputer"},
    {"ultimateremote", "UltimateRemote"},
    {"ultimate",       "UltimateRemote"},
    {"poseidon",       "Poseidon"},
    {"killerhack",     "KillerHack"},
    {"raisinghell",    "Raising Hell"},
    {"raising",        "Raising Hell"},
    {"bit-pirate",     "Bit-Pirate"},
    {"bitpirate",      "Bit-Pirate"},
    {"uiflow",         "UIFlow"},
    {"launcher",       "M5Launcher"},
};

// Pull a clean version like "1.12.3" out of a filename. Prefers a 'v<digit>'
// marker; skips date/build groups (a run of >4 digits, e.g. 20260622). "" if none.
static String extractVersion(const String& low) {
    int n = low.length(), start = -1;
    for (int i = 0; i + 1 < n; i++)                       // explicit 'v' + digit
        if (low[i] == 'v' && isdigit((unsigned char)low[i + 1]) &&
            (i == 0 || !isalnum((unsigned char)low[i - 1]))) { start = i + 1; break; }
    if (start < 0)                                        // else a digit just after a separator
        for (int i = 1; i < n; i++)
            if (isdigit((unsigned char)low[i]) &&
                (low[i-1]=='.'||low[i-1]=='_'||low[i-1]==' '||low[i-1]=='-')) { start = i; break; }
    if (start < 0) return "";
    String ver; int groups = 0, i = start;
    while (i < n && groups < 4) {
        int j = i; while (j < n && isdigit((unsigned char)low[j])) j++;
        int len = j - i;
        if (len == 0 || len > 4) break;                   // 0 = not a number, >4 = date/build
        if (groups) ver += ".";
        ver += low.substring(i, j);
        groups++;
        if (j < n && (low[j]=='.'||low[j]=='_') && j+1 < n && isdigit((unsigned char)low[j+1])) i = j + 1;
        else break;
    }
    return ver;
}

// Turn an SD filename into a readable display name. Recognizes known firmwares
// by keyword ("...marauder_v1_12_3..." -> "Marauder 1.12.3"); otherwise tidies
// the raw name ("NEMO_v3.bin" -> "NEMO v3").
static String prettyName(String fn) {
    int slash = fn.lastIndexOf('/'); if (slash >= 0) fn = fn.substring(slash + 1);
    String low = fn; low.toLowerCase();
    if (low.endsWith(".bin")) { fn = fn.substring(0, fn.length() - 4); low = fn; low.toLowerCase(); }
    const char* tails[] = { ".tar", ".gz", ".zip", ".xz", ".elf" };  // strip archive/junk tails
    for (const char* t : tails) { int p = low.indexOf(t); if (p >= 0) { fn = fn.substring(0, p); low = fn; low.toLowerCase(); break; } }

    for (const FwAlias& a : FW_ALIASES) {                 // known firmware -> clean label (+ version)
        if (low.indexOf(a.key) >= 0) {
            String ver = extractVersion(low), out = a.label;
            if (ver.length()) out += " " + ver;
            return out;
        }
    }
    fn.replace('_', ' ');                                  // fallback: tidy the raw name
    while (fn.indexOf("  ") >= 0) fn.replace("  ", " ");
    fn.trim();
    if (fn.isEmpty()) fn = "firmware";
    return fn;
}

// Turn a display name into a safe SD filename (no extension).
static String safeName(const String& name) {
    String s; for (size_t i = 0; i < name.length() && s.length() < 28; i++) {
        char c = name[i];
        if (isalnum((unsigned char)c) || c == '_' || c == '-') s += c;
        else if (c == ' ') s += '_';
    }
    if (s.isEmpty()) s = "fw";
    return s;
}

static String fwIdFromPath(const String& path) {       // basename without .bin
    String b = path; int sl = b.lastIndexOf('/'); if (sl >= 0) b = b.substring(sl + 1);
    int dot = b.lastIndexOf('.'); if (dot > 0) b = b.substring(0, dot);
    return b;
}
