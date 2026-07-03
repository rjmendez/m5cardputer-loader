// Host unit tests for src/fwutil.h. Compile + run with a plain host compiler:
//   g++ -std=c++17 -Wall -Wextra -o t test/test_fwutil.cpp && ./t
// Returns non-zero if any check fails. See .github/workflows/test.yml.
#include "arduino_compat.h"     // host String shim (must precede fwutil.h)
#include "../src/fwutil.h"      // functions under test
#include <cstdio>
#include <cstring>

static int checks = 0, failures = 0;

static void eq_str(const char* what, const String& got, const char* want) {
    checks++;
    if (strcmp(got.c_str(), want) != 0) {
        failures++;
        printf("FAIL %-22s got \"%s\"  want \"%s\"\n", what, got.c_str(), want);
    }
}
static void eq_int(const char* what, int got, int want) {
    checks++;
    if (got != want) { failures++; printf("FAIL %-22s got %d  want %d\n", what, got, want); }
}

int main() {
    // ---- lipoPct: piecewise-linear Li-Po curve ----
    eq_int("lipo below-min",  lipoPct(3000), 0);
    eq_int("lipo at-min",     lipoPct(3300), 0);
    eq_int("lipo above-max",  lipoPct(4300), 100);
    eq_int("lipo at-max",     lipoPct(4200), 100);
    eq_int("lipo knot 3750",  lipoPct(3750), 45);
    eq_int("lipo interp 3550", lipoPct(3550), 15);   // between 3500(10%) and 3600(20%)

    // ---- extractVersion ----
    eq_str("ver v-marker",  extractVersion(String("bruce_v1.2.3")), "1.2.3");
    eq_str("ver separator", extractVersion(String("nemo-3.1")), "3.1");
    eq_str("ver none",      extractVersion(String("marauder")), "");
    eq_str("ver skip-date", extractVersion(String("fw_20260622_v2")), "2");
    eq_str("ver 4-groups",  extractVersion(String("bruce.1.2.3.3")), "1.2.3.3");

    // ---- prettyName ----
    eq_str("pretty marauder", prettyName(String("esp32_marauder_v1_12_3_20260101.bin")), "Marauder 1.12.3");
    eq_str("pretty bruce-tar", prettyName(String("/firmware/Bruce.1.2.3.3.tar.bin")), "Bruce 1.2.3.3");
    eq_str("pretty nemo",     prettyName(String("NEMO_v3.bin")), "NEMO 3");
    eq_str("pretty unknown",  prettyName(String("my_cool_app.bin")), "my cool app");

    // ---- safeName ----
    eq_str("safe spaces", safeName(String("Cool App!")), "Cool_App");
    eq_str("safe all-bad", safeName(String("!!!")), "fw");
    eq_int("safe 28-cap",  (int)safeName(String("0123456789012345678901234567890123456789")).length(), 28);

    // ---- fwIdFromPath ----
    eq_str("fwid path", fwIdFromPath(String("/firmware/Bruce.bin")), "Bruce");
    eq_str("fwid bare", fwIdFromPath(String("thing.bin")), "thing");

    printf("%d checks, %d failure(s)\n", checks, failures);
    return failures ? 1 : 0;
}
