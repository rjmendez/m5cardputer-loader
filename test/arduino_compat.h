// =====================================================================
//  arduino_compat.h — HOST-ONLY minimal Arduino `String` shim
//  ---------------------------------------------------------------------
//  Implements just the subset of the Arduino String API that src/fwutil.h
//  uses, on top of std::string, so those pure helpers can be unit-tested with
//  a plain host compiler. This file is NEVER compiled into the firmware — on
//  device the real String comes from the Arduino core.
// =====================================================================
#pragma once
#include <string>
#include <cctype>

class String {
    std::string s_;
public:
    String() {}
    String(const char* c) : s_(c ? c : "") {}
    String(const std::string& c) : s_(c) {}

    String& operator=(const char* c) { s_ = c ? c : ""; return *this; }

    unsigned int length() const { return (unsigned int)s_.size(); }
    bool isEmpty() const { return s_.empty(); }
    const char* c_str() const { return s_.c_str(); }

    // Arduino returns 0 for an out-of-range index; mirror that.
    char operator[](int i) const { return (i < 0 || (size_t)i >= s_.size()) ? 0 : s_[i]; }

    String substring(int begin) const { return substring(begin, (int)s_.size()); }
    String substring(int begin, int end) const {
        int n = (int)s_.size();
        if (begin < 0) begin = 0;
        if (begin > n) begin = n;
        if (end < begin) end = begin;
        if (end > n) end = n;
        return String(s_.substr(begin, end - begin));
    }

    int indexOf(char c) const { auto p = s_.find(c); return p == std::string::npos ? -1 : (int)p; }
    int indexOf(char c, int from) const { if (from < 0) from = 0; auto p = s_.find(c, from); return p == std::string::npos ? -1 : (int)p; }
    int indexOf(const char* sub) const { auto p = s_.find(sub); return p == std::string::npos ? -1 : (int)p; }
    int indexOf(const String& sub) const { return indexOf(sub.c_str()); }
    int lastIndexOf(char c) const { auto p = s_.rfind(c); return p == std::string::npos ? -1 : (int)p; }

    void toLowerCase() { for (auto& ch : s_) ch = (char)std::tolower((unsigned char)ch); }
    void toUpperCase() { for (auto& ch : s_) ch = (char)std::toupper((unsigned char)ch); }

    bool endsWith(const String& suf) const {
        return s_.size() >= suf.s_.size() && s_.compare(s_.size() - suf.s_.size(), suf.s_.size(), suf.s_) == 0;
    }
    bool startsWith(const String& pre) const {
        return s_.size() >= pre.s_.size() && s_.compare(0, pre.s_.size(), pre.s_) == 0;
    }

    void replace(char f, char r) { for (auto& ch : s_) if (ch == f) ch = r; }
    void replace(const String& f, const String& r) {
        if (f.s_.empty()) return;
        size_t p = 0;
        while ((p = s_.find(f.s_, p)) != std::string::npos) { s_.replace(p, f.s_.size(), r.s_); p += r.s_.size(); }
    }

    void trim() {
        size_t b = 0, e = s_.size();
        while (b < e && std::isspace((unsigned char)s_[b])) b++;
        while (e > b && std::isspace((unsigned char)s_[e - 1])) e--;
        s_ = s_.substr(b, e - b);
    }

    String& operator+=(char c)         { s_ += c; return *this; }
    String& operator+=(const char* c)  { if (c) s_ += c; return *this; }
    String& operator+=(const String& o){ s_ += o.s_; return *this; }

    bool operator==(const String& o) const { return s_ == o.s_; }
    bool operator==(const char* c) const { return s_ == (c ? c : ""); }

    friend String operator+(const String& a, const String& b);
    friend String operator+(const char* a, const String& b);
    friend String operator+(const String& a, const char* b);
    friend String operator+(const String& a, char b);
};

inline String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const char* a,   const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, const char* b)   { String r(a); r += b; return r; }
inline String operator+(const String& a, char b)          { String r(a); r += b; return r; }
