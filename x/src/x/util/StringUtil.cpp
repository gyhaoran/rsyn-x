#include "StringUtil.h"

std::string padString(std::string s, int len) {
    if (s.length() < len) {
        s.resize(len, ' ');
    }
    return s;
}
