#include "PinScore.h"
#include "x/util/StringUtil.h"
#include <iostream>

using std::string;

namespace Rsyn
{

PinScore::PinScore(const std::string& pinName, double pinLength, bool connect, bool isBound)
    : pinName_{pinName}, pinLength_{pinLength}, connectToUpperLayer_{connect}, isBound_{isBound} {}

double PinScore::length() const { return pinLength_; }

double PinScore::pinLenScore(double maxPnLen) const {
    double baseScore = 10;
    auto percent = pinLength_ / maxPnLen;
    return baseScore * percent;
}

double PinScore::ovlpScore() const {
    double baseScore = 40;
    return connectToUpperLayer_ ? 0 : baseScore;
}

double PinScore::bdPinScore() const {
    double baseScore = 20;
    return isBound_ ? 0 : baseScore;
}

double PinScore::dirScore(double value, double maxVal, int cnt) const {
    double baseScore = 5.0;
    if (maxVal == 0) {
        return baseScore / cnt; // Avoid division by zero
    }
    return (baseScore * value) / (maxVal * cnt);
}

double PinScore::expandSocre(const PinExpand& expand, const PinExpand& maxExpand, int cnt) const {
    return dirScore(expand.acsEMx, maxExpand.acsEMx, cnt) + dirScore(expand.acsNMx, maxExpand.acsNMx, cnt)
            + dirScore(expand.acsSMx, maxExpand.acsSMx, cnt) + dirScore(expand.acsWMx, maxExpand.acsWMx, cnt);
}

double PinScore::expandSocre(PinMetaExpand& maxExpand) const {
    double score = 0;
    for (auto it : expand_) {
        string meta = it.first;
        PinExpand expand = it.second;
        score += expandSocre(expand, maxExpand[meta], maxExpand.size());
    }
    return score;
}

double PinScore::score(double maxPnLen, PinMetaExpand& maxExpand) const {
    return pinLenScore(maxPnLen) + ovlpScore() + bdPinScore() + expandSocre(maxExpand);
}

bool PinScore::isConnected() const { return connectToUpperLayer_; }

bool PinScore::isBdPin() const { return isBound_; }

const std::string& PinScore::name() const { return pinName_; }

void PinScore::setExpandLen(const PinMetaExpand& expand) { expand_ = expand; }

void PinScore::printExpand(PinMetaExpand& maxExpand) {
    for (auto it : expand_) {
        string meta = it.first;
        PinExpand expand = it.second;
        std::cout << padString(std::to_string(dirScore(expand.acsNMx, maxExpand[meta].acsNMx)))
                    << padString(std::to_string(dirScore(expand.acsSMx, maxExpand[meta].acsSMx)))
                    << padString(std::to_string(dirScore(expand.acsWMx, maxExpand[meta].acsWMx)))
                    << padString(std::to_string(dirScore(expand.acsEMx, maxExpand[meta].acsEMx)));
    }
}

} // namespace Rsyn
