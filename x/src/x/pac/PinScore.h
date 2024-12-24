#ifndef B14973A6_A791_4EC9_B019_CAFDF05FEE9F
#define B14973A6_A791_4EC9_B019_CAFDF05FEE9F

#include "PinExpand.h"
#include <string>

namespace Rsyn
{

class PinScore {
public:
    explicit PinScore(const std::string& pinName, double pinLength=0, bool connect=false, bool isBound=false);

    double pinLenScore(double maxPnLen) const;
    double score(double maxPnLen, PinMetaExpand& maxExpand) const;

    bool isConnected() const;
    bool isBdPin() const;
    const std::string& name() const;
    double length() const;

    void setExpandLen(const PinMetaExpand& expand);
    void printExpand(PinMetaExpand& maxExpand);

private:
    double ovlpScore() const;
    double bdPinScore() const;
    double dirScore(double value, double maxVal, int cnt=1) const;
    double expandSocre(const PinExpand& expand, const PinExpand& maxExpand, int cnt) const;
    double expandSocre(PinMetaExpand& maxExpand) const;

private:
    const std::string& pinName_;
    double pinLength_;
    bool connectToUpperLayer_{false};
    bool isBound_{false};
    PinMetaExpand expand_{};
};


} // namespace Rsyn

#endif /* B14973A6_A791_4EC9_B019_CAFDF05FEE9F */
