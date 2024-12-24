#ifndef D47F145C_5AF3_4E40_8709_C11E758F35D5
#define D47F145C_5AF3_4E40_8709_C11E758F35D5

#include "PinScore.h"
#include "PinExpand.h"
#include <vector>

class LefMacroDscp;

namespace Rsyn
{

class MacroScore {
public:
    explicit MacroScore(const LefMacroDscp& macro);

    void calc();
    void updateMaxExpand(const PinMetaExpand& metaExpand);
    void printExpandHead();

    void printHead();
    void print();
    std::string toString();

private:
    const LefMacroDscp& macro_;
    double maxPinLength_{0};
    PinMetaExpand maxExpand_{};
    std::vector<PinScore> pinScores_{};
};

void setDesignRules(double minWidth, double minSpace);

} // namespace Rsyn


#endif /* D47F145C_5AF3_4E40_8709_C11E758F35D5 */
