#ifndef BA50224B_6387_42EC_9F76_D95612E3124F
#define BA50224B_6387_42EC_9F76_D95612E3124F

#include <map>
#include <string>

namespace Rsyn
{

struct PinExpand {
    double acsNMx{0};
    double acsSMx{0};
    double acsEMx{0};
    double acsWMx{0};

    void updateMax(const PinExpand& other) {
        acsNMx = std::max(acsNMx, other.acsNMx);
        acsSMx = std::max(acsSMx, other.acsSMx);
        acsEMx = std::max(acsEMx, other.acsEMx);
        acsWMx = std::max(acsWMx, other.acsWMx);
    }
};

using PinMetaExpand = std::map<std::string, PinExpand>;

} // namespace Rsyn


#endif /* BA50224B_6387_42EC_9F76_D95612E3124F */
