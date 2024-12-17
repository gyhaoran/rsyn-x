#include "PinAccessCheck.h"

namespace Rsyn
{

bool PinAccessCheck::run(const Rsyn::Json &params) {
    this->session = session;
    this->design = session.getDesign();
    this->module = design.getTopModule();

    const std::string &designName = session.getDesign().getName();
    std::cout << "params: " << params << '\n';
            
    return true;
} // end method
    
} // namespace Rsyn
