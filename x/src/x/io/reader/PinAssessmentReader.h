#ifndef D3C9AC1E_0A01_4D47_8ECE_312D25CB5419
#define D3C9AC1E_0A01_4D47_8ECE_312D25CB5419

#include "rsyn/phy/util/LefDescriptors.h"
#include "rsyn/model/timing/types.h"
#include "rsyn/util/Json.h"
#include "rsyn/session/Reader.h"

namespace Rsyn {

class PinAssessmentReader : public Reader {
public:
    PinAssessmentReader() = default;
    virtual bool load(const Rsyn::Json& params) override;

private:
    void parsingFlow();
    void parseLefFiles();
    
private:
    std::vector<std::string> lefFiles;	
    LefDscp lefDescriptor;
    std::string scoreFile_{""};
    bool needExpand_{true};
}; // end class

} // namespace Rsyn

#endif /* D3C9AC1E_0A01_4D47_8ECE_312D25CB5419 */
