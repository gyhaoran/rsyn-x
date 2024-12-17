#ifndef D3C9AC1E_0A01_4D47_8ECE_312D25CB5419
#define D3C9AC1E_0A01_4D47_8ECE_312D25CB5419

#include <Rsyn/Session>
#include "rsyn/model/timing/types.h"

namespace Rsyn {

class PinAssessmentReader : public Reader {
    Session session;	
    std::vector<std::string> lefFiles;

    Number localWireCapacitancePerMicron;
    Number localWireResistancePerMicron;
    DBU maxWireSegmentInMicron;
	
public:
    PinAssessmentReader() = default;

    virtual bool load(const Rsyn::Json& params) override;

private:
    LefDscp lefDescriptor;

    void parsingFlow();
    void populateDesign();
    void parseLEFFiles();
}; // end class

} // namespace Rsyn

#endif /* D3C9AC1E_0A01_4D47_8ECE_312D25CB5419 */
