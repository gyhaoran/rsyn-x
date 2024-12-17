#ifndef B830C38E_DCA8_45BC_9DBB_93381CAC34B6
#define B830C38E_DCA8_45BC_9DBB_93381CAC34B6

#include <Rsyn/Session>

namespace Rsyn
{

class PinAccessCheck : public Rsyn::Process {
private:
	Rsyn::Session session;
	Rsyn::Design design;
	Rsyn::Module module;
    
private:
	virtual bool run(const Rsyn::Json &params);
	
}; // end class

} // namespace Rsyn


#endif /* B830C38E_DCA8_45BC_9DBB_93381CAC34B6 */
