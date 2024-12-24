#ifndef D6EEFF07_CF6D_457F_925B_925508EA2803
#define D6EEFF07_CF6D_457F_925B_925508EA2803

class LefMacroDscp;

namespace Rsyn
{

class MacroCost
{
public:
    explicit MacroCost(const LefMacroDscp& macro, double minWidt=0.01);

    double calc();

private:
    double pec(); // pin existence cost
    double pac(); // pin area cost
    double prc(); // pin resolution cost

private:
    const LefMacroDscp& macro_;
    double minWidth_;
};

} // namespace Rsyn

#endif /* D6EEFF07_CF6D_457F_925B_925508EA2803 */
