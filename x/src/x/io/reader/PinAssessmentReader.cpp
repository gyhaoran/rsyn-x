#include "PinAssessmentReader.h"
#include "rsyn/core/Rsyn.h"
#include "rsyn/io/parser/lef_def/LEFControlParser.h"
#include "rsyn/io/Graphics.h"
#include <Rsyn/PhysicalDesign>
#include "rsyn/util/Stepwatch.h"
#include "rsyn/util/ScopeTimer.h"
#include "rsyn/model/timing/Timer.h"
#include <numeric>
#include <fstream>

namespace Rsyn {

namespace {

// Predefined constants
const string INVALID_LEF_NAME = "INVALID";
double MIN_WIDTH = 0.018; // Minimum width in micrometers
double MIN_SPACING = 0.018; // Minimum spacing between metals in micrometers
const double EPSILON = 1e-9; // Small value for floating point comparison

struct DesignRule {
    double minimumClearance;
    double viaSize;
    // Other design rules can go here, e.g., minimum metal width, via spacing, etc.
};

// Helper function to determine if a via can fit in the given area
bool canFitVia(const DoubleRectangle &bounds, const DesignRule &designRule) {
    double width = bounds.computeLength(X);
    double height = bounds.computeLength(Y);

    return (width >= designRule.viaSize && height >= designRule.viaSize);
}

// Function to calculate the FitRt metric for each pin
void calculateFitRt(LefMacroDscp &macro, const DesignRule &designRule) {
    for (auto &pin : macro.clsPins) {
        int validViaCount = 0;
        int totalViaCount = 0;

        // For each port of the pin, check which vias can fit
        for (const auto &port : pin.clsPorts) {
            for (const auto &portGeo : port.clsLefPortGeoDscp) {
                for (const auto &bound : portGeo.clsBounds) {
                    if (canFitVia(bound, designRule)) {
                        validViaCount++;
                    }
                    totalViaCount++;
                }
            }
        }

        double fitRatio = (totalViaCount == 0) ? 0.0 : (validViaCount / (double) totalViaCount);
        std::cout << "Pin: " << pin.clsPinName << " Fit Ratio: " << fitRatio << std::endl;
    }
}

// Function to check design rules including minimum spacing from other pins
bool checkDesignRules(const std::string& metaName, const DoubleRectangle& rect, 
                      const std::vector<LefObsDscp>& obstructions, 
                      const std::vector<LefPinDscp>& otherPins,
                      const DoubleRectangle& macroBound) {
    // Check minimum width
    if (rect.computeLength(X) < MIN_WIDTH - EPSILON || rect.computeLength(Y) < MIN_WIDTH - EPSILON) {
        // std::cout << "\033[1;31m" << "Check minimum width failed: X: " << rect.computeLength(X) << ", Y: " << rect.computeLength(Y) << ", rect: " << rect << '\n';
        return false;
    }

    // Check minimum spacing from obstructions and other pins
    auto checkSpacing = [&](const DoubleRectangle& bound) -> bool {
        double dx = std::max(0.0, std::min(std::abs(rect[LOWER][X] - bound[UPPER][X]), std::abs(rect[UPPER][X] - bound[LOWER][X])));
        double dy = std::max(0.0, std::min(std::abs(rect[LOWER][Y] - bound[UPPER][Y]), std::abs(rect[UPPER][Y] - bound[LOWER][Y])));
        return std::max(dx, dy) > MIN_SPACING;
    };

    auto checkSpacing2 = [&](const DoubleRectangle& bound) -> bool {
        double dx = std::max(0.0, std::min(std::abs(rect[LOWER][X] - bound[LOWER][X]), std::abs(rect[UPPER][X] - bound[UPPER][X])));
        double dy = std::max(0.0, std::min(std::abs(rect[LOWER][Y] - bound[LOWER][Y]), std::abs(rect[UPPER][Y] - bound[UPPER][Y])));
        return std::min(dx, dy) > MIN_SPACING;
    };

    if (!checkSpacing2(macroBound)) {
        // std::cout << "\033[1;31m" << "Check macro bound failed\n";
        return false;
    }

    // Check against obstructions
    for (const auto& obs : obstructions) {
        if (obs.clsMetalLayer != metaName) {
            continue;
        }
        for (const auto& bound : obs.clsBounds) {
            if (!checkSpacing(bound)) {
                // std::cout << "\033[1;31m" << "Check obstructions bound failed\n";
                return false;
            }
        }
    }

    // Check against other pins
    for (const auto& pin : otherPins) {
        for (const auto& port : pin.clsPorts) {
            for (const auto& geo : port.clsLefPortGeoDscp) {
                if (geo.clsMetalName != metaName) {
                    continue;
                }
                for (const auto& bound : geo.clsBounds) {
                    if (rect.overlap(bound)) {
                        // std::cout << "\033[1;31m" << "expand pin port is overlap other pin port, port_expand: " << rect 
                        //           << ", Other Pin " << pin.clsPinName << " port" << bound << '\n';
                        return false;
                    }
                    if (!checkSpacing(bound)) {
                        // std::cout << "\033[1;31m" << "Check minimum spacing failed, port_expand: " << rect 
                        //           << ", Other Pin " << pin.clsPinName << " port" << bound << '\n';
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

// Function to calculate maximum expansion in one direction
double calculateMaxExpansion(const std::string& metaName, const DoubleRectangle& pinRect, 
                             const std::vector<LefObsDscp>& obstructions,
                             const std::vector<LefPinDscp>& otherPins,
                             const DoubleRectangle& macroBound,
                             const int dimension,
                             const bool positiveDirection) {
    double maxExpansion = 0.0;
    double increment = 0.01; // Larger step size for faster execution

    while (true) {
        DoubleRectangle testRect(pinRect);
        double newCoord = positiveDirection ? pinRect[UPPER][dimension] + maxExpansion :
                                              pinRect[LOWER][dimension] - maxExpansion;

        // Update the points of the rectangle based on the dimension being expanded
        if (dimension == X) {
            testRect.updatePoints(
                positiveDirection ? pinRect[LOWER] : double2(newCoord, pinRect[LOWER][Y]),
                positiveDirection ? double2(newCoord, pinRect[UPPER][Y]) : pinRect[UPPER]
            );
        } else if (dimension == Y) {
            testRect.updatePoints(
                positiveDirection ? pinRect[LOWER] : double2(pinRect[LOWER][X], newCoord),
                positiveDirection ? double2(pinRect[UPPER][X], newCoord) : pinRect[UPPER]
            );
        }

        // std::cout << "\033[0m" << "pinRect: " << pinRect << ", dimension: " << dimension<< ", positiveDirection: " << positiveDirection 
        //     << ", maxExpansion: " << maxExpansion << ", testRect: " << testRect << '\n';

        maxExpansion += increment;

        if (!checkDesignRules(metaName, testRect, obstructions, otherPins, macroBound)) {
            break;
        }

        if (newCoord < macroBound[LOWER][Y] || newCoord < macroBound[LOWER][X] 
            || newCoord > std::max(macroBound[UPPER][Y], macroBound[UPPER][X]))
        {
            std::cout << "\033[1;31m" << "error: over the macro bound " << macroBound << " in expand, will exist.\033[0m\n";
            std::exit(1);
            return 0;
        }
    }

    return maxExpansion - increment; // Subtract the last increment that caused failure
}

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

// Function to calculate expansion capabilities in four directions
using PinMetaExpand = std::map<string, PinExpand>;

PinMetaExpand calcExpansion(const LefPinDscp& pin, 
                            const std::vector<LefObsDscp>& obstructions, 
                            const std::vector<LefPinDscp>& otherPins,
                            const DoubleRectangle& macroBound) {

    PinMetaExpand metaExpand;

    for (const auto& port : pin.clsPorts) {
        for (const auto& geo : port.clsLefPortGeoDscp) {
            auto& metaName = geo.clsMetalName;
            if (metaName.substr(0, 1) != "M") {
                continue;
            }
            for (const auto& rect : geo.clsBounds) {

                double acsNMx = calculateMaxExpansion(metaName, rect, obstructions, otherPins, macroBound, Y, true); // North
                double acsSMx = calculateMaxExpansion(metaName, rect, obstructions, otherPins, macroBound, Y, false); // South
                double acsEMx = calculateMaxExpansion(metaName, rect, obstructions, otherPins, macroBound, X, true); // East
                double acsWMx = calculateMaxExpansion(metaName, rect, obstructions, otherPins, macroBound, X, false); // West

                if (acsNMx < 0 || acsSMx < 0 || acsEMx < 0 || acsWMx < 0) {
                    std::cout << "error: acsNMx: " << acsNMx << ", acsSMx: " << acsSMx << ", acsEMx: " << acsEMx << ", acsWMx: " << acsWMx << '\n';
                }

                auto& pinExpand = metaExpand[metaName];
                pinExpand.updateMax(PinExpand{acsNMx, acsSMx, acsEMx, acsWMx});

                // std::cout << "    Layer " << metaName << ", acsNMx: " << acsNMx << ", acsSMx: " << acsSMx << ", acsEMx: " << acsEMx << ", acsWMx: " << acsWMx << '\n';
            }
        }
    }    

    return metaExpand;
}

std::string padString(std::string s, int len=12) {
    if (s.length() < len) {
        s.resize(len, ' ');
    }
    return s;
}

class PinScore {
public:
    explicit PinScore(const std::string& pinName, double pinLength=0, bool connect=false, bool isBound=false)
        : pinName_{pinName}, pinLength_{pinLength}, connectToUpperLayer_{connect}, isBound_{isBound} {}

    double length() const { return pinLength_; }

    double pinLenScore(double maxPnLen) const {
        double baseScore = 10;
        auto percent = pinLength_ / maxPnLen;
        return baseScore * percent;
    }

    double ovlpScore() const {
        double baseScore = 40;
        return connectToUpperLayer_ ? 0 : baseScore;
    }

    double bdPinScore() const {
        double baseScore = 20;
        return isBound_ ? 0 : baseScore;
    }

    double dirScore(double value, double maxVal, int cnt=1) const {
        double baseScore = 5.0;
        if (maxVal == 0) {
            return baseScore / cnt; // Avoid division by zero
        }
        return (baseScore * value) / (maxVal * cnt);
    }

    double expandSocre(const PinExpand& expand, const PinExpand& maxExpand, int cnt) const {
        return dirScore(expand.acsEMx, maxExpand.acsEMx, cnt) + dirScore(expand.acsNMx, maxExpand.acsNMx, cnt)
             + dirScore(expand.acsSMx, maxExpand.acsSMx, cnt) + dirScore(expand.acsWMx, maxExpand.acsWMx, cnt);
    }

    double expandSocre(PinMetaExpand& maxExpand) const {
        double score = 0;
        for (auto it : expand_) {
            string meta = it.first;
            PinExpand expand = it.second;
            score += expandSocre(expand, maxExpand[meta], maxExpand.size());
        }
        return score;
    }

    double score(double maxPnLen, PinMetaExpand& maxExpand) const {
        return pinLenScore(maxPnLen) + ovlpScore() + bdPinScore() + expandSocre(maxExpand);
    }

    bool isConnected() const { return connectToUpperLayer_; }

    bool isBdPin() const { return isBound_; }

    const std::string& name() const { return pinName_; }

    void setExpandLen(const PinMetaExpand& expand) { expand_ = expand; }

    void printExpand(PinMetaExpand& maxExpand) {
        for (auto it : expand_) {
            string meta = it.first;
            PinExpand expand = it.second;
            std::cout << padString(std::to_string(dirScore(expand.acsNMx, maxExpand[meta].acsNMx)))
                      << padString(std::to_string(dirScore(expand.acsSMx, maxExpand[meta].acsSMx)))
                      << padString(std::to_string(dirScore(expand.acsWMx, maxExpand[meta].acsWMx)))
                      << padString(std::to_string(dirScore(expand.acsEMx, maxExpand[meta].acsEMx)));
        }
    }

private:
    const std::string& pinName_;
    double pinLength_;
    bool connectToUpperLayer_{false};
    bool isBound_{false};
    PinMetaExpand expand_{};
};

bool isBoundPin(const LefMacroDscp& macro, const DoubleRectangle& bound)
{
    double macroWidth = macro.clsSize.x;
    double macroHeight = macro.clsSize.y;

    double pinLeft = bound[LOWER].x;
    double pinRight = bound[UPPER].x;
    double pinBottom = bound[LOWER].y;
    double pinTop = bound[UPPER].y;

    double tolerance = 0.05; 

    bool isNearLeft = pinLeft < tolerance * macroWidth;
    bool isNearRight = pinRight > macroWidth - tolerance * macroWidth;
    bool isNearBottom = pinBottom < tolerance * macroHeight;
    bool isNearTop = pinTop > macroHeight - tolerance * macroHeight;

    return isNearLeft || isNearRight || isNearBottom || isNearTop;
}

int getMetalLayerIndex(const std::string& metalName) {
    std::regex mRegex("^M(\\d+)$");
    std::regex metaRegex("^Metal(\\d+)$"); 

    std::smatch match;
    if (std::regex_match(metalName, match, mRegex)) {
        int layerIndex = std::stoi(match[1]);
        return layerIndex;
    } else if (std::regex_match(metalName, match, metaRegex)) {
        int layerIndex = std::stoi(match[1]);
        return layerIndex;
    }
    return 0;
}

int findMaxMetalLayerIndex(const LefPortDscp& port) {
    int maxMetalLayer = 0;
    int index = 0;

    for (int i =0; i < port.clsLefPortGeoDscp.size(); ++i) {
        auto& geometry = port.clsLefPortGeoDscp[i];

        int layerIndex = getMetalLayerIndex(geometry.clsMetalName);
        if (layerIndex > maxMetalLayer) {
            maxMetalLayer = layerIndex;
            index = i;
        }
    }

    return index;
}

PinScore calcPinScore(const LefMacroDscp& macro, const LefPinDscp& pin) {
    double length = 0.0;
    bool connectToUpperLayer = false;
    bool isBound = false;
    for (auto& port : pin.clsPorts)
    {
        if (port.clsLefPortGeoDscp.size() > 1)
        {
            connectToUpperLayer = true;
        }

        auto index = findMaxMetalLayerIndex(port);
        for (auto& bound : port.clsLefPortGeoDscp[index].clsBounds)
        {
            length += bound.computeDiagonal();

            if (!isBound)
            {
                isBound = isBoundPin(macro, bound);
            }
        }
    }

    return PinScore(pin.clsPinName, length, connectToUpperLayer, isBound);
}

class MacroScore {
public:
    explicit MacroScore(const LefMacroDscp& macro) : macro_{macro} {}

    void calc() {
        for (size_t i = 0; i < macro_.clsPins.size(); ++i) {
            const auto& pin = macro_.clsPins[i];
            
            if (pin.clsPinName == "VDD" || pin.clsPinName == "VSS") {
                continue;     
            }

            std::vector<LefPinDscp> otherPins;
            otherPins.reserve(macro_.clsPins.size() - 1);
            for (size_t j = 0; j < macro_.clsPins.size(); ++j) {
                if (i != j) {
                    otherPins.emplace_back(macro_.clsPins[j]);
                }
            }

            DoubleRectangle macroBound(macro_.clsOrigin, macro_.clsSize);
            auto expand = calcExpansion(pin, macro_.clsObs, otherPins, macroBound);
            updateMaxExpand(expand);

            auto score =  calcPinScore(this->macro_, pin);
            maxPinLength_ = std::max(maxPinLength_, score.length());
            score.setExpandLen(expand);
            
            pinScores_.emplace_back(score);
        }
    }

    void updateMaxExpand(const PinMetaExpand& metaExpand) {
        for (auto it : metaExpand) {
            auto meta = it.first;
            auto expand = it.second;
            maxExpand_[meta].updateMax(expand);
        }
    }

    void printExpandHead() {
        for (auto it : maxExpand_) {
            auto meta = it.first;
            auto layer = std::to_string(getMetalLayerIndex(meta));
            std::cout << "     AcsNM" << layer << "      AcsSM" << layer << "      AcsEM" << layer << "      AcsWM" << layer << " ";
        }
    }

    void printHead() {
        std::cout << "Macro " << macro_.clsMacroName << ": pin num: " << macro_.clsPins.size() << ", clsSize" << macro_.clsSize << '\n';
        std::cout << "  PinName     Ovlp    PinLen    PlScore     Score       BdPin";
        printExpandHead();
        std::cout << '\n';
    }

    void print()
    {
        if (maxPinLength_ <= EPSILON) {
            std::cout << "  no pin in macro, total pin calc length: " << maxPinLength_ << '\n';
            return;
        }
        
        printHead();

        for (auto& pin : pinScores_)
        {
            auto overlap = pin.isConnected() ? "YES" : "NO";
            std::string bound = pin.isBdPin() ? "YES" : "NO";

            std::cout << "  " << padString(pin.name());
            std::cout << padString(overlap, 8)
                      << padString(std::to_string(pin.length()), 10)
                      << padString(std::to_string(pin.pinLenScore(maxPinLength_)))
                      << padString(std::to_string(pin.score(maxPinLength_, maxExpand_)))
                      << padString(bound, 10);
            pin.printExpand(maxExpand_);
            std::cout << '\n';
        }
    }

    std::string toString() {
        std::stringstream ss;
        ss << "  {\"" << macro_.clsMacroName << "\": [";

        int end = pinScores_.size() - 1;
        for (int i = 0; i < end; ++i) {
            auto& pin = pinScores_[i];
            ss << "{\"" << pin.name() << "\": " << pin.score(maxPinLength_, maxExpand_) << "}, ";
        }
        if (end >= 0) {
            auto& pin = pinScores_[end];
            ss << "{\"" << pin.name() << "\": " << pin.score(maxPinLength_, maxExpand_) << "}";
        }

        ss << "]}";

        return ss.str();
    }

private:
    const LefMacroDscp& macro_;
    double maxPinLength_{0};
    PinMetaExpand maxExpand_{};
    std::vector<PinScore> pinScores_{};
};

void printMacros(const LefMacroDscp& macro)
{
    std::cout << "Macro " << macro.clsMacroName << ": pin num: " << macro.clsPins.size() << '\n';
    for (auto& pin : macro.clsPins)
    {
        printf("  Pin %s, dir %s, use %s, port num %lu\n", pin.clsPinName.c_str(), 
                pin.clsPinDirection.c_str(), pin.clsPinUse.c_str(), pin.clsPorts.size());

        for (auto& port : pin.clsPorts)
        {
            for (auto& port_geo : port.clsLefPortGeoDscp)
            {
                printf("    Layer %s\n", port_geo.clsMetalName.c_str());
                for (auto& bound : port_geo.clsBounds)
                {
                    std::cout << "      " << bound << '\n';
                }
            }
        }
    }
}

void pinAccessCheck(const LefDscp& lef, const std::string& socre_file) {
    MEASURE_TIME();
    std::string res = "[\n";
    for (auto& macro : lef.clsLefMacroDscps)
    {
        MacroScore macroScore(macro);
        macroScore.calc();
        res += macroScore.toString() + ", \n";

        // macroScore.print();
        // printMacros(macro);
    }
    if (!lef.clsLefMacroDscps.empty())
    {
        res = res.substr(0, res.size() - 3);
    }

    res += "\n]\n";
    std::cout << res;

    std::ofstream outFile(socre_file);
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    outFile << res;
    outFile.close();
}

} // namespace

bool PinAssessmentReader::load(const Rsyn::Json& params) {
    std::cout << params << "\n";

	std::string path = params.value("path", "");
    score_file_ = params.value("score_file", "pac.json");
    MIN_WIDTH = params.value("min_width", 0.01);
    MIN_SPACING = params.value("min_space", 0.01);

    if (path.back() != '/') {
        path += "/";
    }

    if (!params.count("lefFiles")) {
        std::cout << "[ERROR] at least one LEF file must be specified...\n";
        return false;
    } // end if

    if (params["lefFiles"].is_array()) {
        const Rsyn::Json fileList = params["lefFiles"];
        for (const std::string file : fileList) {
            lefFiles.push_back(path + std::string(file));
        } // end for
    } else {
        lefFiles.push_back(path + params.value("lefFiles", ""));
    } // end if

	this->session = session;

	parsingFlow();
	return true;
} // end method

// -----------------------------------------------------------------------------

void PinAssessmentReader::parsingFlow() {
	parseLEFFiles();
    // populateDesign();

    pinAccessCheck(lefDescriptor, score_file_);

    // Stepwatch watch("Run pac process");
    // session.runProcess("pin.accesscheck");
} // end method 

// -----------------------------------------------------------------------------

void PinAssessmentReader::parseLEFFiles() {
	Stepwatch watch("Parsing LEF files");
	LEFControlParser lefParser;

	for (int i = 0; i < lefFiles.size(); i++) {
		if (!boost::filesystem::exists(lefFiles[i])) {
			std::cout << "[WARNING] Failed to open file " << lefFiles[i] << "\n";
			std::exit(1);
		} // end if 

		lefParser.parseLEF(lefFiles[i], lefDescriptor);
	} // end for
} // end method 

void PinAssessmentReader::populateDesign() {
	Stepwatch watch("Populating the design");

	Rsyn::Design design = session.getDesign();

    Stepwatch watchRsyn("Loading design into Rsyn");
	Reader::populateRsynLibraryFromLef(lefDescriptor, design);
	watchRsyn.finish();

	Rsyn::Json physicalDesignConfiguration;
	physicalDesignConfiguration["clsEnableMergeRectangles"] = false;
	physicalDesignConfiguration["clsEnableNetPinBoundaries"] = true;
	physicalDesignConfiguration["clsEnableRowSegments"] = true;

	session.startService("rsyn.physical", physicalDesignConfiguration);
	Rsyn::PhysicalDesign physicalDesign = session.getPhysicalDesign();
	physicalDesign.loadLibrary(lefDescriptor);

	physicalDesign.updateAllNetBounds(false);
	watch.finish();
} // end method
    
} // namespace Rsyn

