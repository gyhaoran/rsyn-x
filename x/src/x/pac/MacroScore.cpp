#include "MacroScore.h"
#include "rsyn/phy/util/LefDescriptors.h"
#include "x/util/StringUtil.h"
#include <regex>
#include <iostream>

namespace Rsyn
{

namespace {

double MIN_WIDTH = 0.018; // Minimum width in micrometers
double MIN_SPACING = 0.018; // Minimum spacing between metals in micrometers
const double EPSILON = 1e-9; // Small value for floating point comparison

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



} // namespace

MacroScore::MacroScore(const LefMacroDscp& macro) : macro_{macro} {}

void MacroScore::calc() {
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

void MacroScore::updateMaxExpand(const PinMetaExpand& metaExpand) {
    for (auto it : metaExpand) {
        auto meta = it.first;
        auto expand = it.second;
        maxExpand_[meta].updateMax(expand);
    }
}

void MacroScore::printExpandHead() {
    for (auto it : maxExpand_) {
        auto meta = it.first;
        auto layer = std::to_string(getMetalLayerIndex(meta));
        std::cout << "     AcsNM" << layer << "      AcsSM" << layer << "      AcsEM" << layer << "      AcsWM" << layer << " ";
    }
}

void MacroScore::printHead() {
    std::cout << "Macro " << macro_.clsMacroName << ": pin num: " << macro_.clsPins.size() << ", clsSize" << macro_.clsSize << '\n';
    std::cout << "  PinName     Ovlp    PinLen    PlScore     Score       BdPin";
    printExpandHead();
    std::cout << '\n';
}

void MacroScore::print()
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

std::string MacroScore::toString() {
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

void setDesignRules(double minWidth, double minSpace) {
    MIN_WIDTH = minWidth;
    MIN_SPACING = minSpace;
}

} // namespace Rsyn
