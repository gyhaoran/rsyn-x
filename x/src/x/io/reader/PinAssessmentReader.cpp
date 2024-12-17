#include "PinAssessmentReader.h"
#include "rsyn/core/Rsyn.h"
#include "rsyn/io/parser/lef_def/LEFControlParser.h"
#include "rsyn/io/Graphics.h"
#include <Rsyn/PhysicalDesign>
#include "rsyn/util/Stepwatch.h"
#include "rsyn/util/ScopeTimer.h"
#include "rsyn/model/timing/Timer.h"
#include <numeric>

namespace Rsyn {

namespace {

class PinScore {
public:
    explicit PinScore(const std::string& pinName, double pinLength=0, bool connect=false, bool isBound=false)
        : pinName_{pinName}, pinLength_{pinLength}, connectToUpperLayer_{connect}, isBound_{isBound} {}

    double length() const { return pinLength_; }

    double pinLenScore(double total) const {
        double baseScore = 10;
        auto percent = pinLength_ / total;
        return baseScore * percent;
    }

    double ovlpScore() const {
        double baseScore = 40;
        return connectToUpperLayer_ ? baseScore : 0;
    }

    double bdPinScore() const {
        double baseScore = 20;
        return isBound_ ? baseScore : 0;
    }

    double score(double total) const {
        return pinLenScore(total) + ovlpScore() + bdPinScore();
    }

    bool isConnected() const { return connectToUpperLayer_; }

    bool isBdPin() const { return isBound_; }

    const std::string& name() const { return pinName_; }

private:
    const std::string& pinName_;
    double pinLength_;
    bool connectToUpperLayer_{false};
    bool isBound_{false};
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
    std::regex metaRegex("^Meta(\\d+)$"); 

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

    void calc()
    {        
        std::transform(macro_.clsPins.begin(), macro_.clsPins.end(), std::back_inserter(pinScores_), [this](auto& pin) {
            return calcPinScore(this->macro_, pin);
        });

        pinTotalLength_ = std::accumulate(pinScores_.begin(), pinScores_.end(), 0.0, [](double init, auto& pin) { 
            return init + pin.length(); 
        });
    }

    std::string padString(std::string s, int len=12) const {
        if (s.length() < len) {
            s.resize(len, ' ');
        }
        return s;
    }

    void print() const
    {
        double epsilon = 1e-9;
        if (pinTotalLength_ < epsilon) {
            std::cout << "  no pin in macro, total pin calc length: " << pinTotalLength_ << '\n';
            return;
        }
        
        std::cout << "  PinName     Ovlp    PinLen    PlScore     Score       BdPin\n";
        for (auto& pin : pinScores_)
        {
            auto overlap = pin.isConnected() ? "YES" : "NO";
            std::string bound = pin.isBdPin() ? "YES" : "NO";

            std::cout << "  " << padString(pin.name()) << padString(overlap, 8)
                      << padString(std::to_string(pin.length()), 10)
                      << padString(std::to_string(pin.pinLenScore(pinTotalLength_)))
                      << padString(std::to_string(pin.score(pinTotalLength_)))
                      << bound << '\n';
        }
    }

private:
    const LefMacroDscp& macro_;
    double pinTotalLength_{0};
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

void pinAccessCheck(const LefDscp& lef) {
    MEASURE_TIME();
    for (auto& macro : lef.clsLefMacroDscps)
    {
        std::cout << "Macro " << macro.clsMacroName << ": pin num: " << macro.clsPins.size() << '\n';
        MacroScore macroScore(macro);

        macroScore.calc();
        macroScore.print();

        // printMacros(macro);
    }
}

} // namespace

bool PinAssessmentReader::load(const Rsyn::Json& params) {
	std::string path = params.value("path", "");

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

    pinAccessCheck(lefDescriptor);

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

