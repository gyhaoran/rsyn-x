#include "PinAssessmentReader.h"
#include "rsyn/core/Rsyn.h"
#include "rsyn/io/parser/lef_def/LEFControlParser.h"
#include "rsyn/io/Graphics.h"
#include <Rsyn/PhysicalDesign>
#include "rsyn/util/Stepwatch.h"
#include "rsyn/util/ScopeTimer.h"
#include "rsyn/model/timing/Timer.h"
#include "x/pac/PinExpand.h"
#include "x/pac/MacroScore.h"
#include "x/pac/PinScore.h"

#include <numeric>
#include <fstream>

namespace Rsyn {

namespace {

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

        macroScore.print();
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


double calculateSmallestBoundingBoxArea(const LefPinDscp& pin1, const LefPinDscp& pin2) {
    // Initialize a degenerate bounding box
    DoubleRectangle bbox;
    bbox.degenerate();

    // Helper function to stretch the bounding box to fit a rectangle
    auto stretchToFit = [&bbox](const DoubleRectangle& rect) {
        bbox.stretchToFit(rect[0]);
        bbox.stretchToFit(rect[1]);
    };

    // Iterate over all geometries of both pins and update the bounding box
    for (const auto& port : pin1.clsPorts) {
        for (const auto& geo : port.clsLefPortGeoDscp) {
            for (const auto& rect : geo.clsBounds) {
                stretchToFit(rect);
            }
        }
    }

    for (const auto& port : pin2.clsPorts) {
        for (const auto& geo : port.clsLefPortGeoDscp) {
            for (const auto& rect : geo.clsBounds) {
                stretchToFit(rect);
            }
        }
    }

    // Calculate the width and height of the bounding box
    double width = bbox.computeLength(0); // Dimension 0 is X
    double height = bbox.computeLength(1); // Dimension 1 is Y

    // Return the area of the bounding box
    return width * height;
}

} // namespace

bool PinAssessmentReader::load(const Rsyn::Json& params) {
    std::cout << params << "\n";

	std::string path = params.value("path", "");
    scoreFile_ = params.value("score_file", "pac.json");
    
    double minWidth = params.value<double>("min_width", 0.01);
    double minSpace = params.value<double>("min_space", 0.01);
    setDesignRules(minWidth, minSpace);

    if (path.back() != '/') {
        path += "/";
    }

    if (!params.count("lefFiles")) {
        std::cout << "[ERROR] at least one LEF file must be specified...\n";
        return false;
    }

    if (params["lefFiles"].is_array()) {
        const Rsyn::Json fileList = params["lefFiles"];
        for (const std::string file : fileList) {
            lefFiles.push_back(path + std::string(file));
        }
    } else {
        lefFiles.push_back(path + params.value("lefFiles", ""));
    }

	this->session = session;

	parsingFlow();
	return true;
}

void PinAssessmentReader::parsingFlow() {
	parseLefFiles();
    pinAccessCheck(lefDescriptor, scoreFile_);
}

void PinAssessmentReader::parseLefFiles() {
	Stepwatch watch("Parsing LEF files");
	LEFControlParser lefParser;

	for (int i = 0; i < lefFiles.size(); i++) {
		if (!boost::filesystem::exists(lefFiles[i])) {
			std::cout << "[WARNING] Failed to open file " << lefFiles[i] << "\n";
			std::exit(1);
		}
		lefParser.parseLEF(lefFiles[i], lefDescriptor);
	}
}

} // namespace Rsyn
