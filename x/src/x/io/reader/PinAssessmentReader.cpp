#include "PinAssessmentReader.h"
#include "rsyn/io/parser/lef_def/LEFControlParser.h"
#include "rsyn/util/Stepwatch.h"
#include "rsyn/util/ScopeTimer.h"
#include "x/pac/PinExpand.h"
#include "x/pac/MacroScore.h"
#include "x/pac/PinScore.h"

#include <boost/filesystem.hpp>
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
