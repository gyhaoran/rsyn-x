#include "MacroCostReader.h"
#include "rsyn/io/parser/lef_def/LEFControlParser.h"
#include "rsyn/util/Stepwatch.h"
#include "rsyn/util/ScopeTimer.h"
#include "x/pac/MacroCost.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

namespace Rsyn
{

namespace
{

void calcMacroCost(const LefDscp& lef, double minWidth, const std::string& costFile) {
    std::stringstream ss;
    ss << "[\n";
    for (auto& macro : lef.clsLefMacroDscps)
    {
        MacroCost macroCost(macro);
        auto cost = macroCost.calc();
        ss << "  {\"" << macro.clsMacroName << "\": " << cost << "}\n";
    }
    ss << "]\n";

    auto res = ss.str();
    std::cout << res;

    std::ofstream outFile(costFile);
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    outFile << res;
    outFile.close();
}

} // namespace


bool MacroCostReader::load(const Rsyn::Json& params) {
    std::cout << params << "\n";

	std::string path = params.value("path", "");
    costFile_ = params.value("cost_file", "macro_cost.json");
    minWidth_ = params.value<double>("min_width", 0.01);
    
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
            lefFiles_.push_back(path + std::string(file));
        }
    } else {
        lefFiles_.push_back(path + params.value("lefFiles", ""));
    }

	parsingFlow();
	return true;
}

void MacroCostReader::parsingFlow() {
	parseLefFiles();
    calcMacroCost(lefDescriptor_, minWidth_, costFile_);
}

void MacroCostReader::parseLefFiles() {
	Stepwatch watch("Parsing LEF files");
	LEFControlParser lefParser;

	for (int i = 0; i < lefFiles_.size(); i++) {
		if (!boost::filesystem::exists(lefFiles_[i])) {
			std::cout << "[WARNING] Failed to open file " << lefFiles_[i] << "\n";
			std::exit(1);
		}
		lefParser.parseLEF(lefFiles_[i], lefDescriptor_);
	}
}

} // namespace Rsyn
