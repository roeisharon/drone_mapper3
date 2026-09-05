#pragma once

#include "args/ArgsParser.h"

namespace simulator {

// The two run modes share one flow (scan folder → load fixed plugin → create results folder →
// load varying plugins → parse composition → run the matrix per pairing → write reports); each
// entry point parameterizes which side varies, the folder/file arguments, the output-folder
// prefix, and the report format. Both return the process exit code.

// -comparative: every mission control in the folder × the one algorithm.
int runComparativeMode(const ParsedArgs& args);

// -competition: the one mission control × every algorithm in the folder.
int runCompetitionMode(const ParsedArgs& args);

} // namespace simulator
