#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <fstream>

#include "gcem.hpp"

// Optimistic
constexpr size_t kMaxStates = 8;
constexpr size_t kMaxSymbols = 7;
constexpr size_t kOof = 10000;
constexpr size_t kBusyBeaverNumbers[kMaxSymbols][kMaxStates] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
	// I'm not actually sure about this one
	{0, 1, kOof, kOof, kOof, kOof, kOof, kOof}, 
	{0, 1, 6, 21, 107, kOof, kOof, kOof},
	{0, 1, 38, kOof, kOof, kOof, kOof, kOof},
	{0, 1, kOof, kOof, kOof, kOof, kOof, kOof},
	{0, 1, kOof, kOof, kOof, kOof, kOof, kOof},
	{0, 1, kOof, kOof, kOof, kOof, kOof, kOof}
};

using symbol_t = uint8_t;
using state_t = uint8_t;
using input_t = uint8_t;
using output_t = uint8_t;

// State count does not include the halt state
// Symbol count does not work currently if it is not a power of 2, due to unused values
// Only the state count currently has unused values and therefore can be handled easily
template<size_t kStateCount, size_t kSymbolCount>
struct TuringMachine {
	static constexpr size_t kHaltState = kStateCount;

	static constexpr size_t kSymbolWidth = std::bit_width(kSymbolCount == 1 ? 1 : kSymbolCount - 1);

	static constexpr size_t kDirectionCount = 2;
	static constexpr size_t kDirectionWidth = std::bit_width(kDirectionCount - 1);
	static constexpr int kDirectionOffsets[kDirectionCount] = { -1, 1 };

	static constexpr size_t kInputs = kStateCount * kSymbolCount;
	static constexpr size_t kOutputs = kSymbolCount * kDirectionCount * (kStateCount + 1);
	static constexpr size_t kTransitionCount = kInputs;
	static constexpr size_t kUniqueMachineCount = gcem::round(gcem::pow(kOutputs, kInputs));

	// The tape is double ended and large enough to ensure that the turing machine doesn't go out of bounds
	// An extra bit of padding is added just to make sure
	static constexpr size_t kTapeSize = (2 * kBusyBeaverNumbers[kSymbolCount][kStateCount]) + 2;

	const std::array<output_t, kTransitionCount> kTransitions;
	symbol_t tape[kTapeSize] = { 0 };
	state_t currentState = 0;
	size_t currentPosition = kTapeSize / 2;

	TuringMachine(std::array<output_t, kTransitionCount> transitions) : kTransitions(transitions) {}

	int run() {
		// Constexpr step count should allow for compile time loop unrolling
		constexpr size_t stepCount = kBusyBeaverNumbers[kSymbolCount][kStateCount];

		// It's entirely possible that I'm optimising prematurely here
		for (int stepIndex = 0; stepIndex < stepCount; stepIndex++) {
			// Get the input symbol on the tape
			symbol_t inputSymbol = tape[currentPosition];
			// State will have unused bits, so it must go at the end for easy enumeration
			// This makes room for the symbol bit
			currentState = currentState << kSymbolWidth;
			// Add the symbol bit
			input_t input = currentState ^ inputSymbol;
			
			// Store the output in the current state, shifting away the other values afterwards
			currentState = kTransitions[input];
			
			// The direction bit is at the start of the output
			constexpr output_t outputDirectionMask = 1;
			// Get the appropriate position offset based on whether the bit is set
			size_t positionOffset = kDirectionOffsets[currentState & outputDirectionMask];
			// Shift away the direction bit
			currentState >>= kDirectionWidth;
			
			// The symbol bit is now at the start of the output
			constexpr output_t outputSymbolMask = 1;
			// Set the tape cell at the current position to be the output symbol
			tape[currentPosition] = currentState & outputSymbolMask;
			// Shift away the symbol bit
			// Only the output state itself is left now and already stored in currentState
			currentState >>= kSymbolWidth;

			// Move the tape head left/right
			currentPosition += positionOffset;

			// This hopefully shouldn't have a performance impact due to branch prediction
			// Return the number of steps the program took to halt
			if (currentState == kHaltState) { return stepIndex + 1; }
		}

		// Return -1 if the program failed to halt
		return -1;
	}
};

template<size_t kStateCount, size_t kSymbolCount>
void testMachines(std::ofstream& resultsFile, int& haltingCount, int& nonHaltingCount) {
	constexpr size_t kTransitionCount = TuringMachine<kStateCount, kSymbolCount>::kTransitionCount;
	constexpr size_t kOutputs = TuringMachine<kStateCount, kSymbolCount>::kOutputs;

	std::array<output_t, kTransitionCount> transitions = { 0 };

	for (size_t machineId = 0; machineId < TuringMachine<kStateCount, kSymbolCount>::kUniqueMachineCount; machineId++) {
		TuringMachine<kStateCount, kSymbolCount> machine(transitions);

		int steps = machine.run();
		if (steps > 0) { haltingCount++; }
		else { nonHaltingCount++; }

		double haltingProbability = (100.0 * haltingCount) / (double(haltingCount) + double(nonHaltingCount));
		resultsFile << kStateCount << "," << kSymbolCount << "," << machineId << "," << steps << "," << haltingProbability << "\n";

		for (size_t input_index = 0; input_index < kTransitionCount; input_index++) {
			if (transitions[input_index] == (kOutputs - 1)) {
				transitions[input_index] = 0;
			}
			else {
				transitions[input_index] += 1;
				break;
			}
		}
	}
}

int main() {
	std::ofstream resultsFile;
	resultsFile.open("machine_results.csv");
	resultsFile << "state_count,symbol_count,machine_id,steps_to_halt,halting_probability\n";

	int haltingCount = 0;
	int nonHaltingCount = 0;

	constexpr size_t kSymbolCount = 2;
	testMachines<1, kSymbolCount>(resultsFile, haltingCount, nonHaltingCount);
	testMachines<2, kSymbolCount>(resultsFile, haltingCount, nonHaltingCount);
	testMachines<3, kSymbolCount>(resultsFile, haltingCount, nonHaltingCount);
	testMachines<4, kSymbolCount>(resultsFile, haltingCount, nonHaltingCount);

	std::cout << "Number of halting and non-halting machines: " << haltingCount << ", " << nonHaltingCount << std::endl;

	double finalHaltingProbability = (100.0 * haltingCount) / (double(haltingCount) + double(nonHaltingCount));
	std::cout << "Halting probability: " << finalHaltingProbability << std::endl;

	resultsFile.close();
	return 0;
}
