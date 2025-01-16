#include <cassert>
#include <cstdint>
#include <ctime>
#include <exception>
#include <string>
#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>
#include <set>
#include <chrono>

#ifdef PROCESSORTESTS_CASE
#include "catch.hpp"
#endif

#include "json.hpp"

#include "../O2.hpp"

using namespace O2;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string path = "./ProcessorTests/nes6502_legal/v1";

CPU cpu;
uint8_t ram[65536];
bool lastRW;
size_t cycle_count;
size_t time_count;
size_t time_count_opcode;
uint16_t lastAdr;
uint8_t lastVal;
static uint8_t read(uint16_t adr)
{
    lastRW  = false;
    lastAdr = adr;
    lastVal = ram[adr];
    return lastVal;
}
static void write(uint16_t adr, uint8_t val)
{
    lastRW  = true;
    lastAdr = adr;
    lastVal = val;
    ram[adr] = val;
}
static inline void cycl()
{
    auto start = std::chrono::high_resolution_clock::now();
    cpu.cycle();
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    time_count += duration.count();
    time_count_opcode += duration.count();
}

#define COMPARE_REG(v, t) \
    if (v != t) { goto reg_fail; }
#define P_CHAR                                                                 \
	(cpu.P[CARRY] | (cpu.P[ZERO] << 1) | (cpu.P[INTERRUPT] << 2) | (cpu.P[DECIMAL] << 3) |     \
	 (cpu.P[B0] << 4) | (cpu.P[B1] << 5) | (cpu.P[OVERFLOW] << 6) | (cpu.P[NEGATIVE] << 7))
bool Eval(const nlohmann::basic_json<>& c)
{
    // Init state
    const auto& ini = c["initial"];

    cpu.PC = ini["pc"];
    cpu.S  = ini["s"];
    cpu.A  = ini["a"];
    cpu.X  = ini["x"];
    cpu.Y  = ini["y"];
    
    uint8_t P = ini["p"];
    cpu.P[CARRY]        = P & (1 << CARRY);
    cpu.P[ZERO]         = P & (1 << ZERO);
    cpu.P[INTERRUPT]    = P & (1 << INTERRUPT);
    cpu.P[DECIMAL]      = P & (1 << DECIMAL);
    cpu.P[B0]           = P & (1 << B0);
    cpu.P[B1]           = P & (1 << B1);
    cpu.P[OVERFLOW]     = P & (1 << OVERFLOW);
    cpu.P[NEGATIVE]     = P & (1 << NEGATIVE);

    const auto& r = ini["ram"];
    for (const auto& b : r) {
        ram[b[0]] = b[1];
    }

    // Exec
    const auto& cycles = c["cycles"];
    for (const auto& cycle : cycles) {
        cycl();

        std::string cpu_lastRWstr = cycle[2];
        bool cmp = (cpu_lastRWstr == "write");

        if (lastRW != cmp) {
            std::cerr << "\n" << c["name"] << " - IO rw err: expected " << cycle << " INSTEAD " << (lastRW ? "write\n" : "read\n");
            return false;
        }
        if (lastAdr != cycle[0]) {
            std::cerr << "\n" << c["name"] << " - IO address err: expected " << cycle << " INSTEAD " << lastAdr << "\n";
            return false;
        }
        if (lastVal != cycle[1]) {
            std::cerr << "\n" << c["name"] << " - IO value err: expected " << cycle << " INSTEAD " << lastVal << "\n";
            return false;
        }
        ++cycle_count;
    }
    
    // Result
    const auto& res = c["final"];
    uint8_t flags = P_CHAR;
    const auto& res_r = res["ram"];

    COMPARE_REG(cpu.PC, res["pc"]);
    COMPARE_REG(cpu.S,  res["s"]);
    COMPARE_REG(cpu.A,  res["a"]);
    COMPARE_REG(cpu.X,  res["x"]);
    COMPARE_REG(cpu.Y,  res["y"]);
    COMPARE_REG(flags,  res["p"]);
    
    for (const auto& mem : res_r) {
        if (ram[mem[0]] != mem[1]) {
            std::cerr << "\n" << c["name"] << " - ram result error: \n";
            for (const auto& mem1 : res_r) {
                std::cerr << "\t" << mem1 << " - " << (int)ram[mem[0]] << "\n";
            }
            return false;
        }
    }

    return true;

    reg_fail:
    const json j = {
        {"pc", cpu.PC},
        {"s", cpu.S},
        {"a", cpu.A},
        {"x", cpu.X},
        {"y", cpu.Y},
        {"p", flags}
    };

    std::cerr << "\n" << c["name"] << " - register result error: \n" << j << "\n" << res << "\n";
    return false;
}
bool ProcessorTestsSuiteEval()
{
    std::cout << "[ProcessorTests] Running standalone unit tests..." << std::endl;

    // Preinit the emulator
    cpu = CPU();
    cpu.steps(8);
    memset(ram, 0, sizeof(uint8_t)*65536);
    cpu.setRW(read, write);
    cycle_count = 0;
    time_count = 0;

    size_t count = 1;
    std::set<fs::path> opcodes;
    std::set<std::string> opcodes_done;
    if (fs::exists(path + "/save_state")) {
        try {
            auto save = std::ifstream(path + "/save_state");
            auto j = json::parse(save);
            opcodes_done = j;
            std::cout << "Resuming from " << path + "/save_state" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Could not load save state: " << e.what() << std::endl;
        }
    }
    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.path().extension() == ".json" && opcodes_done.find(entry.path()) == opcodes_done.end())
            opcodes.insert(entry.path());
    }
    for (const auto& entry : opcodes) {
        try {
            auto t = std::ifstream(entry);

            if (!t.is_open()) {
                std::cerr << "\n" << "Failed to open file: " << entry << std::endl;

                continue;
            }

            auto j = json::parse(t);

            if (!j.is_array()) {
                std::cerr << "\n" << entry << " - invalid format" << std::endl;
            }

            time_count_opcode = 0;
            cpu.tsc = 0;
            for (const auto& testcase : j) {
                std::cout << "\33[2K\r" << "n" << count << " - " << testcase["name"] << std::flush;
                if (!Eval(testcase)) {
                    std::cerr << "\n" << "Test failed:\n" << std::setw(4) << testcase << std::endl;

                    json j(opcodes_done);
                    auto save = std::ofstream(path + "/save_state");
                    save << std::setw(4) << j << std::endl;
                    return false;
                }
                ++count;
            }

            double hz = (double)cpu.tsc / ((double)time_count_opcode * 10e-9);
            std::cout << "\nTime spent: " << time_count_opcode << " ns, cycles: " << cpu.tsc << ", Average speed: " << hz << " Hz\n" << std::flush;

            opcodes_done.insert(entry);
        }
        catch (const json::parse_error& e) {
            std::cerr << "\n" << e.what() << std::endl;
            continue;
        }
        catch (const std::exception& e) {
            std::cerr << "\n" << e.what() << std::endl;
            continue;
        }
    }
    std::cout << "\nSuccess!\n";
    double hz = (double)cycle_count / ((double)time_count * 10e-9);
    std::cout << "\nTotal time spent: " << time_count << " ns, cycles: " << cycle_count << ", Average speed: " << hz << " Hz\n" << std::flush;
    return true;
}

#ifdef PROCESSORTESTS_CASE
TEST_CASE("ProcessorTests")
{
    REQUIRE(ProcessorTestsSuiteEval());
}
#else
int main()
{
    return ProcessorTestsSuiteEval() ? 0 : -1;
}
#endif