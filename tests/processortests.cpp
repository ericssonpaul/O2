#include <cassert>
#include <cstdint>
#include <exception>
#include <string>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <fstream>
#include <unordered_map>

#include "catch.hpp"
#include "json.hpp"

#include "../O2.hpp"

using namespace O2;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string path = "./ProcessorTests/nes6502/v1";

CPU cpu;
uint8_t ram[65536];
bool lastRW;
size_t cycle_count;
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
    cycle_count = 0;
    const auto& cycles = c["cycles"];
    for (const auto& cycle : cycles) {
        cpu.cycle();

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

    size_t count = 0;
    for (const auto & entry : fs::directory_iterator(path)) {
        try {
            auto t = std::ifstream(entry.path());

            if (!t.is_open()) {
                std::cerr << "\n" << "Failed to open file: " << entry.path() << std::endl;

                continue;
            }

            auto j = json::parse(t);

            if (!j.is_array()) {
                std::cerr << "\n" << entry.path() << " - invalid format" << std::endl;
            }

            for (const auto& testcase : j) {
                std::cout << "\33[2K\r" << "n" << count << " - " << testcase["name"] << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (!Eval(testcase)) {
                    std::cerr << "\n" << "Test failed:\n" << testcase << std::endl;
                    return false;
                }
                ++count;
            }
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