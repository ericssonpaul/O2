#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <O2.hpp>
using namespace O2;

TEST_CASE("startup", "[init]")
{
	CPU cpu;

	SECTION("initialization of variables")
	{
		REQUIRE(cpu.A == 0);
		REQUIRE(cpu.X == 0);
		REQUIRE(cpu.Y == 0);
		REQUIRE(cpu.S == 0xFD);

		REQUIRE(cpu.P[CARRY] == false);
		REQUIRE(cpu.P[ZERO] == false);
		REQUIRE(cpu.P[INTERRUPT] == true);
		REQUIRE(cpu.P[DECIMAL] == false);
		REQUIRE(cpu.P[B0] == true);
		REQUIRE(cpu.P[B1] == true);
		REQUIRE(cpu.P[OVERFLOW] == false);
		REQUIRE(cpu.P[NEGATIVE] == false);

		REQUIRE(cpu.state == FETCH);
		REQUIRE(cpu.ishalted() == false);
	}
	SECTION("hard reset")
	{
		cpu.setRW([](uint16_t addr) { return addr == 0xFFFC ? 0x43 : 0x64; },
				  [](uint16_t, uint8_t) {});
		cpu.step();

		REQUIRE(cpu.PC == 0x6443);
		REQUIRE(cpu.S == 0xFD);
	}
	SECTION("soft reset")
	{
		bool write = false;
		cpu.setRW([](uint16_t addr) { return addr == 0xFFFC ? 0x65 : 0xA2; },
				  [&](uint16_t, uint8_t) { write = true; });
		cpu.step();

		cpu.A = 0x34;
		cpu.X = 0x56;
		cpu.Y = 0xFE;

		cpu.P[INTERRUPT] = false;

		cpu.reset();

		REQUIRE(cpu.PC == 0xA265);
		REQUIRE(cpu.S == 0xFA);

		REQUIRE(cpu.A == 0x34);
		REQUIRE(cpu.X == 0x56);
		REQUIRE(cpu.Y == 0xFE);
		REQUIRE(cpu.P[INTERRUPT] == true);

		REQUIRE(write == false);
	}
}
TEST_CASE("control flow", "[exec]")
{
	CPU cpu;
	SECTION("interrupt")
	{
		uint8_t ram[0x200];
		bool	write = false;
		cpu.setRW(
		  [&](uint16_t addr) {
			  switch (addr) {
				  case 0xFFFA: return (uint8_t)0xE5;
				  case 0xFFFB: return (uint8_t)0x23;
				  case 0x23E5: return (uint8_t)0x40;
				  case 0 ... 0x1FF: return ram[addr];

				  default: return (uint8_t)0;
			  }
		  },
		  [&](uint16_t addr, uint8_t v) {
			  if (addr < 0x1FF)
				  ram[addr] = v;
			  write = true;
		  });
		cpu.step();
		cpu.nmi();

		REQUIRE(cpu.PC == 0x23E5);
		cpu.step();
		REQUIRE(cpu.PC == 0x0000);
		REQUIRE(write == true);
	}
	SECTION("read-write")
	{
		uint8_t v = 0x87;
		cpu.setRW(
		  [&](uint16_t addr) {
			  switch (addr) {
				  case 0: return v;
				  case 1: return (uint8_t)0xA5; // LDA $00
				  case 2: return (uint8_t)0x00;
				  case 3: return (uint8_t)0xA9; // LDA #$34
				  case 4: return (uint8_t)0x34;
				  case 5: return (uint8_t)0x85; // STA $00
				  case 6: return (uint8_t)0x00;
				  case 0xFFFC: return (uint8_t)0x01;

				  default: return (uint8_t)0;
			  }
		  },
		  [&](uint16_t addr, uint8_t val) {
			  if (addr == 0)
				  v = val;
		  });
		cpu.step();

		cpu.step();
		REQUIRE(cpu.A == 0x87);
		cpu.step();
		REQUIRE(cpu.A == 0x34);
		cpu.step();
		REQUIRE(v == 0x34);
	}
	SECTION("flags")
	{
		cpu.setRW(
		  [&](uint16_t addr) {
			  switch (addr) {
				  case 0: return (uint8_t)0xA9; // LDA #$00
				  case 1: return (uint8_t)0x00;
				  case 2: return (uint8_t)0xA9; // LDA #$81
				  case 3: return (uint8_t)0x81;

				  default: return (uint8_t)0;
			  }
		  },
		  [&](uint16_t addr, uint8_t val) {});
		cpu.step();

		cpu.step();
		REQUIRE(cpu.P[ZERO] == true);
		REQUIRE(cpu.P[NEGATIVE] == false);
		cpu.step();
		REQUIRE(cpu.P[ZERO] == false);
		REQUIRE(cpu.P[NEGATIVE] == true);
	}
}