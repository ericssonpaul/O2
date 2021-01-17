#include <O2.hpp>
#include <chrono>
#include <string.h>
#include <thread>

namespace O2 {

void CPU::variables_init()
{
	memset(&P, 0, sizeof(P));

	A = 0;
	X = 0;
	Y = 0;
	S = 0xFD;

	P[INTERRUPT] = true;
	P[B0]		 = true;
	P[B1]		 = true;

	cold			   = true;
	state			   = FETCH;
	rnmi			   = false;
	rirq			   = false;
	rbrk			   = false;
	cbrk			   = rbrk;
	intc			   = true;
	iflag			   = RESET;
	hlt				   = false;
	cycle_count		   = 0;
	instruction_cycles = 0;
}
void CPU::setRW(std::function<uint8_t(uint16_t)>	   readByte,
				std::function<void(uint16_t, uint8_t)> writeByte)
{
	CPU::read  = readByte;
	CPU::write = writeByte;
}
CPU::CPU()
{
	variables_init();
}
CPU::CPU(std::function<uint8_t(uint16_t)>		readByte,
		 std::function<void(uint16_t, uint8_t)> writeByte)
{
	variables_init();
	setRW(readByte, writeByte);
}

void CPU::run(unsigned long speed)
{
	unsigned long ns;
	if (speed)
		ns = 1000000000 / speed;

	while (true) {
		auto now = std::chrono::high_resolution_clock::now();
		cycle();
		if (speed)
			std::this_thread::sleep_for(
			  std::chrono::nanoseconds(ns) -
			  (std::chrono::high_resolution_clock::now() - now));
	}
}
void CPU::run(unsigned long long ms, unsigned long speed)
{
	unsigned long ns;
	if (speed)
		ns = 1000000000 / speed;

	auto startTime = std::chrono::high_resolution_clock::now();
	while (true) {
		auto now = std::chrono::high_resolution_clock::now();
		cycle();
		if (std::chrono::high_resolution_clock::now() - startTime >
			std::chrono::milliseconds(ms))
			break;
		if (speed)
			std::this_thread::sleep_for(
			  std::chrono::nanoseconds(ns) -
			  (std::chrono::high_resolution_clock::now() - now));
	}
}
void CPU::raise(enum INTERRUPT_FLAGS m)
{
	switch (m) {
		case NMI: rnmi = true; break;
		case IRQ: rirq = true; break;
		case RESET:
			iflag = RESET;
			intc  = true;
			hlt	  = false;
			break;
		case INONE:
			rnmi = false;
			rirq = false;
			break;
		default: break;
	}
}
void CPU::unraise(enum INTERRUPT_FLAGS m)
{
	switch (m) {
		case NMI: rnmi = false; break;
		case IRQ: rirq = false; break;
		case RESET:
			if (state != FETCH || cycle_count == 0) {
				iflag = INONE;
				intc  = false;
			}
			break;
		default: break;
	}
}

void CPU::cycle()
{
	if (hlt)
		return;
	if (intc) {
	interrupt:
		interrupt();
		if (!intc) {
			instruction_cycles = cycle_count;
			cycle_count		   = 0;
			switch (iflag) {
				case NMI: rnmi = false; break;
				case IRQ: rirq = false; break;
				default: break;
			}
			iflag = INONE;
		} else
			++cycle_count;
		return;
	}
	switch (state) {
		case FETCH:
			if (rnmi) {
				iflag = NMI;
				intc  = true;
				goto interrupt;
			} else if (rirq && !P[INTERRUPT]) {
				iflag = IRQ;
				intc  = true;
				goto interrupt;
			}
			instruction_cycles = 1;
			return fetch();
		case LOAD:
			load();
			if (state != LOAD) {
				instruction_cycles += cycle_count;
				cycle_count = 0;
			} else {
				++cycle_count;
			}
			return;
		case EXECUTE:
			exec();
			if (state != EXECUTE) {
				instruction_cycles += cycle_count + 1;
				cycle_count = 0;
			} else {
				++cycle_count;
			}
			return;
	}
}

#define VECTOR_NMI	 0xFFFA
#define VECTOR_RESET 0xFFFC
#define VECTOR_IRQ	 0xFFFE
#define P_CHAR                                                                 \
	(P[CARRY] | (P[ZERO] << 1) | (P[INTERRUPT] << 2) | (P[DECIMAL] << 3) |     \
	 (P[B0] << 4) | (P[B1] << 5) | (P[OVERFLOW] << 6) | (P[NEGATIVE] << 7))
void CPU::interrupt()
{
	if (iflag == INONE) {
		intc = false;
		return;
	}
	switch (cycle_count) {
		case 0:
			if (rbrk) {
				cbrk = true;
				rbrk = false;
				++cycle_count;
			}
			return;
		case 1: return;
		case 2:
			if (iflag != RESET) {
				push(PC >> 8);
			} else {
				P[INTERRUPT] = true;
				if (!cold)
					S -= 3;
				else
					cold = false;
			}
			return;
		case 3:
			if (iflag != RESET)
				push(PC & 0xFF);
			return;
		case 4:
			if (iflag != RESET) {
				uint8_t p = P_CHAR;
				if (cbrk)
					p |= (1 << 4);
				else
					p &= ~(1 << 4);
				push(p | (1 << 5));
				P[INTERRUPT] = true;
				cbrk		 = false;
			}
			return;
		case 5:
			switch (iflag) {
				case NMI: PC = rd(VECTOR_NMI); break;
				case RESET: PC = rd(VECTOR_RESET); break;
				case IRQ: PC = rd(VECTOR_IRQ); break;
				default: break;
			}
			return;
		case 6:
			switch (iflag) {
				case NMI: PC |= (rd(VECTOR_NMI + 1) << 8); break;
				case RESET: PC |= (rd(VECTOR_RESET + 1) << 8); break;
				case IRQ: PC |= (rd(VECTOR_IRQ + 1) << 8); break;
				default: break;
			}
			intc = false;
			return;
	}
}

void CPU::fetch()
{
	opcode = rd(PC++);

	switch (opcode & 3) {
		case 0: ctrl_parse(); break;
		case 1:
		case 3: aluc_parse(); break;
		case 2: rmwc_parse(); break;
	}

	av	= PC;
	wv0 = PC & 0xFF;
	wv1 = (PC >> 8) & 0xFF;

	if (addm == IMM || addm == IMPLIED) {
		state = EXECUTE;
		if (addm == IMM)
			++PC;
	} else {
		state = LOAD;
	}
}
void CPU::load()
{
	switch (addm) {
		case ZP: return zp();
		case ZPX: return zpx();
		case ABS: return abs();
		case ABSX: return absx();
		case ABSY: return absy();
		case IND: return ind();
		case INDX: return indx();
		case INDY: return indy();
		case IMM:
		case IMPLIED:
			state		= EXECUTE;
			cycle_count = 0;
			return exec();
	}
}

void CPU::exec()
{
	// ALU instructions
	if ((opcode & 3) == 1) {
		switch (opcode / 0x20) {
			case 0: return o_ora();
			case 1: return o_and();
			case 2: return o_eor();
			case 3: return o_adc();
			case 4:
				if (addm != IMM)
					return o_st(A);
				else
					return o_nop();
			case 5: return o_ld(A);
			case 6: return o_cp(A);
			case 7: return o_sbc();
		}
	}
	// Single type instructions
	switch (opcode) {
		/* Flag instructions */
		case 0x18: return o_fl(P[CARRY], false);
		case 0x38: return o_fl(P[CARRY], true);
		case 0x58: return o_fl(P[INTERRUPT], false);
		case 0x78: return o_fl(P[INTERRUPT], true);
		case 0xB8: return o_fl(P[OVERFLOW], false);
		case 0xD8: return o_fl(P[DECIMAL], false);
		case 0xF8: return o_fl(P[DECIMAL], true);
		/* Branch instructions */
		case 0x10: return o_br(P[NEGATIVE], false);
		case 0x30: return o_br(P[NEGATIVE], true);
		case 0x50: return o_br(P[OVERFLOW], false);
		case 0x70: return o_br(P[OVERFLOW], true);
		case 0x90: return o_br(P[CARRY], false);
		case 0xB0: return o_br(P[CARRY], true);
		case 0xD0: return o_br(P[ZERO], false);
		case 0xF0: return o_br(P[ZERO], true);
		/* Stack instructions */
		case 0x9A: return o_tr(S, X);
		case 0xBA: return o_tr(X, S);
		case 0x48: return o_ph(A);
		case 0x68: return o_pl(A);
		case 0x08: return o_php();
		case 0x28: return o_plp();
		/* Register instructions */
		case 0xAA: return o_tr(X, A);
		case 0x8A: return o_tr(A, X);
		case 0xA8: return o_tr(Y, A);
		case 0x98: return o_tr(A, Y);
		case 0xCA: return o_de(X);
		case 0xE8: return o_in(X);
		case 0x88: return o_de(Y);
		case 0xC8: return o_in(Y);
		case 0x2A: return o_rol();
		case 0x6A: return o_ror();
		case 0x0A: return o_asl();
		case 0x4A: return o_lsr();
		/* Jump instructions */
		case 0x4C: return o_jmp();
		case 0x6C: return o_jmp();
		case 0x20: return o_jsr();
		case 0x00: return o_brk();
		case 0x40: return o_rti();
		case 0x60: return o_rts();

		default: break;
	}
	// CTRL instructions
	if ((opcode & 3) == 0) {
		switch (opcode / 0x20) {
			case 1:
				if (addm == ZP || addm == ABS)
					return o_bit();
				else
					return o_nop();
			case 4:
				if (opcode == 0x9C)
					return o_shy();
				else if (addm != IMM)
					return o_st(Y);
				else
					return o_nop();
			case 5: return o_ld(Y);
			case 6:
				if (addm != ZPX && addm != ABSX)
					return o_cp(Y);
				else
					return o_nop();
			case 7:
				if (addm != ZPX && addm != ABSX)
					return o_cp(X);
				else
					return o_nop();

			default: return o_nop();
		}
	}
	// RMW instructions
	if ((opcode & 3) == 2) {
		if (opcode % 0x20 == 0x1A)
			return o_nop();
		if (opcode % 0x20 == 0x12)
			return o_stp();
		if (opcode % 0x20 == 0x02 && opcode / 0x20 < 4)
			return o_stp();

		switch (opcode / 0x20) {
			case 0: return o_asl();
			case 1: return o_rol();
			case 2: return o_lsr();
			case 3: return o_ror();
			case 4:
				if (addm == IMM)
					return o_nop();
				if (addm == ABSY)
					return o_shx();
				return o_st(X);
			case 5: return o_ld(X);
			case 6:
				if (addm == IMM)
					return o_nop();
				return o_dec();
			case 7:
				if (addm == IMM || addm == IMPLIED)
					return o_nop();
				return o_inc();
		}
	}
	// Illegal instructions
	if ((opcode & 3) == 3) {
		switch (opcode / 0x20) {
			case 0:
				if (addm == IMM)
					return o_anc();
				return o_slo();
			case 1:
				if (addm == IMM)
					return o_anc();
				return o_rla();
			case 2:
				if (addm == IMM)
					return o_alr();
				return o_sre();
			case 3:
				if (addm == IMM)
					return o_arr();
				return o_rra();
			case 4:
				if (addm == IMM)
					return o_xaa();
				if (addm == INDY)
					return o_ahx();
				if (addm == ABSY)
					return o_tas();
				return o_sax();
			case 5:
				if (addm == ABSY)
					return o_las();
				return o_lax();
			case 6:
				if (addm == IMM)
					return o_axs();
				return o_dcp();
			case 7:
				if (addm == IMM)
					return o_sbc();
				return o_isc();
		}
	}

	return o_nop();
}

void CPU::ctrl_parse()
{
	switch (opcode % 0x20) {
		case 0x00:
			switch (opcode / 0x20) {
				case 0:
				case 2:
				case 3: addm = IMPLIED; break;
				case 1: addm = ABS; break;
				case 4 ... 7: addm = IMM; break;
			}
			return;
		case 0x04: addm = ZP; return;
		case 0x18:
		case 0x08: addm = IMPLIED; return;
		case 0x0C:
			if (opcode == 0x6C)
				addm = IND;
			else
				addm = ABS;
			return;
		case 0x10: addm = IMM; return;
		case 0x14: addm = ZPX; return;
		case 0x1C: addm = ABSX; return;
	}
}
void CPU::aluc_parse()
{
	switch (opcode % 0x20) {
		case 0x01: addm = INDX; return;
		case 0x05: addm = ZP; return;
		case 0x09: addm = IMM; return;
		case 0x0D: addm = ABS; return;
		case 0x11: addm = INDY; return;
		case 0x15: addm = ZPX; return;
		case 0x19: addm = ABSY; return;
		case 0x1D: addm = ABSX; return;
	}
}
void CPU::rmwc_parse()
{
	switch (opcode % 0x20) {
		case 0x02:
			switch (opcode / 0x20) {
				case 0 ... 3: addm = IMPLIED; break;
				case 4 ... 7: addm = IMM; break;
			}
			return;
		case 0x06: addm = ZP; return;
		case 0x0A:
		case 0x12:
		case 0x1A: addm = IMPLIED; return;
		case 0x0E: addm = ABS; return;
		case 0x16: addm = ZPX; return;
		case 0x1E:
			switch (opcode / 0x20) {
				case 0 ... 3:
				case 6 ... 7: addm = ABSX; break;
				case 4:
				case 5: addm = ABSY; break;
			}
			return;
	}
}

void CPU::zp()
{
	wv0 = rd(av);
	wv1 = 0;
	++PC;
	state = EXECUTE;
}
void CPU::zpx()
{
	if (cycle_count == 0) {
		wv0 = rd(av);
	} else {
		wv0 += X;
		wv1 = 0;
		++PC;
		state = EXECUTE;
	}
}
void CPU::abs()
{
	if (cycle_count == 0) {
		wv0 = rd(av++);
	} else {
		wv1			= rd(av);
		cycle_count = 0;
		PC += 2;
		state = EXECUTE;

		if (opcode == 0x4C)
			exec();
	}
}
void CPU::absx()
{
	if (cycle_count == 0) {
		wv0 = rd(av++);
	} else if (cycle_count == 1) {
		wv1				  = rd(av);
		unsigned char tmp = wv0;
		wv0 += X;
		if (wv0 < tmp) {
			++wv1;
		} else {
			PC += 2;
			state = EXECUTE;
		}
	} else {
		PC += 2;
		state = EXECUTE;
	}
}
void CPU::absy()
{
	if (cycle_count == 0) {
		wv0 = rd(av++);
	} else if (cycle_count == 1) {
		wv1				  = rd(av);
		unsigned char tmp = wv0;
		wv0 += Y;
		if (wv0 < tmp) {
			++wv1;
		} else {
			PC += 2;
			state = EXECUTE;
		}
	} else {
		PC += 2;
		state = EXECUTE;
	}
}
void CPU::ind()
{
	switch (cycle_count) {
		case 0: wv0 = rd(av++); break;
		case 1: wv1 = rd(av); break;
		case 2: av = rd((wv1 << 8) | wv0++); break;
		case 3:
			wv1 = rd((wv1 << 8) | wv0);
			wv0 = av;
			PC += 2;
			state = EXECUTE;

			if (opcode == 0x6C)
				exec();
			break;
	}
}
void CPU::indx()
{
	if (cycle_count == 0) {
		av = rd(av);
	} else {
		av			= (av + X) & 0xFF;
		cycle_count = 0;
		++PC;
		addm = ABS;
	}
}
void CPU::indy()
{
	av = rd(av);
	++PC;
	addm = ABSY;
}

// Opcodes
void CPU::o_adc()
{
	unsigned char p = rd((wv1 << 8) | wv0);

	signed short r = p + A + P[CARRY];
	P[CARRY]	   = r > 0xFF;
	P[OVERFLOW]	   = ~(A ^ p) & (A ^ p) & 0x80;
	A			   = r;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_and()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A &= p;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_asl()
{
	if (addm == IMPLIED) {
		P[CARRY] = (A >> 7) & 1;
		A		 = A << 1;
		flUpdate(A);
		state = FETCH;
	} else {
		if (cycle_count == 0) {
			av	= (wv1 << 8) | wv0;
			wv0 = rd(av);
		} else if (cycle_count == 1) {
			P[CARRY] = (wv0 >> 7) & 1;
			wv0		 = wv0 << 1;
			flUpdate(A);
		} else {
			wr(av, wv0);
			state = FETCH;
		}
	}
}
void CPU::o_bit()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	p				= A & p;

	P[OVERFLOW] = (p >> 6) & 1;
	flUpdate(p);
	state = FETCH;
}
void CPU::o_brk()
{
	++PC;
	intc  = true;
	rbrk  = true;
	iflag = NMI;
	state = FETCH;
	interrupt();
}
void CPU::o_dec()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		--wv0;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_eor()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A				= A ^ p;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_inc()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		++wv0;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_jmp()
{
	PC	  = (wv1 << 8) | wv0;
	state = FETCH;
}
void CPU::o_jsr()
{
	if (cycle_count == 0) {
		push((--PC) >> 8);
	} else if (cycle_count == 1) {
		push(PC & 0xFF);
	} else {
		PC	  = (wv1 << 8) | wv0;
		state = FETCH;
	}
}
void CPU::o_lsr()
{
	if (addm == IMPLIED) {
		P[CARRY] = A & 1;
		A		 = A >> 1;
		flUpdate(A);
		state = FETCH;
	} else {
		if (cycle_count == 0) {
			av	= (wv1 << 8) | wv0;
			wv0 = rd(av);
		} else if (cycle_count == 1) {
			P[CARRY] = wv0 & 1;
			wv0		 = wv0 >> 1;
			flUpdate(wv0);
		} else {
			wr(av, wv0);
			state = FETCH;
		}
	}
}
void CPU::o_nop()
{
	state = FETCH;
}
void CPU::o_ora()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A |= p;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_rol()
{
	if (addm == IMPLIED) {
		bool tmp = P[CARRY];
		P[CARRY] = (A >> 7) & 1;
		A		 = A << 1;
		A |= tmp;
		flUpdate(A);
		state = FETCH;
	} else {
		if (cycle_count == 0) {
			av	= (wv1 << 8) | wv0;
			wv0 = rd(av);
		} else if (cycle_count == 1) {
			bool tmp = P[CARRY];
			P[CARRY] = (wv0 >> 7) & 1;
			wv0		 = wv0 << 1;
			wv0 |= tmp;
			flUpdate(wv0);
		} else {
			wr(av, wv0);
			state = FETCH;
		}
	}
}
void CPU::o_ror()
{
	if (addm == IMPLIED) {
		bool tmp = P[CARRY];
		P[CARRY] = A & 1;
		A		 = A >> 1;
		A |= (tmp << 7);
		flUpdate(A);
		state = FETCH;
	} else {
		if (cycle_count == 0) {
			av	= (wv1 << 8) | wv0;
			wv0 = rd(av);
		} else if (cycle_count == 1) {
			bool tmp = P[CARRY];
			P[CARRY] = wv0 & 1;
			wv0		 = wv0 >> 1;
			wv0 |= (tmp << 7);
			flUpdate(wv0);
		} else {
			wr(av, wv0);
			state = FETCH;
		}
	}
}
void CPU::o_rti()
{
	switch (cycle_count) {
		case 0 ... 1: break;
		case 2:
			o_plp();
			state = EXECUTE;
			break;
		case 3: PC = pop(); break;
		case 4:
			PC	  = (pop() << 8) | PC;
			state = FETCH;
			break;
		default: break;
	}
}
void CPU::o_rts()
{
	switch (cycle_count) {
		case 0 ... 2: break;
		case 3: PC = pop(); break;
		case 4:
			PC = (pop() << 8) | PC;
			++PC;
			state = FETCH;
			break;
		default: break;
	}
}
void CPU::o_sbc()
{
	unsigned char  p = rd((wv1 << 8) | wv0);
	unsigned short r = (0x100 + A) - p;
	if (r < 0x100)
		P[CARRY] = false;
	P[OVERFLOW] = ~(A ^ p) & (A ^ p) & 0x80;
	A			= r;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_php()
{
	uint8_t p = P_CHAR;
	push(p | (3 << 4));
	state = FETCH;
}
void CPU::o_plp()
{
	unsigned char p = pop();

	P[CARRY]	 = p & 1;
	P[ZERO]		 = (p >> 1) & 1;
	P[INTERRUPT] = (p >> 2) & 1;
	P[DECIMAL]	 = (p >> 3) & 1;
	P[B0]		 = (p >> 4) & 1;
	P[B1]		 = (p >> 5) & 1;
	P[OVERFLOW]	 = (p >> 6) & 1;
	P[NEGATIVE]	 = (p >> 7) & 1;
	state		 = FETCH;
}

// Wildcard opcodes
void CPU::o_br(bool& flag, bool v)
{
	if (cycle_count == 0) {
		wv0 = rd((wv1 << 8) | wv0);
		if (flag == v) {
			return;
		} else {
			state = FETCH;
		}
	} else if (cycle_count == 1) {
		unsigned char tmp = (PC >> 8);
		PC += (signed char)wv0;
		if (tmp != (PC >> 8))
			return;
		state = FETCH;
	} else {
		state = FETCH;
	}
}
void CPU::o_fl(bool& reg0, bool value)
{
	reg0  = value;
	state = FETCH;
}
void CPU::o_cp(unsigned char& reg)
{
	unsigned char p = rd((wv1 << 8) | wv0);
	P[CARRY]		= reg >= p;
	P[ZERO]			= reg == p;
	P[NEGATIVE]		= reg >= 0x80;
	state			= FETCH;
}
void CPU::o_de(unsigned char& reg0)
{
	--reg0;
	flUpdate(reg0);
	state = FETCH;
}
void CPU::o_in(unsigned char& reg0)
{
	++reg0;
	flUpdate(reg0);
	state = FETCH;
}
void CPU::o_ld(unsigned char& reg)
{
	unsigned char p = rd((wv1 << 8) | wv0);
	reg				= p;
	flUpdate(reg);
	state = FETCH;
}
void CPU::o_st(unsigned char& reg)
{
	wr((wv1 << 8) | wv0, reg);
	state = FETCH;
}
void CPU::o_tr(unsigned char& reg0, unsigned char reg1)
{
	reg0 = reg1;
	flUpdate(reg0);
	state = FETCH;
}
void CPU::o_ph(unsigned char value)
{
	push(value);
	state = FETCH;
}
void CPU::o_pl(unsigned char& reg0)
{
	reg0  = pop();
	state = FETCH;
}

// Illegal opcodes
void CPU::o_stp()
{
	hlt	  = true;
	state = FETCH;
}
void CPU::o_slo()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		P[CARRY] = (wv0 >> 7) & 1;
		wv0		 = wv0 << 1;
		wv0 |= A;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_anc()
{
	o_and();
	P[CARRY] = P[NEGATIVE];
}
void CPU::o_rla()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		bool tmp = P[CARRY];
		P[CARRY] = (wv0 >> 7) & 1;
		wv0		 = wv0 << 1;
		wv0 |= tmp;
		wv0 &= A;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_sre()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		P[CARRY] = wv0 & 1;
		wv0		 = wv0 >> 1;
		wv0 ^= A;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_alr()
{
	o_and();
	addm = IMPLIED;
	o_lsr();
}
void CPU::o_rra()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		bool tmp = P[CARRY];
		P[CARRY] = wv0 & 1;
		wv0		 = wv0 >> 1;
		wv0 |= (tmp << 7);
	} else {
		wr(av, wv0);
		signed short r = wv0 + A + P[CARRY];
		P[CARRY]	   = r > 0xFF;
		P[OVERFLOW]	   = ~(A ^ wv0) & (A ^ wv0) & 0x80;
		A			   = r;
		flUpdate(A);
		state = FETCH;
	}
}
void CPU::o_arr()
{
	o_and();
	addm = IMPLIED;
	o_ror();
	P[CARRY]	= (A >> 6) & 1;
	P[OVERFLOW] = ((A >> 6) & 1) ^ (((A >> 5) & 1));
}
void CPU::o_shy()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = Y & (wv1 + 1);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_shx()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = X & (wv1 + 1);
	} else {
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_sax()
{
	unsigned char p = A & X;
	o_st(p);
}
void CPU::o_xaa()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A				= X;
	A &= p;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_ahx()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = A & X & (wv1 + 1);
	} else {
		flUpdate(wv0);
		wr(av, wv0);
		state = FETCH;
	}
}
void CPU::o_tas()
{
	if (cycle_count == 0) {
		av = (wv1 << 8) | wv0;
		S  = A & X;
	} else {
		S &= (wv1 + 1);
		flUpdate(S);
		wr(av, S);
		state = FETCH;
	}
}
void CPU::o_lax()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A				= p;
	X				= p;
	flUpdate(p);
	state = FETCH;
}
void CPU::o_las()
{
	unsigned char p = rd((wv1 << 8) | wv0);
	A				= p & S;
	X				= A;
	S				= A;
	flUpdate(A);
	state = FETCH;
}
void CPU::o_dcp()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count == 1) {
		--wv0;
		flUpdate(wv0);
	} else {
		wr(av, wv0);
		P[CARRY]	= A >= wv0;
		P[ZERO]		= A == wv0;
		P[NEGATIVE] = A >= 0x80;
		state		= FETCH;
	}
}
void CPU::o_axs()
{
	X = A & X - rd(av);
	flUpdate(X);
	state = FETCH;
}
void CPU::o_isc()
{
	if (cycle_count == 0) {
		av	= (wv1 << 8) | wv0;
		wv0 = rd(av);
	} else if (cycle_count) {
		++wv0;
	} else {
		wr(av, wv0);
		unsigned short r = (0x100 + A) - wv0;
		if (r < 0x100)
			P[CARRY] = false;
		P[OVERFLOW] = ~(A ^ wv0) & (A ^ wv0) & 0x80;
		A			= r;
		flUpdate(A);
		state = FETCH;
	}
}
}