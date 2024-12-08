#ifndef O2_HPP
#define O2_HPP

#ifdef __cplusplus

#include <cstdint>
#include <functional>

namespace O2 {

enum INTERRUPT_FLAGS
{
	NMI,
	RESET,
	IRQ,
	INONE,
};
enum STATUS_FLAGS
{
	CARRY,
	ZERO,
	INTERRUPT,
	DECIMAL,
	B0,
	B1,
	OVERFLOW,
	NEGATIVE,
};
enum INSTRUCTION_STATE
{
	FETCH,
	LOAD,
	EXECUTE,
};
enum ADDRESSING_MODE
{
	IMPLIED,
	IMM,
	ZP,
	ZPX,
	ZPY,
	ABS,
	ABSX,
	ABSY,
	IND,
	INDX,
	INDY,
};
const unsigned long CPU_SPEED_2A03		   = 1789773; /* ~1.79 MHz */
const unsigned long CPU_SPEED_2A07		   = 1662607; /* ~1.67 MHz */
const unsigned long CPU_DEFAULT_SPEED	   = CPU_SPEED_2A03;
const uint16_t 		CPU_VECTOR_NMI		   = 0xFFFA;
const uint16_t		CPU_VECTOR_RESET	   = 0xFFFC;
const uint16_t		CPU_VECTOR_IRQ		   = 0xFFFE;
const uint8_t		CPU_DEFAULT_READ_VALUE = 0xEA; /* NOP */

class CPU
{
  public:
	uint16_t PC;
	uint8_t	 A, X, Y, S;
	bool	 P[8];

	unsigned int instruction_cycles;
	size_t tsc;

	INSTRUCTION_STATE state;

  protected:
	std::function<uint8_t(uint16_t)>	   read;
	std::function<void(uint16_t, uint8_t)> write;

	bool			cold;
	ADDRESSING_MODE addm;
	INTERRUPT_FLAGS iflag;
	bool			rnmi, rirq, rbrk, cbrk;
	bool			intc;
	bool			hlt;
	unsigned char	cycle_count;
	uint8_t			opcode, wv0, wv1, page;
	uint16_t		av;

  public:
	CPU();
	CPU(std::function<uint8_t(uint16_t)>	   readByte,
		std::function<void(uint16_t, uint8_t)> writeByte);

	void setRW(std::function<uint8_t(uint16_t)>		  readByte,
			   std::function<void(uint16_t, uint8_t)> writeByte);

	void		cycle();
	inline void cycles(unsigned int c)
	{
		while (c--) {
			cycle();
		}
	}
	inline void step()
	{
		do {
			cycle();
		} while (state != FETCH || intc == true);
	}
	inline void steps(unsigned int c)
	{
		while (c--) {
			step();
		}
	}

	void run(unsigned long speed = CPU_DEFAULT_SPEED);
	void run(unsigned long long ms, unsigned long speed = CPU_DEFAULT_SPEED);

	void raise(INTERRUPT_FLAGS);
	void unraise(INTERRUPT_FLAGS);

	inline void nmi()
	{
		raise(NMI);
		step();
	}
	inline void reset()
	{
		raise(RESET);
		step();
	}
	inline void irq()
	{
		raise(IRQ);
		step();
	}

	inline void halt() { hlt = true; tsc = 0; };
	void		unhalt() { hlt = false; };
	bool		ishalted() { return hlt; };

  private:
	void state_init();

	void fetch();
	void load();
	void exec();
	void interrupt();

	void ctrl_parse();
	void aluc_parse();
	void rmwc_parse();

	/* Addressing mode functions */
	void zp();
	void zpx();
	void zpy();
	void abs();
	void absx();
	void absy();
	void ind();
	void indx();
	void indy();

	/* Instructions */
	void o_adc();
	void o_and();
	void o_asl();
	void o_bit();
	void o_brk();
	void o_dec();
	void o_eor();
	void o_inc();
	void o_jmp();
	void o_jsr();
	void o_lsr();
	void o_nop();
	void o_ora();
	void o_rol();
	void o_ror();
	void o_rti();
	void o_rts();
	void o_sbc();
	void o_php();
	void o_plp();
	void o_txs();

	void o_br(bool&, bool);
	void o_fl(bool&, bool);
	void o_cp(uint8_t&);
	void o_de(uint8_t&);
	void o_in(uint8_t&);
	void o_ld(uint8_t&);
	void o_st(uint8_t&);
	void o_tr(uint8_t&, uint8_t);
	void o_ph(uint8_t);
	void o_pl(uint8_t&);

	// Illegal opcodes
	void o_stp(); // HLT
	void o_slo();
	void o_anc();
	void o_rla();
	void o_sre(); // LSE
	void o_alr();
	void o_rra();
	void o_arr();
	void o_shy(); // SAY
	void o_shx(); // XAS
	void o_sax(); // AXS
	void o_xaa();
	void o_ahx(); // AXA
	void o_tas();
	void o_lax();
	void o_las();
	void o_dcp(); // DCM
	void o_axs();
	void o_isc(); // INS

	inline uint8_t rd(uint16_t adr)
	{
		if (read)
			return read(adr);
		return CPU_DEFAULT_READ_VALUE;
	}
	inline void wr(uint16_t adr, uint8_t value)
	{
		if (write)
			write(adr, value);
	}
	inline void	   push(uint8_t val) { wr(0x100 + S--, val); }
	inline uint8_t pop() { return rd(0x100 + (++S)); }
	inline void	   flUpdate(uint8_t v)
	{
		P[NEGATIVE] = v & 0x80;
		P[ZERO]		= v == 0;
	}
	inline bool    pageCross(uint8_t& v1, uint8_t& v2) {
		page = v1;
		v1  += v2;
		if (v1 < page) {
			page = 1;
			return true;
		}
		page = 0;
		return false;
	}
};
}
#else
#error O2 is a C++11 library, please use a C++ compiler instead
#endif
#endif