/* ym2151.cpp -- the A2VERA's YM2151 on ymfm (plan E14).  See ym2151.h. */
#include "ym2151.h"
#include "extern/ymfm/ymfm_opm.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>

extern "C" unsigned int ticks;             /* puce6502.c: the CPU cycle counter */

#define YM_HZ   3579545.0
#define CPU_HZ  1020484.0                  /* a2core.c A2_CPU_HZ */
#define YM_RATE (YM_HZ / 64.0)             /* one OPM sample per 64 clocks */
#define RING    16384                      /* samples, ~0.3 s */
#define LAG     4                          /* samples kept ahead of the reader */

int           g_ym_enabled = 1;
unsigned long g_ym_writes, g_ym_busy_writes;

namespace {

/* YM clocks -> CPU cycles */
inline unsigned cyc(double ymclocks) { return (unsigned)(ymclocks * CPU_HZ / YM_HZ + 0.5); }

class opm : public ymfm::ymfm_interface {
public:
	opm() : m_chip(*this) {}

	void reset() {
		m_chip.reset();
		m_busy_until = ticks;
		m_timer_on[0] = m_timer_on[1] = false;
	}
	virtual void ymfm_set_timer(uint32_t tnum, int32_t clocks) override {
		if (tnum >= 2) return;
		m_timer_on[tnum] = clocks >= 0;
		if (clocks >= 0) m_timer_at[tnum] = ticks + cyc(clocks);
	}
	virtual void ymfm_set_busy_end(uint32_t clocks) override { m_busy_until = ticks + cyc(clocks); }
	virtual bool ymfm_is_busy() override { return (int)(ticks - m_busy_until) < 0; }

	/* fire the timers that ran out by now (expiry reloads them through set_timer) */
	void sync() {
		for (int i = 0; i < 2; i++)
			while (m_timer_on[i] && (int)(ticks - m_timer_at[i]) >= 0) {
				unsigned late = ticks - m_timer_at[i];
				m_engine->engine_timer_expired(i);
				if (m_timer_on[i]) m_timer_at[i] -= late;   /* keep the period exact */
			}
	}
	void write(uint8_t a, uint8_t v) { m_chip.write_address(a); m_chip.write_data(v); }
	uint8_t status() { return m_chip.read_status(); }
	void generate(int16_t &l, int16_t &r) {
		ymfm::ym2151::output_data o;
		m_chip.generate(&o);
		l = (int16_t)std::clamp(o.data[0], -32768, 32767);
		r = (int16_t)std::clamp(o.data[1], -32768, 32767);
	}

private:
	ymfm::ym2151 m_chip;
	unsigned m_busy_until = 0, m_timer_at[2] = {0, 0};
	bool m_timer_on[2] = {false, false};
};

opm       chip;
uint8_t   latch;
bool      audio;                  /* generating samples */
unsigned  gen_last;               /* CPU cycle the generated audio reaches */
double    gen_frac;               /* YM samples owed, fraction */
int16_t   ring[RING][2];
unsigned  rd, wr;                 /* ring indices (wr - rd = samples ready) */
double    rd_frac;
FILE     *logf;
long      log_frame;

/* generate the samples between gen_last and now */
void gen_to_now() {
	if (!audio) return;
	unsigned d = ticks - gen_last; gen_last = ticks;
	gen_frac += d * (YM_RATE / CPU_HZ);
	int n = (int)gen_frac; gen_frac -= n;
	while (n-- > 0) {
		if (wr - rd >= RING) rd++;                      /* nobody listens: drop the oldest */
		chip.generate(ring[wr % RING][0], ring[wr % RING][1]);
		wr++;
	}
}

} // namespace

extern "C" {

void ym_init(int enabled) {
	g_ym_enabled = enabled;
	chip.reset();
	latch = 0;
	gen_last = ticks; gen_frac = 0; rd = wr = 0; rd_frac = 0;
}

uint8_t ym_read(int) {
	if (!g_ym_enabled) return ticks & 0xFF;
	chip.sync();
	return chip.status();
}

void ym_write(int reg, uint8_t value) {
	if (!g_ym_enabled) return;
	if (!(reg & 1)) { latch = value; return; }
	gen_to_now();                                       /* the write lands at this sample */
	chip.sync();
	if (chip.ymfm_is_busy()) {
		g_ym_busy_writes++;
		fprintf(stderr, "YM2151 write received while busy (reg $%02X = $%02X)\n", latch, value);
		return;
	}
	g_ym_writes++;
	if (logf) fprintf(logf, "%ld %02x %02x\n", log_frame, latch, value);
	chip.write(latch, value);
}

void ym_audio_on(void) {
	if (audio) return;
	audio = true; gen_last = ticks; gen_frac = 0; rd = 0; wr = LAG; rd_frac = 0;
	for (int i = 0; i < LAG; i++) ring[i][0] = ring[i][1] = 0;
}

void ym_mix(int16_t *buf, int n, double rate) {
	if (!g_ym_enabled || !audio) return;
	chip.sync();
	gen_to_now();
	/* resample the ring (YM_RATE) to `rate`, linear; hold the last sample on an underrun */
	double step = YM_RATE / rate;
	for (int i = 0; i < n; i++) {
		unsigned a = rd, b = rd + 1;
		if (wr - rd < 2) a = b = wr - 1;
		double f = rd_frac;
		double sl = ring[a % RING][0] + (ring[b % RING][0] - ring[a % RING][0]) * f;
		double sr = ring[a % RING][1] + (ring[b % RING][1] - ring[a % RING][1]) * f;
		int l = buf[2 * i] + (int)(sl / 2);              /* -6 dB against the PSG */
		int r = buf[2 * i + 1] + (int)(sr / 2);
		buf[2 * i]     = (int16_t)std::clamp(l, -32768, 32767);
		buf[2 * i + 1] = (int16_t)std::clamp(r, -32768, 32767);
		rd_frac += step;
		while (rd_frac >= 1.0) { rd_frac -= 1.0; if (wr - rd > 0) rd++; }
	}
	if (wr - rd > (unsigned)(YM_RATE / 30)) rd = wr - LAG;
}

void ym_log_open(FILE *f) { logf = f; }
void ym_log_frame(long frame) { log_frame = frame; }

}
