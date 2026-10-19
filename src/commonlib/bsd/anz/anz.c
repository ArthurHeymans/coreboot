/* SPDX-License-Identifier: MIT
 * ANZ: adaptive-nibble LZ. LZMA-class modelling with LZNA-style 16-ary
 * adaptive symbols so a literal costs two decode steps instead of eight:
 *  - rANS, two interleaved 64-bit states (symbol k uses state k & 1), 32-bit
 *    branchless renormalisation, 15-bit probabilities;
 *  - every model is a <=16-symbol CDF moved towards the coded symbol with an
 *    AV1-style shift (RATE-2 for 16 updates, RATE-1 to 32, then RATE);
 *  - commands as FHL: literal-run code (ctx: previous match-length code) and
 *    match-length code (ctx: run==0, previous was a rep), extra bits raw;
 *  - literal high nibble ctx = previous byte, low nibble ctx = high nibble and
 *    the top LCLO bits of the previous byte, each averaged with a dense model
 *    (top LH2/LL2 bits, faster rate); the first literal after a match uses the
 *    match byte instead (LZMA's matched literal, at nibble level);
 *  - distance symbol (3 reps + 40 slots) as 6-ary high (ctx: length) and 8-ary
 *    low part; slot extra bits raw except an adaptive 4-bit align field.
 * Raw bits travel in a second, forward LSB-first stream.
 *
 *   anz c [-l lclo] [-p passes] [-d depth] [-s settings] [-j threads] in out
 *   anz d in out             decompress
 *
 * Stream: u32 size | u8 lclo | u32 raw-stream offset - 9 | two u64 rANS
 * states | rANS words | raw bits | 8 zero bytes.
 * -DANZ_DECODER_ONLY keeps only anz_decode(); -DANZ_LIB drops main().
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "anz.h"

#define PB 15
#define TOT (1u << PB)
/* adaptation shifts per model group, hill-climbed on coreboot stages */
#define R_CMD 6     /* literal-run and match-length codes */
#define R_LHI 6     /* literal high nibble */
#define R_LLO 6     /* literal low nibble */
#define R_MAT 6     /* first literal after a match */
#define R_DIST 5    /* distance symbol */
#define R_AL 8      /* distance align bits */
#define R_LHI2 4    /* dense literal companions */
#define R_LLO2 5
#define WARM 2      /* start WARM shifts faster, one slower after 16 and 32 updates */
#define MINP 32     /* probability floor per symbol, in 1/TOT */
/* the sparse previous-byte literal models are averaged with dense ones keyed
 * by the top LH2/LL2 bits of the previous byte */
#define LH2 5
#define LL2 3
#define LCLO_MAX 6  /* previous-byte bits in the low-nibble context */
#define RL (1ull << 31)

static const uint32_t ll_base[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 14, 22, 38, 70, 134, 262};
static const uint8_t ll_bits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 20};
static const uint32_t ml_base[16] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16, 24, 40, 72, 136, 264};
static const uint8_t ml_bits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 16};

/* A model: CDF c[0..16] (c[0] = 0, c[n] stored as TOT-1 to fit int16). c[1]
 * sits on a 16-byte boundary so the two SIMD lane loads never split. */
struct __attribute__((aligned(16))) m16 {
	uint16_t cnt;
	uint8_t rate, pad0_;
	int16_t pad_[5];
	int16_t c[17];
};

static void m_init(struct m16 *m, unsigned n, unsigned rate)
{
	m->rate = (uint8_t)rate;
	for (unsigned i = 0; i <= 16; i++) m->c[i] = (int16_t)(i < n ? (i * TOT) / n : TOT - 1);
	m->c[n] = (int16_t)(TOT - 1);
	m->cnt = 0;
}
static inline unsigned m_hi(const struct m16 *m, unsigned s, unsigned n)
{
	return s + 1 == n ? TOT : (unsigned)m->c[s + 1];
}
static inline void m_update(struct m16 *m, unsigned s, unsigned n)
{
	/* AV1-style: the adaptive part a[i] = c[i] - i*MINP moves monotonically
	 * (towards 0 for i <= s, towards TOT - n*MINP above), so the fixed
	 * i*MINP floor keeps every symbol at probability >= MINP/TOT. */
	int r = m->rate - WARM + (m->cnt > 15) + (m->cnt > 31) + (WARM > 2 && m->cnt > 63);
	m->cnt += m->cnt < 128;
	for (unsigned i = 1; i < n; i++) {
		int a = m->c[i] - (int)(i * MINP);
		if (i <= s) a -= a >> r;
		else a += ((int)(TOT - n * MINP) - a) >> r;
		m->c[i] = (int16_t)(a + (int)(i * MINP));
	}
}

/* Optional input prefetch hint, e.g. to overlap decoding with flash reads. */
#ifndef ANZ_PREFETCH
#define ANZ_PREFETCH(p) ((void)0)
#endif

static inline uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static inline uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

/* two interleaved rANS states: symbol k uses state k & 1, so the decoder
 * swaps x and y after every symbol and the next symbol never waits for the
 * multiply/renormalise of the previous one */
struct ad { uint64_t x, y; const uint8_t *p; };
#if defined(__SSE2__) && !defined(ANZ_SCALAR)
#include <emmintrin.h>
/* Same arithmetic as the scalar path: lanes k = 0..15 hold c[k+1]. */
static inline void mu_simd(struct m16 *m, unsigned s, unsigned n);
static inline unsigned a_step(struct ad *a, __m128i c0, __m128i c1, const int16_t *cl, unsigned n)
{
	/* cl: scalar view of the coding CDF (c[0..16]) for lo/hi lookup */
	unsigned slot = (unsigned)(a->x & (TOT - 1));
	__m128i sv = _mm_set1_epi16((int16_t)slot);
	unsigned live = (1u << (2 * (n - 1))) - 1;
	unsigned le = (unsigned)(_mm_movemask_epi8(_mm_cmpgt_epi16(c0, sv)) |
				 (unsigned)_mm_movemask_epi8(_mm_cmpgt_epi16(c1, sv)) << 16);
	unsigned s = (unsigned)__builtin_popcount(~le & live) >> 1;
	unsigned lo = (unsigned)cl[s], hi = s + 1 == n ? TOT : (unsigned)cl[s + 1];
	uint64_t x = (uint64_t)(hi - lo) * (a->x >> PB) + slot - lo;
	uint64_t need = x < RL, w = rd32(a->p);
	ANZ_PREFETCH(a->p + 256);
	a->x = a->y;
	a->y = need ? x << 32 | w : x;
	a->p += need << 2;
	return s;
}
static inline unsigned a_sym(struct ad *a, struct m16 *m, unsigned n)
{
	__m128i *cv = (__m128i *)(void *)&m->c[1];
	unsigned s = a_step(a, _mm_loadu_si128(cv), _mm_loadu_si128(cv + 1), m->c, n);
	mu_simd(m, s, n);
	return s;
}
static inline unsigned a_sym2(struct ad *a, struct m16 *m, struct m16 *q, unsigned n)
{
	__m128i *cv = (__m128i *)(void *)&m->c[1], *qv = (__m128i *)(void *)&q->c[1];
	__m128i c0 = _mm_avg_epu16(_mm_loadu_si128(cv), _mm_loadu_si128(qv));
	__m128i c1 = _mm_avg_epu16(_mm_loadu_si128(cv + 1), _mm_loadu_si128(qv + 1));
	int16_t cl[17];
	cl[0] = 0;
	_mm_storeu_si128((__m128i *)(void *)&cl[1], c0);
	_mm_storeu_si128((__m128i *)(void *)&cl[9], c1);
	unsigned s = a_step(a, c0, c1, cl, n);
	mu_simd(m, s, n);
	mu_simd(q, s, n);
	return s;
}
static inline void mu_simd(struct m16 *m, unsigned s, unsigned n)
{
	static const int16_t ki[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	__m128i *cv = (__m128i *)(void *)&m->c[1];
	__m128i c0 = _mm_loadu_si128(cv), c1 = _mm_loadu_si128(cv + 1);
	int r = m->rate - WARM + (m->cnt > 15) + (m->cnt > 31) + (WARM > 2 && m->cnt > 63);
	m->cnt += m->cnt < 128;
	__m128i rc = _mm_cvtsi32_si128(r);
	__m128i idx0 = _mm_loadu_si128((const __m128i *)(const void *)ki);
	__m128i idx1 = _mm_add_epi16(idx0, _mm_set1_epi16(8));
	__m128i one = _mm_set1_epi16(1), mp = _mm_set1_epi16(MINP);
	__m128i im0 = _mm_mullo_epi16(_mm_add_epi16(idx0, one), mp);
	__m128i im1 = _mm_mullo_epi16(_mm_add_epi16(idx1, one), mp);
	__m128i A = _mm_set1_epi16((int16_t)(TOT - n * MINP)), sb = _mm_set1_epi16((int16_t)s);
	__m128i nb = _mm_set1_epi16((int16_t)(n - 1));
#define UPD(cx, idx, im, rc) ({                                                    \
		__m128i av = _mm_sub_epi16(cx, im);                               \
		__m128i dec = _mm_sub_epi16(av, _mm_sra_epi16(av, rc));           \
		__m128i inc = _mm_add_epi16(av, _mm_sra_epi16(_mm_sub_epi16(A, av), rc)); \
		__m128i le_s = _mm_cmplt_epi16(idx, sb);   /* i = k+1 <= s */      \
		__m128i nv = _mm_add_epi16(_mm_or_si128(_mm_and_si128(le_s, dec),   \
					_mm_andnot_si128(le_s, inc)), im);          \
		__m128i keep = _mm_cmplt_epi16(idx, nb);   /* live lanes */        \
		_mm_or_si128(_mm_and_si128(keep, nv), _mm_andnot_si128(keep, cx)); })
	_mm_storeu_si128(cv, UPD(c0, idx0, im0, rc));
	_mm_storeu_si128(cv + 1, UPD(c1, idx1, im1, rc));
#undef UPD
}
#else
static inline unsigned a_sym(struct ad *a, struct m16 *m, unsigned n)
{
	unsigned slot = (unsigned)(a->x & (TOT - 1)), s = 0;
	for (unsigned i = 1; i < n; i++) s += slot >= (unsigned)m->c[i];
	unsigned lo = (unsigned)m->c[s], hi = m_hi(m, s, n);
	uint64_t x = (uint64_t)(hi - lo) * (a->x >> PB) + slot - lo;
	if (x < RL) { x = x << 32 | rd32(a->p); ANZ_PREFETCH(a->p + 256); a->p += 4; }
	a->x = a->y;
	a->y = x;
	m_update(m, s, n);
	return s;
}
static inline unsigned a_sym2(struct ad *a, struct m16 *m, struct m16 *q, unsigned n)
{
	unsigned slot = (unsigned)(a->x & (TOT - 1)), s = 0;
	for (unsigned i = 1; i < n; i++) s += slot >= (unsigned)((m->c[i] + q->c[i] + 1) >> 1);
	unsigned lo = s ? (unsigned)((m->c[s] + q->c[s] + 1) >> 1) : 0;
	unsigned hi = s + 1 == n ? TOT : (unsigned)((m->c[s + 1] + q->c[s + 1] + 1) >> 1);
	uint64_t x = (uint64_t)(hi - lo) * (a->x >> PB) + slot - lo;
	if (x < RL) { x = x << 32 | rd32(a->p); ANZ_PREFETCH(a->p + 256); a->p += 4; }
	a->x = a->y;
	a->y = x;
	m_update(m, s, n);
	m_update(q, s, n);
	return s;
}
#endif

struct rb { uint64_t bb; unsigned bc; const uint8_t *p; };
static inline unsigned r_bits(struct rb *b, unsigned n)
{
	if (b->bc < n) {
		b->bb |= rd64(b->p) << b->bc; ANZ_PREFETCH(b->p + 256);
		b->p += (63 - b->bc) >> 3; b->bc |= 56;
	}
	unsigned v = (unsigned)(b->bb & ((1ull << n) - 1));
	b->bb >>= n; b->bc -= n;
	return v;
}

struct anz_models {
	struct m16 llm[16], mlm[16];            /* literal-run, match-length codes */
	struct m16 lhi[256 + 16], llo[(16 << LCLO_MAX) + 256];   /* literals; + matched */
	struct m16 dh[4], dl[8], al;            /* distance symbol, align bits */
	struct m16 lhi2[1 << LH2], llo2[16 << LL2];   /* dense companions, averaged in */
};
_Static_assert(sizeof(struct anz_models) <= ANZ_WORK_SIZE, "ANZ_WORK_SIZE too small");

static void models_init(struct anz_models *M)
{
	for (int i = 0; i < 16; i++) { m_init(&M->llm[i], 16, R_CMD); m_init(&M->mlm[i], 16, R_CMD); }
	for (int i = 0; i < 4; i++) m_init(&M->dh[i], 6, R_DIST);
	for (int i = 0; i < 8; i++) m_init(&M->dl[i], 8, R_DIST);
	for (int i = 0; i < 256; i++) m_init(&M->lhi[i], 16, R_LHI);
	for (int i = 0; i < 16 << LCLO_MAX; i++) m_init(&M->llo[i], 16, R_LLO);
	for (int i = 256; i < 256 + 16; i++) m_init(&M->lhi[i], 16, R_MAT);
	for (int i = 16 << LCLO_MAX; i < (16 << LCLO_MAX) + 256; i++) m_init(&M->llo[i], 16, R_MAT);
	m_init(&M->al, 16, R_AL);
	for (int i = 0; i < 1 << LH2; i++) m_init(&M->lhi2[i], 16, R_LHI2);
	for (int i = 0; i < 16 << LL2; i++) m_init(&M->llo2[i], 16, R_LLO2);
}

__attribute__((noinline))
size_t anz_decode(const void *srcv, size_t n, void *dstv, size_t cap, void *work)
{
	const uint8_t *src = srcv;
	uint8_t *dst = dstv;
	struct anz_models *M = work;
	if (n < 33) return 0;
	uint32_t size = rd32(src);
	unsigned lclo = src[4];
	if (size > cap || lclo > LCLO_MAX) return 0;
	uint32_t rlen = rd32(src + 5);
	models_init(M);
	struct ad a = { rd64(src + 9), rd64(src + 17), src + 25 };
	struct rb b = { 0, 0, src + 9 + rlen };
	uint8_t *o = dst, *end = dst + size;
	uint32_t r0 = 1, r1 = 1, r2 = 1;
	unsigned pml = 0, prep = 0, last = 0;
	int first = 0;
	for (;;) {
		unsigned llc = a_sym(&a, &M->llm[(pml > 3 ? 3 : pml) * 4 + prep * 2 + ((o - dst) & 1)], 16);
		uint32_t L = ll_base[llc] + r_bits(&b, ll_bits[llc]);
		for (uint32_t j = 0; j < L; j++) {
			unsigned hi, lo;
			if (first && j == 0) {
				unsigned mb = o[-(ptrdiff_t)r0];
				hi = a_sym(&a, &M->lhi[256 + (mb >> 4)], 16);
				lo = a_sym(&a, &M->llo[hi == mb >> 4 ? (16u << LCLO_MAX) + mb : hi << LCLO_MAX | last >> (8 - lclo)], 16);
			} else {
				hi = a_sym2(&a, &M->lhi[last], &M->lhi2[last >> (8 - LH2)], 16);
				lo = a_sym2(&a, &M->llo[hi << LCLO_MAX | last >> (8 - lclo)], &M->llo2[hi << LL2 | last >> (8 - LL2)], 16);
			}
			last = hi << 4 | lo;
			*o++ = (uint8_t)last;
		}
		if (o >= end) break;
		unsigned mlc = a_sym(&a, &M->mlm[((llc == 0) * 2 + prep) * 4 + ((o - dst) & 3)], 16);
		uint32_t Mlen = ml_base[mlc] + r_bits(&b, ml_bits[mlc]);
		unsigned lb = Mlen - 2 > 3 ? 3 : Mlen - 2;
		unsigned dhs = a_sym(&a, &M->dh[lb], 6);
		unsigned ds = dhs << 3 | a_sym(&a, &M->dl[dhs], 8);
		uint32_t d;
		if (ds == 0) d = r0;
		else if (ds == 1) { d = r1; r1 = r0; r0 = d; }
		else if (ds == 2) { d = r2; r2 = r1; r1 = r0; r0 = d; }
		else {
			unsigned slot = ds - 3;
			if (slot < 4) d = slot + 1;
			else {
				unsigned nb = (slot >> 1) - 1;
				uint32_t ex = nb >= 4 ? r_bits(&b, nb - 4) << 4 | a_sym(&a, &M->al, 16)
						      : r_bits(&b, nb);
				d = ((2u | (slot & 1)) << nb) + ex + 1;
			}
			r2 = r1; r1 = r0; r0 = d;
		}
		const uint8_t *s = o - d;
		if (d >= 8 && end - o >= (ptrdiff_t)Mlen + 8) {
			uint8_t *e = o + Mlen;
			do { memcpy(o, s, 8); o += 8; s += 8; } while (o < e);
			o = e;
		} else {
			while (Mlen--) *o++ = *s++;
		}
		last = o[-1];
		first = 1;
		pml = mlc; prep = ds < 3;
	}
	return (size_t)(o - dst);
}

#ifndef ANZ_DECODER_ONLY
/* ------------------------------------------------------------------ */
/* encoder: FHL's parser (fhl/fhl.c) produces the LZ parse, ANZ codes it */
/* ------------------------------------------------------------------ */
#include <pthread.h>
#include "fhl/fhl.h"

static unsigned ll_code(uint32_t L) { int c = 15; while (L < ll_base[c]) c--; return c; }
static unsigned ml_code(uint32_t M) { int c = 15; while (M < ml_base[c]) c--; return c; }

/* symbols are recorded as (start, freq) and rANS-coded backwards at the end;
 * raw bits go forward into their own stream */
struct es { uint32_t *st; size_t n, cap; uint8_t *raw; uint64_t acc; unsigned cnt; size_t rn; };
static void e_push(struct es *e, unsigned lo, unsigned hi)
{
	if (e->n + 1 >= e->cap) { e->cap *= 2; e->st = realloc(e->st, e->cap * 4); }
	e->st[e->n++] = lo << 16 | (hi - lo);
}
static void e_sym(struct es *e, struct m16 *m, unsigned s, unsigned n)
{
	e_push(e, (unsigned)m->c[s], m_hi(m, s, n));
	m_update(m, s, n);
}
/* code with the rounded average of two CDFs, update both */
static void e_sym2(struct es *e, struct m16 *a, struct m16 *b, unsigned s, unsigned n)
{
	unsigned lo = s ? (unsigned)((a->c[s] + b->c[s] + 1) >> 1) : 0;
	unsigned hi = s + 1 == n ? TOT : (unsigned)((a->c[s + 1] + b->c[s + 1] + 1) >> 1);
	e_push(e, lo, hi);
	m_update(a, s, n);
	m_update(b, s, n);
}
static void e_raw(struct es *e, uint32_t v, unsigned n)
{
	e->acc |= (uint64_t)v << e->cnt; e->cnt += n;
	while (e->cnt >= 8) { e->raw[e->rn++] = (uint8_t)e->acc; e->acc >>= 8; e->cnt -= 8; }
}

static size_t encode(const uint8_t *in, uint32_t size, const struct fhl_seq *s, size_t ns,
		     unsigned lclo, uint8_t *out)
{
	struct anz_models *M = aligned_alloc(16, sizeof(*M));
	models_init(M);
	struct es e = { malloc(1 << 20), 0, 1 << 18, malloc(size + 1024), 0, 0, 0 };
	uint32_t pos = 0, rep0 = 1;
	unsigned pml = 0, prep = 0;
	for (size_t i = 0; i < ns; i++) {
		unsigned llc = ll_code(s[i].L);
		e_sym(&e, &M->llm[(pml > 3 ? 3 : pml) * 4 + prep * 2 + (pos & 1)], llc, 16);
		e_raw(&e, s[i].L - ll_base[llc], ll_bits[llc]);
		for (uint32_t j = 0; j < s[i].L; j++, pos++) {
			unsigned v = in[pos], last = pos ? in[pos - 1] : 0, hi = v >> 4;
			if (i > 0 && j == 0) {
				unsigned mb = in[pos - rep0];
				e_sym(&e, &M->lhi[256 + (mb >> 4)], hi, 16);
				e_sym(&e, &M->llo[hi == mb >> 4 ? (16u << LCLO_MAX) + mb
							       : hi << LCLO_MAX | last >> (8 - lclo)], v & 15, 16);
			} else {
				e_sym2(&e, &M->lhi[last], &M->lhi2[last >> (8 - LH2)], hi, 16);
				e_sym2(&e, &M->llo[hi << LCLO_MAX | last >> (8 - lclo)],
				       &M->llo2[hi << LL2 | last >> (8 - LL2)], v & 15, 16);
			}
		}
		if (!s[i].M) break;
		unsigned ds = s[i].ds, mlc = ml_code(s[i].M);
		e_sym(&e, &M->mlm[((llc == 0) * 2 + prep) * 4 + (pos & 3)], mlc, 16);
		e_raw(&e, s[i].M - ml_base[mlc], ml_bits[mlc]);
		unsigned lb = s[i].M - 2 > 3 ? 3 : s[i].M - 2;
		e_sym(&e, &M->dh[lb], ds >> 3, 6);
		e_sym(&e, &M->dl[ds >> 3], ds & 7, 8);
		if (ds >= 7) {
			unsigned sl = ds - 3, nb = (sl >> 1) - 1;
			uint32_t ex = (s[i].d - 1) - ((2u | (sl & 1)) << nb);
			if (nb >= 4) { e_raw(&e, ex >> 4, nb - 4); e_sym(&e, &M->al, ex & 15, 16); }
			else e_raw(&e, ex, nb);
		}
		pml = mlc; prep = ds < 3;
		rep0 = s[i].d;
		pos += s[i].M;
	}
	if (e.cnt) e_raw(&e, 0, 8 - e.cnt);
	/* rANS encode backwards into 32-bit words, alternating the two states */
	uint32_t *words = malloc((e.n + 4) * 4);
	size_t nw = 0;
	uint64_t xs[2] = { RL, RL };
	for (size_t k = e.n; k-- > 0;) {
		uint64_t x = xs[k & 1];
		uint32_t lo = e.st[k] >> 16, f = e.st[k] & 0xffff;
		uint64_t xmax = ((RL >> PB) << 32) * f;
		if (x >= xmax) { words[nw++] = (uint32_t)x; x >>= 32; }
		xs[k & 1] = ((x / f) << PB) + (x % f) + lo;
	}
	uint8_t *p = out;
	memcpy(p, &size, 4); p[4] = (uint8_t)lclo;
	uint32_t rlen = (uint32_t)(16 + nw * 4);
	memcpy(p + 5, &rlen, 4);
	p += 9;
	memcpy(p, &xs[0], 8); p += 8;
	memcpy(p, &xs[1], 8); p += 8;
	for (size_t k = nw; k-- > 0;) { memcpy(p, &words[k], 4); p += 4; }
	memcpy(p, e.raw, e.rn); p += e.rn;
	memset(p, 0, 8); p += 8;
	free(words); free(e.st); free(e.raw); free(M);
	return (size_t)(p - out);
}

/* One parse setting of anz_compress(), run on its own thread. */
struct job {
	const uint8_t *in;
	uint32_t n;
	const struct fhl_mf *m;
	unsigned K, Kf, lclo, passes;
	uint8_t *out;
	size_t size;
};
static void *run_job(void *arg)
{
	struct job *j = arg;
	struct fhl_seq *s = malloc(sizeof(*s) * ((size_t)j->n + 2));
	size_t ns = fhl_parse_passes(j->in, j->n, j->m, j->K, j->Kf, (int)j->passes, s);
	j->out = malloc((size_t)j->n * 2 + 4096);
	j->size = encode(j->in, j->n, s, ns, j->lclo, j->out);
	free(s);
	return NULL;
}

/* Parse settings, cheapest first: K previous-byte and Kf match-byte literal
 * tables steer FHL's parse prices. Each setting is a full parse, so the sweep
 * is most of the encode time; (1,0) never won on coreboot stages, and -s 2 or
 * -s 1 trade 0.06% / 0.11% for 1.3x / 2.2x. Ties go to the earlier setting. */
static const unsigned cfg[][2] = { { 16, 0 }, { 16, 8 }, { 1, 0 } };
enum { NJ = sizeof(cfg) / sizeof(cfg[0]) };

size_t anz_compress_opts(const uint8_t *in, uint32_t n, const struct anz_opts *o, uint8_t *out)
{
	unsigned lclo = o->lclo, threads = o->threads, passes = o->passes;
	unsigned ns_ = o->settings < 1 ? 1 : o->settings > NJ ? NJ : o->settings;
	if (threads < 1) threads = 1;
	struct fhl_mf *m = fhl_matches_mt(in, n, o->depth, threads,
					  (o->tree ? FHL_MF_TREE : 0) | (o->greedy ? FHL_MF_GREEDY : 0));
	struct job jobs[NJ];
	pthread_t tid[NJ];
	for (unsigned k = 0; k < ns_; k++)
		jobs[k] = (struct job){ in, n, m, cfg[k][0], cfg[k][1], lclo, passes, NULL, 0 };
	for (unsigned a = 0; a < ns_; a += threads) {
		unsigned b = a + threads < ns_ ? a + threads : ns_;
		for (unsigned k = a + 1; k < b; k++) pthread_create(&tid[k], NULL, run_job, &jobs[k]);
		run_job(&jobs[a]);
		for (unsigned k = a + 1; k < b; k++) pthread_join(tid[k], NULL);
	}
	unsigned best = 0;
	for (unsigned k = 1; k < ns_; k++) best = jobs[k].size < jobs[best].size ? k : best;
	size_t sz = jobs[best].size;
	memcpy(out, jobs[best].out, sz);
	for (unsigned k = 0; k < ns_; k++) free(jobs[k].out);
	fhl_matches_free(m);
	return sz;
}

/* The original entry point: 4 passes, hash-chain depth 256, all three
 * settings. */
size_t anz_compress(const uint8_t *in, uint32_t n, unsigned lclo, unsigned threads, uint8_t *out)
{
	struct anz_opts o = { lclo, threads, 4, 256, 3, 0, 0 };
	return anz_compress_opts(in, n, &o, out);
}

#ifndef ANZ_LIB
static uint8_t *slurp(const char *path, size_t *n)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); exit(1); }
	fseek(f, 0, SEEK_END);
	*n = (size_t)ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *p = malloc(*n + 16);
	if (fread(p, 1, *n, f) != *n) { perror(path); exit(1); }
	fclose(f);
	return p;
}

int main(int argc, char **argv)
{
	static struct anz_models M;
	unsigned lclo = LCLO_MAX, threads = 3;
	struct anz_opts o = { LCLO_MAX, 3, 4, 256, 3, 0, 0 };
	int a = 2;
	while (a < argc && argv[a][0] == '-') {
		if (!strcmp(argv[a], "-t")) { o.tree = 1; a++; continue; }
		if (!strcmp(argv[a], "-g")) { o.greedy = 1; a++; continue; }
		if (a + 1 >= argc) break;
		if (!strcmp(argv[a], "-l")) o.lclo = (unsigned)atoi(argv[++a]);
		else if (!strcmp(argv[a], "-p")) o.passes = (unsigned)atoi(argv[++a]);
		else if (!strcmp(argv[a], "-d")) o.depth = (unsigned)atoi(argv[++a]);
		else if (!strcmp(argv[a], "-s")) o.settings = (unsigned)atoi(argv[++a]);
		else if (!strcmp(argv[a], "-j")) o.threads = (unsigned)atoi(argv[++a]);
		a++;
	}
	lclo = o.lclo; threads = o.threads;
	if (argc - a != 2 || (argv[1][0] != 'c' && argv[1][0] != 'd') || lclo > LCLO_MAX || threads < 1 ||
	    o.depth < 1 || o.settings < 1) {
		fprintf(stderr, "usage: anz c [-l 0..6] [-p passes] [-d depth] [-s 1..3] [-t] [-g] [-j threads] in out\n"
				"       anz d in out\n"
				"  -t  binary-tree match finder (serial; faster on one core, finds more)\n"
				"  -g  seed the prices with a greedy parse instead of a flat-priced one\n"
				"  fastest: -s 1 -p 1 -t -g\n");
		return 1;
	}
	size_t n, on;
	uint8_t *in = slurp(argv[a], &n), *out;
	if (argv[1][0] == 'c') {
		if (n > 0xffffffffu) { fprintf(stderr, "%s: too large\n", argv[a]); return 1; }
		out = malloc(n * 2 + 4096);
		on = anz_compress_opts(in, (uint32_t)n, &o, out);
		uint8_t *chk = malloc(n + 1);
		if (anz_decode(out, on, chk, n, &M) != n || memcmp(chk, in, n)) {
			fprintf(stderr, "%s: verify failed\n", argv[a]);
			return 2;
		}
		free(chk);
	} else {
		size_t size = n >= 33 ? rd32(in) : 0;
		out = malloc(size + 1);
		on = anz_decode(in, n, out, size, &M);
		if (on != size || n < 33) { fprintf(stderr, "%s: not an ANZ stream\n", argv[a]); return 2; }
	}
	FILE *f = fopen(argv[a + 1], "wb");
	if (!f || fwrite(out, 1, on, f) != on) { perror(argv[a + 1]); return 1; }
	fclose(f);
	free(in); free(out);
	return 0;
}
#endif
#endif
