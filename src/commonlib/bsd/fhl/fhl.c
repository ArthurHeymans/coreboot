/* SPDX-License-Identifier: MIT
 * FHL: firmware Huffman LZ. A per-stream semi-static Huffman LZ77 meant to sit
 * between LZSA2 (tiny, zero RAM) and LZMA (dense, slow):
 *  - LSB-first bitstreams, single-level Huffman lookup tables (MAXL bits);
 *  - a joint literal-run/match-length command symbol (16x16);
 *  - LZMA-style distance slots and three rep distances;
 *  - literals coded with K tables selected by an encoder-clustered map of the
 *    literal two before (not the previous output byte, see below).
 *
 * Decoding runs in two passes. All literals are decoded first, into the tail
 * of the output buffer; they alternate between two bitstreams and each
 * stream's context is its own previous literal, so the two Huffman chains
 * run in parallel. The commands then copy literal runs from the tail and
 * matches from the output, with wide over-copies for the leading nfast
 * commands (counted by the encoder) for which those stay in bounds.
 *
 *   fhl c [-K k] [-p passes] [-d depth] [-c] [-g] [-f] [-j threads] in out
 *                                               compress (and verify)
 *   fhl d in out                                 decompress
 *
 * K only enters through the literal prices, so -f prices every K = 1..16
 * from a single parse and writes the one with the fewest estimated literal
 * bits; the default re-parses for each K. Matches come from a binary tree,
 * or with -c from hash chains searched on several threads; -g swaps the
 * flat-priced first parse for a greedy one.
 *
 * Stream: u32 size | u8 K | u32 nlit | u32 nfast | u32 offsets of literal
 * streams A and B and of the distance stream | map | meta lengths | code
 * lengths | commands (code, literal-run and match-length extras) | the three
 * other streams. Every stream ends in 8 zero bytes of refill slack, the
 * last in 16 (the word reader looks one word further ahead).
 * -DFHL_DECODER_ONLY leaves only fhl_decode(); -DFHL_LIB drops main().
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fhl.h"
#ifndef FHL_DECODER_ONLY
#include <pthread.h>
#endif

#ifndef MAXL
#define MAXL 10                     /* max Huffman code length = LUT bits; 10 keeps K tables in L1 */
#endif
#define NLIT 256
#define NCMD 256
#define NDIST 43
#define NMETA 15
#define MAXK 16                     /* literal tables in a stream */
#ifndef PARSE_K
#define PARSE_K 4                   /* literal tables the single parse is priced with */
#endif
#define MAXT 32                     /* encoder: K + Kf tables in the ANZ parse mode */
#define NLEN (MAXT * NLIT + NCMD + NDIST)
#define MIN_MATCH 2
#define HDR 25

static const uint32_t ll_base[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 14, 22, 38, 70, 134, 262};
static const uint8_t ll_bits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 20};
static const uint32_t ml_base[16] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16, 24, 40, 72, 136, 264};
static const uint8_t ml_bits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 16};

static unsigned log2k(unsigned k) { return k > 8 ? 4 : k > 4 ? 3 : k > 2 ? 2 : k > 1 ? 1 : 0; }

/* ------------------------------------------------------------------ */
/* decoder                                                             */
/* ------------------------------------------------------------------ */

struct br { uint64_t bb; unsigned bc; const uint8_t *p; };

/* Explicit builtins keep fixed-size loads/copies inline in freestanding builds. */
static inline uint64_t rd64(const uint8_t *p) { uint64_t v; __builtin_memcpy(&v, p, 8); return v; }
static inline uint32_t rd32le(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static inline void refill(struct br *b)
{
	b->bb |= rd64(b->p) << b->bc;
	b->p += (63 - b->bc) >> 3;
	b->bc |= 56;
}
static inline unsigned bits(struct br *b, unsigned n)
{
	unsigned v = (unsigned)(b->bb & ((1ull << n) - 1));
	b->bb >>= n; b->bc -= n;
	return v;
}
static inline unsigned sym(struct br *b, const uint16_t *lut, unsigned lb)
{
	unsigned e = lut[b->bb & ((1u << lb) - 1)];
	unsigned n = (e >> 8) & 15;
	b->bb >>= n; b->bc -= n;
	return e & 255;
}

/* Canonical codes, bit-reversed for an LSB-first reader, into rev[]. */
static void rcodes(const uint8_t *len, unsigned n, uint16_t *rev)
{
	unsigned cnt[MAXL + 2] = {0}, next[MAXL + 2], code = 0;
	for (unsigned s = 0; s < n; s++) cnt[len[s]]++;
	cnt[0] = 0;
	for (unsigned l = 1; l <= MAXL; l++) { code = (code + cnt[l - 1]) << 1; next[l] = code; }
	for (unsigned s = 0; s < n; s++) {
		unsigned l = len[s], x = l ? next[l]++ : 0;
		x = (x & 0x5555) << 1 | (x >> 1 & 0x5555);
		x = (x & 0x3333) << 2 | (x >> 2 & 0x3333);
		x = (x & 0x0f0f) << 4 | (x >> 4 & 0x0f0f);
		x = (x & 0x00ff) << 8 | (x >> 8 & 0x00ff);
		rev[s] = (uint16_t)(x >> (16 - l));   /* l == 0: unused */
	}
}
/* Fill a 2^lb LUT with ent(sym, len). A lone symbol has length 1 but only
 * one code: it gets both halves. */
#define FILL_LUT(lut, len, n, lb, ent) do {                                   \
	uint16_t rev_[256];                                                     \
	unsigned used_ = 0, one_ = 0;                                           \
	rcodes(len, n, rev_);                                                   \
	for (unsigned s_ = 0; s_ < (n); s_++) {                                \
		unsigned l_ = (len)[s_];                                        \
		if (!l_) continue;                                              \
		for (unsigned i_ = rev_[s_]; i_ < (1u << (lb)); i_ += 1u << l_) \
			(lut)[i_] = ent(s_, l_);                                \
		used_ += 1u << ((lb) - l_);                                     \
		one_ = s_;                                                      \
	}                                                                       \
	if (used_ == (1u << (lb)) >> 1)                                         \
		for (unsigned i_ = 0; i_ < (1u << (lb)); i_++) (lut)[i_] = ent(one_, 1); \
} while (0)

/* 16-bit entries sym | len << 8 | ctx[sym] << 12: literal tables carry the
 * context the decoded literal selects, so the next table needs no lookup. */
static void build_lut(uint16_t *lut, const uint8_t *len, unsigned n, unsigned lb, const uint8_t *ctx)
{
#define ENT16(s, l) (uint16_t)((s) | (l) << 8 | (ctx ? ctx[s] << 12 : 0))
	FILL_LUT(lut, len, n, lb, ENT16);
#undef ENT16
}

/* command: llx 5 | llb 9 | mlx 5 | mlb 9 (shifted by 4 for the length) */
static uint32_t cmd_payload(unsigned c)
{
	unsigned ll = c >> 4, ml = c & 15;
	return ll_bits[ll] | ll_base[ll] << 5 | (uint32_t)ml_bits[ml] << 14 | (uint32_t)ml_base[ml] << 19;
}
/* distance: rep 2 (3 = new) | nb 5 | base 21 */
static uint32_t dist_payload(unsigned ds)
{
	if (ds < 3) return ds;
	unsigned slot = ds - 3;
	if (slot < 4) return 3 | (uint32_t)(slot + 1) << 7;
	unsigned nb = (slot >> 1) - 1;
	return 3 | nb << 2 | (((2u | (slot & 1)) << nb) + 1) << 7;
}

/* 32-bit LUT for command/distance symbols: len in bits 0-3, payload above */
static void build_lut32(uint32_t *lut, const uint8_t *len, unsigned n, uint32_t (*payload)(unsigned sym))
{
	uint32_t pay[256];
	for (unsigned s = 0; s < n; s++) pay[s] = payload(s) << 4;
#define ENT32(s, l) ((l) | pay[s])
	FILL_LUT(lut, len, n, MAXL, ENT32);
#undef ENT32
}

struct dtab {
	uint16_t lit[MAXK][1 << MAXL];
	uint32_t cmd[1 << MAXL], dist[1 << MAXL];
	uint64_t msk[33];
	uint8_t len[MAXK * NLIT + NCMD + NDIST];
	uint32_t size;
	unsigned lit0;          /* table of the initial context */
};

_Static_assert(sizeof(struct dtab) <= FHL_WORK_SIZE, "FHL_WORK_SIZE too small");

/* Header and tables; returns the bit position of the command stream. */
__attribute__((noinline)) static uint64_t fhl_setup(const uint8_t *src, struct dtab *t)
{
	t->size = rd32le(src);
	unsigned K = src[4], kb = log2k(K);
	struct br b = { 0, 0, src + HDR };
	uint8_t map[256];
	for (unsigned i = 0; i < 256; i++) {
		refill(&b);
		map[i] = (uint8_t)bits(&b, kb);
	}
	uint8_t mlen[NMETA];
	uint16_t mlut[1 << 7];
	refill(&b);
	for (unsigned i = 0; i < NMETA; i++) mlen[i] = (uint8_t)bits(&b, 3);
	build_lut(mlut, mlen, NMETA, 7, NULL);
	unsigned total = K * NLIT + NCMD + NDIST, i = 0, prev = 0;
	uint8_t *len = t->len;
	while (i < total) {
		refill(&b);
		unsigned s = sym(&b, mlut, 7), r;
		if (s < 12) { len[i++] = (uint8_t)(prev = s); continue; }
		if (s == 12) { r = 3 + bits(&b, 2); while (r--) len[i++] = (uint8_t)prev; continue; }
		r = s == 13 ? 3 + bits(&b, 3) : 11 + bits(&b, 7);
		while (r--) len[i++] = 0;
		prev = 0;
	}
	for (unsigned k = 0; k < K; k++) build_lut(t->lit[k], len + k * NLIT, NLIT, MAXL, map);
	build_lut32(t->cmd, len + K * NLIT, NCMD, cmd_payload);
	build_lut32(t->dist, len + K * NLIT + NCMD, NDIST, dist_payload);
	for (unsigned k = 0; k <= 32; k++) t->msk[k] = k ? ~0ull >> (64 - k) : 0;
	t->lit0 = map[0];
	return (uint64_t)(b.p - src) * 8 - b.bc;
}

/* FHL_WORD_READS reads the bitstreams as aligned native words instead of
 * unaligned 8-byte loads. It is faster where those loads are expensive:
 * 32-bit targets, where the 64-bit bit buffer costs several instructions
 * per operation (30% on a Ryzen 9, 10% on an Atom D410), and targets that
 * split or trap misaligned loads. It is 15-20% slower on x86-64. */
#ifndef FHL_WORD_READS
#if __SIZEOF_POINTER__ >= 8 && (defined(__x86_64__) || defined(__aarch64__) || \
				  defined(__riscv_misaligned_fast))
#define FHL_WORD_READS 0
#else
#define FHL_WORD_READS 1
#endif
#endif

#if FHL_WORD_READS
/* Aligned word window: w0 holds the word containing the next bit, w1 the
 * one after. A peek returns the FW_BITS bits from the current position;
 * skipping at most FW_BITS bits moves the window by at most one word. */
#if __SIZEOF_POINTER__ > 4
typedef uint64_t fw;
typedef unsigned __int128 fdw;
#define FW_LOG 6
#else
typedef uint32_t fw;
typedef uint64_t fdw;
#define FW_LOG 5
#endif
#define FW_BITS (1u << FW_LOG)
#define FW_MASK(n) ((fw)t->msk[n])
#define LPW (FW_BITS / MAXL)        /* literal codes per window peek */
struct win { fw w0, w1, w2; unsigned bit; const fw *p; };
static inline void win_init(struct win *w, const fw *base, uint64_t pos)
{
	w->p = base + (pos >> FW_LOG);
	w->bit = (unsigned)pos & (FW_BITS - 1);
	w->w0 = w->p[0];
	w->w1 = w->p[1];
	w->w2 = w->p[2];
}
static inline fw win_peek(const struct win *w)
{
	return (fw)((((fdw)w->w1 << FW_BITS) | w->w0) >> (w->bit & (FW_BITS - 1)));
}
/* Stateless peek at an absolute bit position: two aligned loads. */
static inline fw peek_at(const fw *base, uint64_t pos)
{
	const fw *p = base + (pos >> FW_LOG);
	return (fw)((((fdw)p[1] << FW_BITS) | p[0]) >> ((unsigned)pos & (FW_BITS - 1)));
}
/* The third word keeps the load off the critical path: the next peek only
 * waits for the selects, the load is needed one advance later. */
static inline void win_skip(struct win *w, unsigned n)
{
	unsigned b = w->bit + n, adv = b >> FW_LOG;
	w->bit = b & (FW_BITS - 1);
	w->p += adv;
	w->w0 = adv ? w->w1 : w->w0;
	w->w1 = adv ? w->w2 : w->w1;
	w->w2 = w->p[2];
}

#endif

/* gcc's SLP pass turns the rep array below into xmm inserts: slower */
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("no-tree-slp-vectorize")))
#endif
__attribute__((noinline)) size_t fhl_decode(const void *srcv, size_t n, void *dstv, size_t cap, void *work)
{
	const uint8_t *src = srcv;
	uint8_t *dst = dstv;
	struct dtab *t = work;
	if (n < HDR || src[4] == 0 || src[4] > MAXK) return 0;
	uint64_t cpos = fhl_setup(src, t);
	uint32_t size = t->size, nlit = rd32le(src + 5), nfast = rd32le(src + 9);
	if (size > cap || nlit > size) return 0;
	uint8_t *o = dst, *const end = dst + size;
	const uint8_t *lr = end - nlit;

#if FHL_WORD_READS
	/* Every stream is read through a window of two aligned native words,
	 * so no load straddles a word (or cache line) and the bit buffer is a
	 * register wide on any target. Bit positions count from the aligned
	 * word below src. */
	const uintptr_t mis = (uintptr_t)src & (sizeof(fw) - 1);
	const fw *const base = (const fw *)(const void *)(src - mis);
	const uint64_t bias = (uint64_t)mis * 8;

	/* pass 1: literal 2i from stream A, 2i+1 from stream B, into the tail */
	{
		struct win a, b;
		win_init(&a, base, bias + (uint64_t)rd32le(src + 13) * 8);
		win_init(&b, base, bias + (uint64_t)rd32le(src + 17) * 8);
		const uint16_t *const lits = t->lit[0];
		const uint16_t *ta = t->lit[t->lit0], *tb = ta;
		uint8_t *q = end - nlit;
		uint32_t k = nlit;
#define LIT(v, s, tt) do {                                                     \
		unsigned e_ = tt[v & ((1u << MAXL) - 1)], k_ = (e_ >> 8) & 15; \
		v >>= k_; s += k_;                                            \
		*q++ = (uint8_t)e_;                                           \
		tt = lits + ((size_t)(e_ >> 12) << MAXL);                     \
	} while (0)
		while (k >= 2 * LPW) {     /* LPW codes of <= MAXL bits per window */
			fw va = win_peek(&a), vb = win_peek(&b);
			unsigned sa = 0, sb = 0;
#pragma GCC unroll 8
			for (unsigned i = 0; i < LPW; i++) { LIT(va, sa, ta); LIT(vb, sb, tb); }
			win_skip(&a, sa); win_skip(&b, sb);
			k -= 2 * LPW;
		}
		while (k >= 2) {
			fw va = win_peek(&a), vb = win_peek(&b);
			unsigned sa = 0, sb = 0;
			LIT(va, sa, ta); LIT(vb, sb, tb);
			win_skip(&a, sa); win_skip(&b, sb);
			k -= 2;
		}
		if (k) { fw va = win_peek(&a); unsigned sa = 0; LIT(va, sa, ta); (void)sa; }
#undef LIT
	}

	/* pass 2: commands. One peek covers a command code and both length
	 * extras unless they exceed a word; distances come from their own
	 * stream. Both are read by absolute bit position, which keeps the
	 * state to one register each. */
	uint64_t cp = bias + cpos, dp = bias + (uint64_t)rd32le(src + 21) * 8;
	uint32_t r0 = 1, r1 = 1, r2 = 1;
	unsigned fast = nfast != 0;
	for (;;) {
		fw pk = peek_at(base, cp);
		uint32_t ce = t->cmd[pk & ((1u << MAXL) - 1)];
		unsigned l0 = ce & 15, llx = (ce >> 4) & 31, mlx = (ce >> 18) & 31;
		uint32_t L = (ce >> 9) & 511, M = ce >> 23;
		if (__builtin_expect(l0 + llx + mlx <= FW_BITS, 1)) {
			L += (uint32_t)((pk >> l0) & FW_MASK(llx));
			M += (uint32_t)((pk >> (l0 + llx)) & FW_MASK(mlx));
		} else {
			L += (uint32_t)(peek_at(base, cp + l0) & FW_MASK(llx));
			M += (uint32_t)(peek_at(base, cp + l0 + llx) & FW_MASK(mlx));
		}
		cp += l0 + llx + mlx;
		if (__builtin_expect(fast, 1)) {
			__builtin_memcpy(o, lr, 16);
			if (L > 16) __builtin_memmove(o + 16, lr + 16, L - 16);
			o += L; lr += L;
		} else {
			while (L--) *o++ = *lr++;
			if (o >= end) break;
		}
		fw dk = peek_at(base, dp);
		uint32_t de = t->dist[dk & ((1u << MAXL) - 1)];
		unsigned d0 = de & 15, rep = (de >> 4) & 3, nb = (de >> 6) & 31;
		uint32_t far = (de >> 11) + (uint32_t)((dk >> d0) & FW_MASK(nb));
		dp += d0 + nb;
		uint32_t rr[4] = { r0, r1, r2, far };  /* indexed: a branch chain mispredicts */
		uint32_t d = rr[rep];
		r2 = rep >= 2 ? r1 : r2;
		r1 = rep >= 1 ? r0 : r1;
		r0 = d;
		const uint8_t *s = o - d;
		uint8_t *e = o + M;
		if (__builtin_expect(fast && d >= 8, 1)) {
			/* two 8-byte halves: valid for any d >= 8, covers M <= 16 */
			__builtin_memcpy(o, s, 8);
			__builtin_memcpy(o + 8, s + 8, 8);
			if (__builtin_expect(M > 16, 0)) {
				o += 16; s += 16;
				do { __builtin_memcpy(o, s, 8); o += 8; s += 8; } while (o < e);
			}
		} else {
			while (o < e) *o++ = *s++;
		}
		o = e;
		/* Once cleared, fast stays clear even when the countdown wraps. */
		fast &= --nfast != 0;
	}
#else
	/* pass 1: literal 2i from stream A, 2i+1 from stream B, into the tail */
	{
		const uint8_t *pa = src + rd32le(src + 13), *pb = src + rd32le(src + 17);
		const uint16_t *const lits = t->lit[0];
		const uint16_t *ta = t->lit[t->lit0], *tb = ta;
		uint64_t ba = 0, bb = 0;
		unsigned ca = 0, cb = 0;
		uint8_t *q = end - nlit;
		uint32_t k = nlit;
#define RF(b, c, p) do { b |= rd64(p) << c; p += (63 - c) >> 3; c |= 56; } while (0)
#define LIT(b, c, tt) do {                                                     \
		unsigned e_ = tt[b & ((1u << MAXL) - 1)], k_ = (e_ >> 8) & 15; \
		b >>= k_; c -= k_;                                            \
		*q++ = (uint8_t)e_;                                           \
		tt = lits + ((size_t)(e_ >> 12) << MAXL);                     \
	} while (0)
		while (k >= 10) {          /* 5 codes of <= MAXL bits per refill */
			RF(ba, ca, pa); RF(bb, cb, pb);
			LIT(ba, ca, ta); LIT(bb, cb, tb); LIT(ba, ca, ta); LIT(bb, cb, tb);
			LIT(ba, ca, ta); LIT(bb, cb, tb); LIT(ba, ca, ta); LIT(bb, cb, tb);
			LIT(ba, ca, ta); LIT(bb, cb, tb);
			k -= 10;
		}
		RF(ba, ca, pa); RF(bb, cb, pb);
		while (k >= 2) { LIT(ba, ca, ta); LIT(bb, cb, tb); k -= 2; }
		if (k) LIT(ba, ca, ta);
#undef RF
#undef LIT
	}

	/* pass 2: commands. A 64-bit peek covers a command code and both length
	 * extras; distances come from their own stream. */
	uint64_t dpos = (uint64_t)rd32le(src + 21) * 8;
	uint32_t r0 = 1, r1 = 1, r2 = 1;
	unsigned fast = nfast != 0;
	for (;;) {
		uint64_t pk = rd64(src + (cpos >> 3)) >> (cpos & 7);
		uint32_t ce = t->cmd[pk & ((1u << MAXL) - 1)];
		unsigned l0 = ce & 15, llx = (ce >> 4) & 31, mlx = (ce >> 18) & 31;
		uint32_t L = ((ce >> 9) & 511) + (uint32_t)((pk >> l0) & t->msk[llx]);
		uint32_t M = (ce >> 23) + (uint32_t)((pk >> (l0 + llx)) & t->msk[mlx]);
		cpos += l0 + llx + mlx;
		if (__builtin_expect(fast, 1)) {
			__builtin_memcpy(o, lr, 16);
			if (L > 16) __builtin_memmove(o + 16, lr + 16, L - 16);
			o += L; lr += L;
		} else {
			while (L--) *o++ = *lr++;
			if (o >= end) break;
		}
		uint64_t dk = rd64(src + (dpos >> 3)) >> (dpos & 7);
		uint32_t de = t->dist[dk & ((1u << MAXL) - 1)];
		unsigned d0 = de & 15, rep = (de >> 4) & 3, nb = (de >> 6) & 31;
		uint32_t far = (de >> 11) + (uint32_t)((dk >> d0) & t->msk[nb]);
		dpos += d0 + nb;
		uint32_t rr[4] = { r0, r1, r2, far };  /* indexed: a branch chain mispredicts */
		uint32_t d = rr[rep];
		r2 = rep >= 2 ? r1 : r2;
		r1 = rep >= 1 ? r0 : r1;
		r0 = d;
		const uint8_t *s = o - d;
		uint8_t *e = o + M;
		if (__builtin_expect(fast && d >= 8, 1)) {
			/* two 8-byte halves: valid for any d >= 8, covers M <= 16 */
			__builtin_memcpy(o, s, 8);
			__builtin_memcpy(o + 8, s + 8, 8);
			if (__builtin_expect(M > 16, 0)) {
				o += 16; s += 16;
				do { __builtin_memcpy(o, s, 8); o += 8; s += 8; } while (o < e);
			}
		} else {
			while (o < e) *o++ = *s++;
		}
		o = e;
		/* Once cleared, fast stays clear even when the countdown wraps. */
		fast &= --nfast != 0;
	}
#endif
	return (size_t)(o - dst);
}

#ifndef FHL_DECODER_ONLY
/* ------------------------------------------------------------------ */
/* encoder                                                             */
/* ------------------------------------------------------------------ */

struct bw { uint8_t *p; uint64_t acc; unsigned cnt; };
static void put(struct bw *w, uint32_t v, unsigned n)
{
	w->acc |= (uint64_t)v << w->cnt;
	w->cnt += n;
	while (w->cnt >= 8) { *w->p++ = (uint8_t)w->acc; w->acc >>= 8; w->cnt -= 8; }
}

/* Huffman code lengths limited to maxl (frequency halving until it fits).
 *
 * The live nodes sit in slots 0..na-1; each step merges the two lightest
 * (the lower slot wins ties), moves the node in the last slot into the
 * higher freed one and puts the new node in the lower. A heap of keys
 * weight:slot:node, all distinct, makes those choices in O(log n). */
#define HK(f, slot, node) ((f) << 18 | (uint64_t)(slot) << 9 | (uint64_t)(node))
static inline void hsift(uint64_t *h, int *pos, int n, int i)
{
	uint64_t x = h[i];
	for (;;) {
		int c = 2 * i + 1;
		if (c >= n) break;
		c += c + 1 < n && h[c + 1] < h[c];
		if (h[c] >= x) break;
		h[i] = h[c]; pos[h[i] & 511] = i; i = c;
	}
	h[i] = x; pos[x & 511] = i;
}
static inline void hrise(uint64_t *h, int *pos, int i)
{
	uint64_t x = h[i];
	for (; i > 0 && x < h[(i - 1) / 2]; i = (i - 1) / 2) { h[i] = h[(i - 1) / 2]; pos[h[i] & 511] = i; }
	h[i] = x; pos[x & 511] = i;
}

static void huff_len(const uint32_t *freq, unsigned n, unsigned maxl, uint8_t *len)
{
	uint64_t f[512], h[512];
	int par[512], at[512], slot[512], pos[512];
	uint8_t dep[512];
	for (unsigned scale = 0;; scale++) {
		int nn = 0;
		memset(len, 0, n);
		for (unsigned s = 0; s < n; s++)
			if (freq[s]) {
				f[nn] = scale ? (freq[s] >> scale) + 1 : freq[s];
				at[nn] = nn; slot[nn] = nn;
				h[nn] = HK(f[nn], nn, nn);
				nn++;
			}
		if (nn == 0) return;
		if (nn == 1) { for (unsigned s = 0; s < n; s++) if (freq[s]) len[s] = 1; return; }
		int hn = nn;
		for (int i = hn / 2; i-- > 0;) hsift(h, pos, hn, i);
		for (int i = hn / 2; i < hn; i++) pos[h[i] & 511] = i;
		for (int na = nn; na > 1; nn++) {
			int a = h[0] & 511;
			h[0] = h[--hn]; hsift(h, pos, hn, 0);
			int b = h[0] & 511;
			f[nn] = f[a] + f[b];
			par[a] = nn; par[b] = nn;
			int hi = slot[a] > slot[b] ? slot[a] : slot[b], lo = slot[a] + slot[b] - hi;
			/* the new node takes b's place in the heap */
			at[lo] = nn; slot[nn] = lo;
			h[0] = HK(f[nn], lo, nn); hsift(h, pos, hn, 0);
			int last = at[--na];
			if (hi != na) {
				at[hi] = last; slot[last] = hi;
				int i = pos[last];
				h[i] = HK(f[last], hi, last); hrise(h, pos, i);
			}
		}
		/* depths top-down: parents come after their children */
		unsigned mx = 0;
		dep[(nn - 1) & 511] = 0;   /* (the mask quiets a bogus overflow warning) */
		for (int x = nn - 1; x-- > 0;) dep[x] = dep[par[x]] + 1;
		for (unsigned s = 0, k = 0; s < n; s++) {
			if (!freq[s]) continue;
			len[s] = dep[k]; if (dep[k] > mx) mx = dep[k];
			k++;
		}
		if (mx <= maxl) return;
	}
}

static void codes(const uint8_t *len, unsigned n, unsigned maxl, uint32_t *code)
{
	unsigned cnt[16] = {0}, next[16], c = 0;
	for (unsigned s = 0; s < n; s++) cnt[len[s]]++;
	cnt[0] = 0;
	for (unsigned l = 1; l <= maxl; l++) { c = (c + cnt[l - 1]) << 1; next[l] = c; }
	for (unsigned s = 0; s < n; s++) {
		unsigned l = len[s], cc = l ? next[l]++ : 0, r = 0;
		for (unsigned i = 0; i < l; i++) r |= ((cc >> i) & 1) << (l - 1 - i);
		code[s] = r;
	}
}

static unsigned dist_slot(uint32_t v)
{
	if (v < 4) return v;
	unsigned n = 31 - __builtin_clz(v);
	return 2 * n + ((v >> (n - 1)) & 1);
}
/* past the 8 one-value codes the buckets double: 2, 4, ..., 128, then the rest */
static inline unsigned ll_code(uint32_t L)
{
	if (L < 8) return L;
	unsigned c = 7 + (31 - __builtin_clz(L - 6));
	return c < 15 ? c : 15;
}
static inline unsigned ml_code(uint32_t M)
{
	if (M < 10) return M - 2;
	unsigned c = 7 + (31 - __builtin_clz(M - 8));
	return c < 15 ? c : 15;
}

/* ds: 0..2 rep, else 3 + distance slot; d the actual distance */
#define seq fhl_seq

struct model {
	unsigned K, Kf;
	int legacy;     /* ANZ pricing, see fhl_parse() */
	uint8_t map[256], mapf[256];
};

struct cost {                         /* 1/16 bit */
	uint32_t lit[MAXT][256], cmd[256], dist[NDIST];
};

static void lens_to_cost(const uint8_t *len, unsigned n, uint32_t *c, unsigned miss)
{
	for (unsigned s = 0; s < n; s++) c[s] = len[s] ? len[s] * 16 : miss * 16;
}

struct cand { uint32_t len, dist; };
struct mf { struct cand *c; uint32_t *off; };

static uint32_t mlen(const uint8_t *a, const uint8_t *b, uint32_t max)
{
	uint32_t l = 0;
	while (l + 8 <= max) {
		uint64_t x = rd64(a + l) ^ rd64(b + l);
		if (x) return l + (__builtin_ctzll(x) >> 3);
		l += 8;
	}
	while (l < max && a[l] == b[l]) l++;
	return l;
}

#define MAXM (264 + 65535)
#define NICE 1024
#define MAXD (1u << 20)
/* Match finder: hash chains over 3 bytes, every candidate longer than the
 * ones before it, nearest first. Inside a match of NICE or more only its
 * continuation is a candidate (the parser jumps over it anyway).
 *
 * The chains are built first, so each position's search is independent
 * but for that long-match state, and runs on several threads by segment.
 * A segment starts as if no long match were running; the boundary is then
 * redone serially with the state the previous segment really ended in,
 * until it agrees with the segment's own: the candidates are the serial
 * ones, whatever the thread count. */
struct cbuf { struct cand *c; size_t k, cap; };
static inline void cpush(struct cbuf *b, uint32_t len, uint32_t dist)
{
	if (b->k == b->cap) { b->cap = b->cap * 2 + 1024; b->c = realloc(b->c, b->cap * sizeof(*b->c)); }
	b->c[b->k++] = (struct cand){ len, dist };
}
struct run { uint32_t end, d; };     /* a match >= NICE covers i..end */
static inline int run_on(struct run r, uint32_t i) { return r.end >= i + NICE; }

/* Candidates of position i in state *r. */
static void mf_pos(const uint8_t *in, uint32_t n, const uint32_t *prv, unsigned depth, uint32_t i,
		   struct run *r, struct cbuf *b)
{
	if (i + 3 > n) return;
	uint32_t j = prv[i], best = 2, max = n - i < MAXM ? n - i : MAXM;
	if (run_on(*r, i)) {
		cpush(b, r->end - i < max ? r->end - i : max, r->d);
		return;
	}
	for (unsigned dd = 0; dd < depth && j != 0xffffffff; dd++, j = prv[j]) {
		if (i - j > MAXD) break;
		if (in[j + best] != in[i + best]) continue;
		uint32_t l = mlen(in + i, in + j, max);
		if (l > best) {
			cpush(b, l, i - j);
			best = l;
			if (l >= NICE) *r = (struct run){ i + l, i - j };
			if (l >= NICE || l == max) break;
		}
	}
}

struct mfseg {
	const uint8_t *in;
	const uint32_t *prv;
	uint32_t n, a, b;
	unsigned depth;
	struct cbuf c;
	uint32_t *off;                  /* b - a + 1 offsets into c */
	struct { uint32_t i; struct run r; } *ev;   /* long matches started, by position */
	size_t nev;
};
struct mfq { struct mfseg *g; unsigned ns, next; };
static void mf_one(struct mfseg *g);
static void *mf_work(void *arg)
{
	struct mfq *q = arg;
	for (unsigned t; (t = __atomic_fetch_add(&q->next, 1, __ATOMIC_RELAXED)) < q->ns;) mf_one(&q->g[t]);
	return NULL;
}
static void mf_one(struct mfseg *g)
{
	struct run r = { 0, 0 };
	size_t cap = 0;
	for (uint32_t i = g->a; i < g->b; i++) {
		g->off[i - g->a] = (uint32_t)g->c.k;
		struct run r0 = r;
		mf_pos(g->in, g->n, g->prv, g->depth, i, &r, &g->c);
		if (r.end != r0.end) {
			if (g->nev == cap) { cap = cap * 2 + 16; g->ev = realloc(g->ev, cap * sizeof(*g->ev)); }
			g->ev[g->nev].i = i; g->ev[g->nev++].r = r;
		}
	}
	g->off[g->b - g->a] = (uint32_t)g->c.k;
}

/* Binary-tree match finder (as LZMA's bt), for -t: each position goes into
 * a tree of the earlier positions with its 3-byte hash, ordered by what
 * follows them, and the search walks at most 'depth' nodes of it. Serial,
 * but it finds better matches than a chain of the same depth, faster. */
static void find_matches_tree(const uint8_t *in, uint32_t n, unsigned depth, struct mf *m)
{
	enum { HB = 18 };
	const uint32_t NIL = 0xffffffffu;
	uint32_t *head = malloc(sizeof(uint32_t) << HB), *son = malloc(8 * ((size_t)n + 1));
	memset(head, 0xff, sizeof(uint32_t) << HB);
	struct cbuf b = { NULL, 0, 0 };
	m->off = malloc(4 * ((size_t)n + 1));
	struct run r = { 0, 0 };
	for (uint32_t i = 0; i < n; i++) {
		m->off[i] = (uint32_t)b.k;
		if (i + 3 > n) continue;
		uint32_t max = n - i < MAXM ? n - i : MAXM, nice = max < NICE ? max : NICE;
		uint32_t h = ((in[i] | in[i + 1] << 8 | in[i + 2] << 16) * 2654435761u) >> (32 - HB);
		int on = run_on(r, i);
		if (on) cpush(&b, r.end - i < max ? r.end - i : max, r.d);
		/* walk down from the root, hanging the smaller suffixes under
		 * p1 and the larger under p0; len0/len1 are what the bounds
		 * on each side are known to share with i */
		uint32_t cur = head[h], *p0 = &son[2 * i + 1], *p1 = &son[2 * i], len0 = 0, len1 = 0, best = 2;
		head[h] = i;
		for (unsigned dd = 0;; dd++) {
			if (cur == NIL || i - cur > MAXD || dd >= depth) { *p0 = *p1 = NIL; break; }
			uint32_t len = len0 < len1 ? len0 : len1;
			len += mlen(in + i + len, in + cur + len, nice - len);
			if (len > best && !on) { best = len; cpush(&b, len, i - cur); }
			if (len >= nice) { *p1 = son[2 * cur]; *p0 = son[2 * cur + 1]; break; }
			if (in[cur + len] < in[i + len]) { *p1 = cur; p1 = &son[2 * cur + 1]; cur = *p1; len1 = len; }
			else { *p0 = cur; p0 = &son[2 * cur]; cur = *p0; len0 = len; }
		}
		if (!on && best >= NICE) {
			/* the tree compares up to NICE: measure the long one fully */
			struct cand *c = &b.c[b.k - 1];
			c->len = NICE + mlen(in + i + NICE, in + i - c->dist + NICE, max - NICE);
			r = (struct run){ i + c->len, c->dist };
		}
	}
	m->off[n] = (uint32_t)b.k;
	m->c = b.c ? b.c : malloc(sizeof(struct cand));
	free(head); free(son);
}

static void find_matches(const uint8_t *in, uint32_t n, unsigned depth, unsigned threads, int tree,
			 struct mf *m)
{
	if (tree) { find_matches_tree(in, n, depth, m); return; }
	enum { HB = 18 };
	uint32_t *head = malloc(sizeof(uint32_t) << HB), *prv = malloc(4 * (size_t)n + 4);
	memset(head, 0xff, sizeof(uint32_t) << HB);
	for (uint32_t i = 0; i + 3 <= n; i++) {
		uint32_t h = ((in[i] | in[i + 1] << 8 | in[i + 2] << 16) * 2654435761u) >> (32 - HB);
		prv[i] = head[h]; head[h] = i;
	}
	free(head);
	/* segments of 16 KiB or more, a few per thread for balance */
	enum { MAXSEG = 64 };
	unsigned nt = threads < 1 ? 1 : threads, ns = n / 16384 ? n / 16384 : 1;
	ns = nt == 1 ? 1 : ns < 4 * nt ? ns : 4 * nt;
	ns = ns < MAXSEG ? ns : MAXSEG;
	nt = nt < ns ? nt : ns;
	struct mfseg g[MAXSEG];
	pthread_t tid[MAXSEG];
	for (unsigned t = 0; t < ns; t++) {
		g[t] = (struct mfseg){ in, prv, n, (uint32_t)((uint64_t)n * t / ns),
				       (uint32_t)((uint64_t)n * (t + 1) / ns), depth, { NULL, 0, 0 }, NULL, NULL, 0 };
		g[t].off = malloc(4 * ((size_t)(g[t].b - g[t].a) + 1));
	}
	struct mfq q = { g, ns, 0 };
	for (unsigned t = 1; t < nt; t++) pthread_create(&tid[t], NULL, mf_work, &q);
	mf_work(&q);
	for (unsigned t = 1; t < nt; t++) pthread_join(tid[t], NULL);

	/* stitch: redo each segment's start in the state the one before ended in */
	struct cbuf fix[MAXSEG] = { { NULL, 0, 0 } };
	uint32_t conv[MAXSEG], *foff = malloc(4 * ((size_t)n + 1));
	struct run r = { 0, 0 };
	size_t total = 0;
	for (unsigned t = 0; t < ns; t++) {
		uint32_t i = g[t].a;
		size_t e = 0;
		struct run own = { 0, 0 };   /* the segment's state before i */
		for (; i < g[t].b; i++) {
			while (e < g[t].nev && g[t].ev[e].i < i) own = g[t].ev[e++].r;
			if (run_on(r, i) ? run_on(own, i) && own.end == r.end && own.d == r.d : !run_on(own, i))
				break;
			foff[i] = (uint32_t)fix[t].k;
			mf_pos(in, n, prv, depth, i, &r, &fix[t]);
		}
		conv[t] = i;
		if (i < g[t].b) {
			/* agreed from i on: the state at the end is the segment's */
			r = (struct run){ 0, 0 };
			for (e = 0; e < g[t].nev; e++) r = g[t].ev[e].r;
		}
		total += fix[t].k + (g[t].c.k - g[t].off[i - g[t].a]);
	}
	m->c = malloc((total + 1) * sizeof(struct cand));
	m->off = malloc(4 * ((size_t)n + 1));
	size_t k = 0;
	for (unsigned t = 0; t < ns; t++) {
		for (uint32_t i = g[t].a; i < conv[t]; i++) m->off[i] = (uint32_t)(k + foff[i]);
		if (fix[t].k) memcpy(m->c + k, fix[t].c, fix[t].k * sizeof(struct cand));
		k += fix[t].k;
		uint32_t o0 = g[t].off[conv[t] - g[t].a];
		for (uint32_t i = conv[t]; i < g[t].b; i++) m->off[i] = (uint32_t)(k + g[t].off[i - g[t].a] - o0);
		if (g[t].c.k > o0) memcpy(m->c + k, g[t].c.c + o0, (g[t].c.k - o0) * sizeof(struct cand));
		k += g[t].c.k - o0;
		free(fix[t].c); free(g[t].c.c); free(g[t].off); free(g[t].ev);
	}
	m->off[n] = (uint32_t)k;
	free(foff); free(prv);
}

/* literal table index at position pos; plit holds the last two literals.
 * FHL's context is the literal two back: each literal stream then only
 * depends on itself. */
static inline unsigned lctx(const struct model *md, const uint8_t *in, uint32_t pos, int first,
			    uint32_t rep0, unsigned plit)
{
	if (!md->legacy) return md->map[plit >> 8];
	if (first && md->Kf) return md->K + md->mapf[in[pos - rep0]];
	return md->map[pos ? in[pos - 1] : 0];
}

/* Match lengths the parse tries: every one up to 64, then the bucket starts
 * and l itself. */
static inline uint32_t next_len(uint32_t x, uint32_t l)
{
	if (x < 64 && x < l) return x + 1;
	if (x >= l) return l + 1;
	unsigned mc = ml_code(x);
	uint32_t nb = mc < 15 ? ml_base[mc + 1] : l;
	return nb < l ? nb : l;
}
/* The parse relaxes edges into cost[], from[] and how[] only: how[] is the
 * candidate (index into the match finder's list, or a rep, or a literal),
 * and the length is the distance travelled. The state a position was
 * reached with (reps, literal run, last literals) follows from its best edge
 * and is filled in when the parse gets there. */
#define HOW_LIT 0xffffffffu
#define HOW_REP 0xfffffffcu          /* + r */
#define INF 0x7fffffff
struct pst { uint32_t rep[3], lr; uint16_t plit; uint8_t first; };

/* parse() scratch, kept across passes */
struct pws { int32_t *cost; uint32_t *from, *how, *path; struct pst *st; };
static void pws_alloc(struct pws *w, uint32_t n)
{
	size_t sz = (size_t)n + 1;
	w->cost = malloc(4 * sz); w->from = malloc(4 * sz); w->how = malloc(4 * sz);
	w->path = malloc(4 * sz); w->st = malloc(sizeof(*w->st) * sz);
}
static void pws_free(struct pws *w)
{
	free(w->cost); free(w->from); free(w->how); free(w->path); free(w->st);
}

static inline void reach(struct pws *w, uint32_t z, int32_t t, uint32_t i, uint32_t how)
{
	if (t < w->cost[z]) { w->cost[z] = t; w->from[z] = i; w->how[z] = how; }
}

/* Lengths x0..x1 (at most 64) of one candidate from i: price lp[x] + k.
 * Each length reaches its own position, so four at a time. */
typedef int32_t v4i __attribute__((vector_size(16)));
static inline v4i ld4(const void *p) { v4i v; memcpy(&v, p, 16); return v; }
static inline void st4(void *p, v4i v) { memcpy(p, &v, 16); }
static inline void reach_run(struct pws *w, uint32_t i, const int32_t *lp, int32_t k, uint32_t x0,
			     uint32_t x1, uint32_t how)
{
	uint32_t x = x0;
	int32_t *cost = w->cost + i;
	const v4i vk = { k, k, k, k }, vi = { (int32_t)i, (int32_t)i, (int32_t)i, (int32_t)i },
		  vh = { (int32_t)how, (int32_t)how, (int32_t)how, (int32_t)how };
	for (; x + 3 <= x1; x += 4) {
		v4i t = ld4(lp + x) + vk, o = ld4(cost + x), b = t < o;
		if (!(b[0] | b[1] | b[2] | b[3])) continue;
		st4(cost + x, (t & b) | (o & ~b));
		st4(w->from + i + x, (vi & b) | (ld4(w->from + i + x) & ~b));
		st4(w->how + i + x, (vh & b) | (ld4(w->how + i + x) & ~b));
	}
	for (; x <= x1; x++) reach(w, i + x, lp[x] + k, i, how);
}

/* the price of length x > 64 after literal-run code llc */
static inline int32_t lprice(const struct cost *c, unsigned llc, uint32_t x)
{
	unsigned mc = ml_code(x);
	return (int32_t)(c->cmd[llc * 16 + mc] + ml_bits[mc] * 16);
}

static size_t parse(const uint8_t *in, uint32_t n, const struct mf *m, const struct model *md,
		    const struct cost *c, struct pws *w, struct seq *out)
{
	int32_t lp[16][68];
	for (unsigned a = 0; a < 16; a++)
		for (uint32_t x = MIN_MATCH; x <= 64; x++)
			lp[a][x] = (int32_t)(c->cmd[a * 16 + ml_code(x)] + ml_bits[ml_code(x)] * 16);
	struct pst *st = w->st;
	for (uint32_t i = 0; i <= n; i++) w->cost[i] = INF;
	w->cost[0] = 0;
	st[0] = (struct pst){ .rep = { 1, 1, 1 } };
	for (uint32_t i = 0; i < n; i++) {
		struct pst *a = &st[i];
		if (i) {
			const struct pst *p = &st[w->from[i]];
			uint32_t how = w->how[i];
			if (how == HOW_LIT) {
				*a = *p; a->lr = p->lr + 1;
				a->plit = (uint16_t)(p->plit << 8 | in[i - 1]);
			} else {
				/* rep0 keeps the reps, rep1 swaps the first two, the
				 * rest push the distance in front */
				uint32_t d = how >= HOW_REP ? p->rep[how - HOW_REP] : m->c[how].dist;
				a->lr = 0; a->first = 1; a->plit = p->plit;
				a->rep[0] = d; a->rep[1] = p->rep[how == HOW_REP ? 1 : 0];
				a->rep[2] = p->rep[how == HOW_REP + 1 ? 2 : how == HOW_REP ? 2 : 1];
			}
		}
		int32_t base = w->cost[i];
		unsigned llc = ll_code(a->lr);
		uint32_t lc = c->lit[lctx(md, in, i, a->first && a->lr == 0, a->rep[0], a->plit)][in[i]];
		reach(w, i + 1, base + (int32_t)lc, i, HOW_LIT);
		uint32_t max = n - i < MAXM ? n - i : MAXM, longest = 0;
		int32_t cmdbase = base + ll_bits[llc] * 16;
		for (unsigned r = 0; r < 3; r++) {
			uint32_t d = a->rep[r];
			if (d > i) continue;
			uint32_t l = mlen(in + i, in + i - d, max);
			longest = l > longest ? l : longest;
			if (l < MIN_MATCH) continue;
			/* every length up to 64, then just l */
			int32_t k = cmdbase + (int32_t)c->dist[r];
			reach_run(w, i, lp[llc], k, MIN_MATCH, l < 64 ? l : 64, HOW_REP + r);
			if (l > 64) reach(w, i + l, lprice(c, llc, l) + k, i, HOW_REP + r);
		}
		uint32_t lo = MIN_MATCH + 1;
		for (uint32_t k = m->off[i]; k < m->off[i + 1]; k++) {
			uint32_t d = m->c[k].dist, l = m->c[k].len;
			longest = l > longest ? l : longest;
			if (d == a->rep[0] || d == a->rep[1] || d == a->rep[2]) { lo = l + 1; continue; }
			unsigned sl = dist_slot(d - 1);
			int32_t dk = cmdbase + (int32_t)(c->dist[3 + sl] + (sl < 4 ? 0 : ((sl >> 1) - 1) * 16));
			uint32_t x = lo;
			if (x <= 64) {
				reach_run(w, i, lp[llc], dk, x, l < 64 ? l : 64, k);
				x = next_len(64, l);
			} else if (x < l && x != ml_base[ml_code(x)]) {
				x = next_len(x, l);
			}
			/* past 64: the bucket starts and l itself */
			for (; x <= l; x = next_len(x, l)) reach(w, i + x, lprice(c, llc, x) + dk, i, k);
			lo = l + 1;
		}
		if (longest >= NICE)    /* take long matches greedily: jump over them */
			i += longest - 1;
	}
	uint32_t *path = w->path;
	size_t np = 0, k = 0;
	for (uint32_t i = n; i > 0; i = w->from[i]) path[np++] = i;
	uint32_t L = 0;
	for (size_t p = np; p-- > 0;) {
		uint32_t z = path[p], f = w->from[z], how = w->how[z];
		if (how == HOW_LIT) { L++; continue; }
		uint32_t d, ds;
		if (how >= HOW_REP) { ds = how - HOW_REP; d = st[f].rep[ds]; }
		else { d = m->c[how].dist; ds = 3 + dist_slot(d - 1); }
		out[k] = (struct seq){ L, z - f, ds, d };
		L = 0; k++;
	}
	out[k] = (struct seq){ L, 0, 0, 0 };
	return k + 1;
}

/* Walk a parse; callback-free: fills prev-byte and match-byte literal histograms. */
static void stats(const uint8_t *in, const struct seq *s, size_t ns, int legacy, uint32_t (*pf)[256],
		  uint32_t (*ff)[256], uint32_t *cf, uint32_t *df)
{
	uint32_t pos = 0, rep0 = 1;
	unsigned plit = 0;
	for (size_t i = 0; i < ns; i++) {
		cf[ll_code(s[i].L) * 16 + (s[i].M ? ml_code(s[i].M) : 0)]++;
		for (uint32_t j = 0; j < s[i].L; j++, pos++) {
			if (!legacy) pf[plit >> 8][in[pos]]++;
			else if (j == 0 && i > 0) ff[in[pos - rep0]][in[pos]]++;
			else pf[pos ? in[pos - 1] : 0][in[pos]]++;
			plit = (plit << 8 | in[pos]) & 0xffff;
		}
		if (!s[i].M) break;
		df[s[i].ds]++;
		rep0 = s[i].d;
		pos += s[i].M;
	}
}

/* c[k] = sum of f[x] * cl[x][k] over the literals z seen, for k < kk (a
 * constant per call site, so the k loop vectorizes) */
static inline __attribute__((always_inline)) void group_costs(double *c, const uint32_t *f, const uint8_t *z,
							      unsigned nz, const double (*cl)[MAXK], unsigned kk)
{
	for (unsigned i = 0; i < nz; i++) {
		double v = f[z[i]];
		for (unsigned k = 0; k < kk; k++) c[k] += v * cl[z[i]][k];
	}
}

static void cluster(uint32_t (*pf)[256], unsigned K, uint8_t *map)
{
	for (unsigned p = 0; p < 256; p++) map[p] = (uint8_t)((p * K) >> 8);
	if (K <= 1) { memset(map, 0, 256); return; }
	double (*sum)[256] = malloc(sizeof(double[MAXT][256]));
	double (*cl)[MAXK] = calloc(256, sizeof(*cl));    /* [literal][group] */
	uint8_t *nz = malloc(65536);                      /* each context's literals seen */
	unsigned nn[256];
	for (unsigned p = 0, q = 0; p < 256; q += nn[p++]) {
		nn[p] = 0;
		for (unsigned x = 0; x < 256; x++) { nz[q + nn[p]] = (uint8_t)x; nn[p] += pf[p][x] != 0; }
	}
	for (int it = 0; it < 30; it++) {
		memset(sum, 0, K * sizeof(*sum));
		const uint8_t *z = nz;
		for (unsigned p = 0; p < 256; z += nn[p++])
			for (unsigned i = 0; i < nn[p]; i++) sum[map[p]][z[i]] += pf[p][z[i]];
		for (unsigned k = 0; k < K; k++) {
			double t = 0;
			for (unsigned x = 0; x < 256; x++) t += sum[k][x] + 0.2;
			for (unsigned x = 0; x < 256; x++) cl[x][k] = -__builtin_log2((sum[k][x] + 0.2) / t);
		}
		int changed = 0;
		z = nz;
		for (unsigned p = 0; p < 256; z += nn[p++]) {
			/* all K costs at once, each summed in literal order */
			double c[MAXK] = { 0 };
			if (K <= 2) group_costs(c, pf[p], z, nn[p], cl, 2);
			else if (K <= 4) group_costs(c, pf[p], z, nn[p], cl, 4);
			else if (K <= 8) group_costs(c, pf[p], z, nn[p], cl, 8);
			else group_costs(c, pf[p], z, nn[p], cl, 16);
			double best = 1e300; unsigned bk = map[p];
			for (unsigned k = 0; k < K; k++) if (c[k] < best) { best = c[k]; bk = k; }
			if (bk != map[p]) { map[p] = (uint8_t)bk; changed = 1; }
		}
		if (!changed) break;
	}
	free(sum); free(cl); free(nz);
}

static void write_lengths(struct bw *w, const uint8_t *len, unsigned total)
{
	uint16_t *sy = malloc(2 * NLEN), *ex = malloc(2 * NLEN);
	unsigned ns = 0, prev = 0;
	for (unsigned i = 0; i < total;) {
		unsigned l = len[i], r = 1;
		while (i + r < total && len[i + r] == l) r++;
		if (l == 0 && r >= 3) {
			unsigned t = r > 138 ? 138 : r;
			if (t <= 10) { sy[ns] = 13; ex[ns++] = t - 3; } else { sy[ns] = 14; ex[ns++] = t - 11; }
			i += t; prev = 0; continue;
		}
		if (l == prev && l && r >= 3) {
			unsigned t = r > 6 ? 6 : r;
			sy[ns] = 12; ex[ns++] = t - 3; i += t; continue;
		}
		sy[ns] = (uint16_t)l; ex[ns++] = 0; prev = l; i++;
	}
	uint32_t mf[NMETA] = {0}, code[NMETA];
	for (unsigned i = 0; i < ns; i++) mf[sy[i]]++;
	uint8_t ml[NMETA];
	huff_len(mf, NMETA, 7, ml);
	for (unsigned i = 0; i < NMETA; i++) put(w, ml[i], 3);
	codes(ml, NMETA, 7, code);
	for (unsigned i = 0; i < ns; i++) {
		put(w, code[sy[i]], ml[sy[i]]);
		if (sy[i] == 12) put(w, ex[i], 2);
		else if (sy[i] == 13) put(w, ex[i], 3);
		else if (sy[i] == 14) put(w, ex[i], 7);
	}
	free(sy); free(ex);
}

/* Code lengths from a parse's statistics (see stats()) under a model. */
static void stat_lengths(const uint8_t *in, const struct seq *s, size_t ns, const struct model *md,
			 uint32_t (*pf)[256], uint32_t (*ff)[256], uint32_t *cf, uint32_t *df, uint8_t *len)
{
	uint32_t (*lf)[256] = calloc(MAXT, sizeof(*lf));
	for (unsigned q = 0; q < 256; q++)
		for (unsigned x = 0; x < 256; x++) {
			lf[md->map[q]][x] += pf[q][x];
			if (md->Kf) lf[md->K + md->mapf[q]][x] += ff[q][x];
			else lf[md->map[0]][x] += 0;   /* first literals then use the prev-byte map */
		}
	if (!md->Kf && md->legacy) {
		/* without Kf the first literal is coded by the prev-byte map: recount */
		uint32_t pos = 0;
		for (size_t i = 0; i < ns; i++) {
			if (s[i].L && i > 0) lf[md->map[in[pos - 1]]][in[pos]]++;
			pos += s[i].L + s[i].M;
		}
	}
	unsigned T = md->K + md->Kf;
	for (unsigned k = 0; k < T; k++) huff_len(lf[k], NLIT, MAXL, len + k * NLIT);
	huff_len(cf, NCMD, MAXL, len + T * NLIT);
	huff_len(df, NDIST, MAXL, len + T * NLIT + NCMD);
	free(lf);
}

/* Build code lengths for a parse under a model. */
static void model_lengths(const uint8_t *in, const struct seq *s, size_t ns, const struct model *md,
			  uint8_t *len)
{
	uint32_t (*pf)[256] = calloc(256, sizeof(*pf)), (*ff)[256] = calloc(256, sizeof(*ff));
	uint32_t cf[NCMD] = {0}, df[NDIST] = {0};
	stats(in, s, ns, md->legacy, pf, ff, cf, df);
	stat_lengths(in, s, ns, md, pf, ff, cf, df, len);
	free(pf); free(ff);
}

static void wr32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void flush(struct bw *w)
{
	if (w->cnt) put(w, 0, 8 - w->cnt);
	for (int i = 0; i < 8; i++) *w->p++ = 0;   /* refill slack */
}

static size_t emit(const uint8_t *in, uint32_t n, const struct seq *s, size_t ns,
		   const struct model *md, uint8_t *out)
{
	uint8_t *len = malloc(NLEN), *sb[3];
	uint32_t *code = malloc(4 * NLEN);
	for (int q = 0; q < 3; q++) sb[q] = malloc((size_t)n * 2 + 1024);
	unsigned K = md->K, kb = log2k(K);
	model_lengths(in, s, ns, md, len);
	uint32_t nl = 0, nf = 0;
	for (size_t i = 0; i < ns; i++) nl += s[i].L;
	/* nfast: leading commands whose literal over-copy (16 bytes at o and at
	 * the literal read point lr) and match over-copy (16 bytes from o) stay
	 * clear of the buffer end and of unread literals */
	for (uint64_t i = 0, o = 0, lr = n - nl; i + 1 < ns; i++, nf++) {
		if (lr - o < 32 || n - lr < 16) break;
		o += s[i].L; lr += s[i].L;
		if (!s[i].M || lr < o + s[i].M + 16) break;
		o += s[i].M;
	}
	wr32(out, n); out[4] = (uint8_t)K; wr32(out + 5, nl); wr32(out + 9, nf);
	struct bw w = { out + HDR, 0, 0 }, ws[3] = { { sb[0], 0, 0 }, { sb[1], 0, 0 }, { sb[2], 0, 0 } };
	for (unsigned i = 0; i < 256; i++) put(&w, md->map[i], kb);
	write_lengths(&w, len, K * NLIT + NCMD + NDIST);
	for (unsigned k = 0; k < K; k++) codes(len + k * NLIT, NLIT, MAXL, code + k * NLIT);
	codes(len + K * NLIT, NCMD, MAXL, code + K * NLIT);
	codes(len + K * NLIT + NCMD, NDIST, MAXL, code + K * NLIT + NCMD);
	const uint8_t *cl = len + K * NLIT, *dl = cl + NCMD;
	const uint32_t *cc = code + K * NLIT, *dc = cc + NCMD;
	uint32_t pos = 0, litn = 0;
	unsigned plit = 0;
	for (size_t i = 0; i < ns; i++) {
		unsigned llc = ll_code(s[i].L), mlc = s[i].M ? ml_code(s[i].M) : 0;
		unsigned c = llc * 16 + mlc;
		put(&w, cc[c], cl[c]);
		put(&w, s[i].L - ll_base[llc], ll_bits[llc]);
		for (uint32_t j = 0; j < s[i].L; j++, pos++) {
			unsigned k = lctx(md, in, pos, 0, 0, plit);
			put(&ws[litn++ & 1], code[k * NLIT + in[pos]], len[k * NLIT + in[pos]]);
			plit = (plit << 8 | in[pos]) & 0xffff;
		}
		if (!s[i].M) break;
		put(&w, s[i].M - ml_base[mlc], ml_bits[mlc]);
		put(&ws[2], dc[s[i].ds], dl[s[i].ds]);
		if (s[i].ds >= 7) {
			unsigned sl = s[i].ds - 3, nb = (sl >> 1) - 1;
			put(&ws[2], (s[i].d - 1) - ((2u | (sl & 1)) << nb), nb);
		}
		pos += s[i].M;
	}
	if (getenv("FHL_STATS")) {
		size_t bl = 0, nm = 0, nr = 0;
		for (int q = 0; q < 2; q++) bl += (size_t)(ws[q].p - sb[q]);
		for (size_t i = 0; i < ns && s[i].M; i++) { nm++; nr += s[i].ds < 3; }
		fprintf(stderr, "K=%u hdr+tables+cmds=%zu lit=%zu(%u, %.2fb) dist=%zu cmds=%zu reps=%zu\n",
			K, (size_t)(w.p - out), bl, nl, nl ? 8.0 * bl / nl : 0, (size_t)(ws[2].p - sb[2]), nm, nr);
	}
	flush(&w);
	for (int q = 0; q < 3; q++) {
		wr32(out + 13 + 4 * q, (uint32_t)(w.p - out));
		flush(&ws[q]);
		memcpy(w.p, sb[q], (size_t)(ws[q].p - sb[q]));
		w.p += ws[q].p - sb[q];
		free(sb[q]);
	}
	for (int i = 0; i < 16; i++) *w.p++ = 0;   /* the decoder's look-ahead words */
	free(len); free(code);
	return (size_t)(w.p - out);
}

static void costs_from(const uint8_t *in, const struct seq *s, size_t ns, const struct model *md,
		       uint32_t (*pf)[256], uint32_t (*ff)[256], uint32_t *cf, uint32_t *df, struct cost *c)
{
	uint8_t *len = malloc(NLEN);
	unsigned T = md->K + md->Kf;
	stat_lengths(in, s, ns, md, pf, ff, cf, df, len);
	for (unsigned k = 0; k < T; k++) lens_to_cost(len + k * NLIT, NLIT, c->lit[k], 13);
	lens_to_cost(len + T * NLIT, NCMD, c->cmd, 13);
	lens_to_cost(len + T * NLIT + NCMD, NDIST, c->dist, 13);
	free(len);
}

/* Literal histograms of one parse: [previous byte or match byte][literal].
 * They only depend on the parse, so one parse can be re-clustered and
 * re-priced for every K instead of being redone for each. */
struct hist { uint32_t (*pf)[256], (*ff)[256]; };

static void hist_alloc(struct hist *h)
{
	h->pf = calloc(256, sizeof(*h->pf));
	h->ff = calloc(256, sizeof(*h->ff));
}
static void hist_free(struct hist *h) { free(h->pf); free(h->ff); }

/* Histograms of parse s into h, clustered into md's maps; prices into c. */
static void reprice(const uint8_t *in, const struct seq *s, size_t ns, struct model *md,
		    struct hist *h, struct cost *c)
{
	uint32_t cf[NCMD] = {0}, df[NDIST] = {0};
	memset(h->pf, 0, 256 * sizeof(*h->pf)); memset(h->ff, 0, 256 * sizeof(*h->ff));
	stats(in, s, ns, md->legacy, h->pf, h->ff, cf, df);
	cluster(h->pf, md->K, md->map);
	cluster(h->ff, md->Kf, md->mapf);
	costs_from(in, s, ns, md, h->pf, h->ff, cf, df, c);
}

/* The first parse: flat prices, so it is the same for every model. */
static size_t parse_flat(const uint8_t *in, uint32_t n, const struct mf *m, struct pws *w,
			 struct seq *s)
{
	struct model md = { .K = 1 };
	struct cost *c = malloc(sizeof(*c));
	for (unsigned k = 0; k < MAXT; k++) for (unsigned x = 0; x < 256; x++) c->lit[k][x] = 8 * 16;
	for (unsigned x = 0; x < 256; x++) c->cmd[x] = 6 * 16;
	for (unsigned x = 0; x < NDIST; x++) c->dist[x] = 5 * 16;
	size_t ns = parse(in, n, m, &md, c, w, s);
	free(c);
	return ns;
}

/* For -g, a lazy greedy parse instead: the longest match (a rep if it is
 * nearly as long) unless the next position has a longer one. Cheaper than
 * the flat-priced parse, and only seeds the prices. */
static size_t parse_greedy(const uint8_t *in, uint32_t n, const struct mf *m, struct seq *s)
{
	uint32_t rep[3] = { 1, 1, 1 }, L = 0;
	size_t k = 0;
	for (uint32_t i = 0; i < n;) {
		uint32_t max = n - i < MAXM ? n - i : MAXM, bl = 0, bd = 0, br = 3;
		for (unsigned r = 0; r < 3; r++) {
			if (rep[r] > i) continue;
			uint32_t l = mlen(in + i, in + i - rep[r], max);
			if (l > bl) { bl = l; br = r; }
		}
		uint32_t o = m->off[i + 1];
		if (o > m->off[i] && m->c[o - 1].len > bl + 1) { bl = m->c[o - 1].len; bd = m->c[o - 1].dist; br = 3; }
		if (bl < (br < 3 ? 2u : 3u) ||
		    (br == 3 && i + 1 < n && m->off[i + 2] > m->off[i + 1] && m->c[m->off[i + 2] - 1].len > bl + 1)) {
			L++; i++;
			continue;
		}
		uint32_t d = br < 3 ? rep[br] : bd;
		s[k++] = (struct seq){ L, bl, br < 3 ? br : 3 + dist_slot(d - 1), d };
		if (br == 1) { rep[1] = rep[0]; rep[0] = d; }
		else if (br != 0) { rep[2] = rep[1]; rep[1] = rep[0]; rep[0] = d; }
		L = 0; i += bl;
	}
	s[k] = (struct seq){ L, 0, 0, 0 };
	return k + 1;
}

/* 'passes' rounds of statistics -> prices -> parse, starting from parse s
 * (ns commands), which is replaced by the final one; returns its length. h
 * is left with the histograms that priced it. */
static size_t refine(const uint8_t *in, uint32_t n, const struct mf *m, struct model *md,
		     int passes, struct pws *w, struct seq *s, size_t ns, struct hist *h)
{
	struct cost *c = calloc(1, sizeof(*c));
	for (int p = 0; p < passes; p++) {
		reprice(in, s, ns, md, h, c);
		ns = parse(in, n, m, md, c, w, s);
	}
	free(c);
	return ns;
}

/* One K of compress(), run on its own thread, from the flat parse s0 shared
 * by all K: with -f the parse is shared too and only the literal maps are
 * per K; otherwise this K gets its own priced parse (a few hundred bytes
 * smaller, a parse more expensive). */
struct job {
	const uint8_t *in;
	uint32_t n;
	const struct mf *m;
	const struct seq *s0;           /* flat parse, or with -f the final one */
	size_t ns0;
	const struct hist *h;           /* -f: the shared parse's histograms */
	const uint8_t *map;             /* and the K's map, if already clustered */
	unsigned K;
	int passes;
	struct pws *w;                  /* parse scratch and */
	struct seq *buf;                /* parse of this K, shared by the K's run one after another */
	uint8_t *out;
	size_t size;
};
static void *run_job(void *arg)
{
	struct job *j = arg;
	struct model md = { .K = j->K };
	const struct seq *s = j->s0;
	size_t ns = j->ns0;
	if (j->map) {
		memcpy(md.map, j->map, 256);
	} else if (j->h) {
		cluster(j->h->pf, md.K, md.map);
	} else {
		struct hist h;
		hist_alloc(&h);
		memcpy(j->buf, s, ns * sizeof(*s));
		ns = refine(j->in, j->n, j->m, &md, j->passes, j->w, j->buf, ns, &h);
		s = j->buf;
		hist_free(&h);
	}
	j->out = malloc((size_t)j->n * 2 + 65536);
	j->size = emit(j->in, j->n, s, ns, &md, j->out);
	return NULL;
}

/* Bits of the literals and their code lengths with K tables clustered from
 * the histograms pf, leaving the maps in md: -f picks its K by this. */
static uint64_t lit_bits(uint32_t (*pf)[256], struct model *md)
{
	cluster(pf, md->K, md->map);
	uint32_t (*lf)[256] = calloc(MAXK, sizeof(*lf));
	for (unsigned q = 0; q < 256; q++)
		for (unsigned x = 0; x < 256; x++) lf[md->map[q]][x] += pf[q][x];
	uint8_t *len = calloc(1, NLEN), *buf = malloc(MAXK * NLIT * 2 + 64);
	uint64_t bits = 256 * log2k(md->K);
	for (unsigned k = 0; k < md->K; k++) {
		huff_len(lf[k], NLIT, MAXL, len + k * NLIT);
		for (unsigned x = 0; x < NLIT; x++) bits += (uint64_t)lf[k][x] * len[k * NLIT + x];
	}
	struct bw w = { buf, 0, 0 };
	write_lengths(&w, len, md->K * NLIT);
	bits += 8 * (uint64_t)(w.p - buf) + w.cnt;
	free(lf); free(len); free(buf);
	return bits;
}

struct copts {
	int K, passes;                  /* K 0: try 1..16 */
	unsigned depth, threads;
	int fast, tree, greedy;         /* -f, -t, -g */
};

/* Smallest stream over K = 1..16 (or just o->K), up to o->threads K at a
 * time; ties go to the smaller K. With o->fast one parse serves every K
 * and only the K with the fewest literal bits is written. Returns the size
 * of the stream. */
__attribute__((unused))
static size_t compress(const uint8_t *in, uint32_t n, uint8_t *out, const struct copts *o)
{
	int Kreq = o->K, passes = o->passes, fast = o->fast;
	unsigned threads = o->threads;
	struct mf m;
	find_matches(in, n, o->depth, o->threads, o->tree, &m);
	struct model md = { .K = Kreq > 0 ? (unsigned)Kreq : PARSE_K };
	struct seq *s = malloc(sizeof(*s) * ((size_t)n + 2));
	struct pws w;
	pws_alloc(&w, n);
	size_t ns = o->greedy ? parse_greedy(in, n, &m, s) : parse_flat(in, n, &m, &w, s);
	struct hist h = { NULL, NULL };
	uint8_t map[256];
	if (fast) {
		/* cluster every K from the histograms of the final parse */
		hist_alloc(&h);
		ns = refine(in, n, &m, &md, passes, &w, s, ns, &h);
		uint32_t cf[NCMD] = {0}, df[NDIST] = {0};
		memset(h.pf, 0, 256 * sizeof(*h.pf));
		stats(in, s, ns, 0, h.pf, h.ff, cf, df);
		uint64_t bb = ~0ull;
		for (unsigned K = 1; K <= MAXK && o->K <= 0; K *= 2) {
			struct model mk = { .K = K };
			uint64_t e = lit_bits(h.pf, &mk);
			if (e < bb) { bb = e; Kreq = (int)K; memcpy(map, mk.map, 256); }
		}
	}
	struct job jobs[5];
	pthread_t tid[5];
	struct pws ws[5];
	struct seq *buf[5] = { NULL };
	unsigned nj = 0;
	for (unsigned K = 1; K <= MAXK; K *= 2)
		if (Kreq <= 0 || K == (unsigned)Kreq)
			jobs[nj++] = (struct job){ in, n, &m, s, ns, fast ? &h : NULL, fast && o->K <= 0 ? map : NULL,
						   K, passes, NULL, NULL, NULL, 0 };
	unsigned nw = fast ? 0 : threads < nj ? threads : nj;
	ws[0] = w;
	for (unsigned k = 0; k < nw; k++) {
		if (k) pws_alloc(&ws[k], n);
		buf[k] = malloc(sizeof(*buf[k]) * ((size_t)n + 2));
	}
	for (unsigned k = 0; k < nj && nw; k++) { jobs[k].w = &ws[k % nw]; jobs[k].buf = buf[k % nw]; }
	for (unsigned a = 0; a < nj; a += threads) {
		unsigned b = a + threads < nj ? a + threads : nj;
		for (unsigned k = a + 1; k < b; k++) pthread_create(&tid[k], NULL, run_job, &jobs[k]);
		run_job(&jobs[a]);
		for (unsigned k = a + 1; k < b; k++) pthread_join(tid[k], NULL);
	}
	size_t best = 0;
	for (unsigned k = 1; k < nj; k++) best = jobs[k].size < jobs[best].size ? k : best;
	size_t sz = jobs[best].size;
	memcpy(out, jobs[best].out, sz);
	for (unsigned k = 0; k < nj; k++) free(jobs[k].out);
	for (unsigned k = 0; k < nw; k++) { if (k) pws_free(&ws[k]); free(buf[k]); }
	free(s); hist_free(&h); pws_free(&w); free(m.c); free(m.off);
	return sz;
}

/* The parser for ANZ (github.com/ArthurHeymans/anz). fhl_matches() runs the
 * match finder and the flat-priced first parse once; fhl_parse() is then
 * thread-safe and gives an FHL parse priced with ANZ's literal context: K
 * previous-byte tables and Kf match-byte tables, four passes. out needs
 * n + 2 entries; returns the number of commands, the last of which has
 * M == 0 (literals only). */
struct fhl_mf { struct mf m; struct seq *flat; size_t nflat; };
struct fhl_mf *fhl_matches_mt(const uint8_t *in, uint32_t n, unsigned depth, unsigned threads,
			      unsigned flags)
{
	struct fhl_mf *f = malloc(sizeof(*f));
	struct pws w;
	find_matches(in, n, depth, threads, flags & FHL_MF_TREE, &f->m);
	pws_alloc(&w, n);
	f->flat = malloc(sizeof(*f->flat) * ((size_t)n + 2));
	f->nflat = flags & FHL_MF_GREEDY ? parse_greedy(in, n, &f->m, f->flat) : parse_flat(in, n, &f->m, &w, f->flat);
	pws_free(&w);
	return f;
}
struct fhl_mf *fhl_matches(const uint8_t *in, uint32_t n)
{
	return fhl_matches_mt(in, n, 256, 1, 0);
}
void fhl_matches_free(struct fhl_mf *f)
{
	free(f->m.c); free(f->m.off); free(f->flat); free(f);
}
size_t fhl_parse_passes(const uint8_t *in, uint32_t n, const struct fhl_mf *f, unsigned K,
			unsigned Kf, int passes, struct fhl_seq *out)
{
	struct model md = { .K = K, .Kf = Kf, .legacy = 1 };
	struct hist h;
	struct pws w;
	hist_alloc(&h);
	pws_alloc(&w, n);
	memcpy(out, f->flat, f->nflat * sizeof(*out));
	size_t ns = refine(in, n, &f->m, &md, passes, &w, out, f->nflat, &h);
	hist_free(&h);
	pws_free(&w);
	return ns;
}
size_t fhl_parse(const uint8_t *in, uint32_t n, const struct fhl_mf *f, unsigned K, unsigned Kf,
		 struct fhl_seq *out)
{
	return fhl_parse_passes(in, n, f, K, Kf, 4, out);
}

#ifndef FHL_LIB
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
	struct copts o = { .K = 0, .passes = 1, .depth = 256, .threads = 5, .tree = 1 };
	int a = 2;
	while (a < argc && argv[a][0] == '-') {
		if (!strcmp(argv[a], "-f")) { o.fast = 1; a++; continue; }
		if (!strcmp(argv[a], "-t")) { o.tree = 1; a++; continue; }
		if (!strcmp(argv[a], "-c")) { o.tree = 0; a++; continue; }
		if (!strcmp(argv[a], "-g")) { o.greedy = 1; a++; continue; }
		if (a + 1 >= argc) break;
		if (!strcmp(argv[a], "-K")) o.K = atoi(argv[++a]);
		else if (!strcmp(argv[a], "-p")) o.passes = atoi(argv[++a]);
		else if (!strcmp(argv[a], "-d")) o.depth = (unsigned)atoi(argv[++a]);
		else if (!strcmp(argv[a], "-j")) o.threads = (unsigned)atoi(argv[++a]);
		a++;
	}
	if (argc - a != 2 || (argv[1][0] != 'c' && argv[1][0] != 'd') ||
	    (o.K && (o.K > MAXK || (o.K & (o.K - 1)))) || o.threads < 1) {
		fprintf(stderr, "usage: fhl c [-K 1|2|4|8|16] [-p passes] [-d depth] [-c] [-g] [-f] [-j threads] in out\n"
				"       fhl d in out\n"
				"  -c  hash-chain match finder on -j threads instead of the serial\n"
				"      binary tree (the earlier default; slightly larger streams)\n"
				"  -g  seed the prices with a greedy parse instead of a flat-priced one\n"
				"  -f  price every K from one parse and write the most promising\n"
				"  -g -f together: about 2x faster for about 0.2%% larger\n");
		return 1;
	}
	size_t n, on;
	uint8_t *in = slurp(argv[a], &n), *out;
	static struct dtab t;
	if (argv[1][0] == 'c') {
		if (n > 0xffffffffu) { fprintf(stderr, "%s: too large\n", argv[a]); return 1; }
		out = malloc(n * 2 + 65536);
		on = compress(in, (uint32_t)n, out, &o);
		uint8_t *chk = malloc(n + 1);
		if (fhl_decode(out, on, chk, n, &t) != n || memcmp(chk, in, n)) {
			fprintf(stderr, "%s: verify failed\n", argv[a]);
			return 2;
		}
		free(chk);
	} else {
		size_t size = n >= HDR ? rd32le(in) : 0;
		out = malloc(size + 1);
		on = fhl_decode(in, n, out, size, &t);
		if (on != size || n < HDR) { fprintf(stderr, "%s: not an FHL stream\n", argv[a]); return 2; }
	}
	FILE *f = fopen(argv[a + 1], "wb");
	if (!f || fwrite(out, 1, on, f) != on) { perror(argv[a + 1]); return 1; }
	fclose(f);
	free(in); free(out);
	return 0;
}
#endif
#endif
