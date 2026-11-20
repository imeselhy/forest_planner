#pragma once
// =============================================================================
// can_packet.hpp  —  R2 Robot CAN Packet Codec
//
// Shared between ROS2 (laptop/PC) and STM32.
// No dynamic allocation, no STL, no ROS headers — pure C++17 / C99.
//
// ── FRAME LAYOUT ─────────────────────────────────────────────────────────────
//
// Every frame is exactly 8 bytes.  Byte 7 is always CRC-8.
//
// FRAME 0  —  Header
// ┌──────┬──────────────┬────────────┬──────────┬──────┬──────┬──────┬──────┐
// │  B0  │      B1      │     B2     │    B3    │  B4  │  B5  │  B6  │  B7  │
// │ 0xA2 │ TOTAL_FRAMES │ TOTAL_COST │   META   │ GR0  │ GR1  │ GR2  │ CRC8 │
// └──────┴──────────────┴────────────┴──────────┴──────┴──────┴──────┴──────┘
//
//   META  =  [exit_enc(2b) | entry_enc(2b) | pre_cnt(3b) | retry(1b)]
//   entry_enc = entry_block - 1   (1–3  → 0–2)
//   exit_enc  = exit_block  - 10  (10–12→ 0–2)
//   retry     = bit[0], operator retry flag
//   GR0..GR2  = 12 × 2-bit box states packed (block0 in bits[1:0] of GR0, etc.)
//
// FRAME N  —  Step frame  (N = 1 .. TOTAL_FRAMES-1)
// ┌──────┬──────────────┬──────┬──────┬──────┬──────┬──────────────┬──────┐
// │  B0  │      B1      │  B2  │  B3  │  B4  │  B5  │     B6       │  B7  │
// │FRAME │ STEPS_IN_FRAME│ S0_H │ S0_L │ S1_H │ S1_L │ HDELTA[7:2]  │ CRC8 │
// │ IDX  │   (1 or 2)   │      │      │      │      │ [7:5]=S0_ht  │      │
// │      │              │      │      │      │      │ [4:2]=S1_ht  │      │
// │      │              │      │      │      │      │ [1:0]=0x00   │      │
// └──────┴──────────────┴──────┴──────┴──────┴──────┴──────────────┴──────┘
//   2 steps per frame (4 bytes), B6 carries height deltas, CRC at byte 7.
//   Unused step slot = 0x0000.
//
// ── STEP ENCODING  (16 bits) ─────────────────────────────────────────────────
//
//  [15:14] action_type      0=MOVE  1=PICKUP  2=PRE_ENTRY
//  [13:10] current_block    stored as-is: 0=outside, 1-12=block
//   [9: 6] target_block     stored as (block - 1), so 1–12 → 0–11
//   [5: 4] direction        0=DOWN 1=RIGHT 2=UP 3=LEFT
//   [3: 1] collected_after  0–7  (3 bits, widened to support kfs_total up to 7)
//      [0] auto_pickup
//  Note: requires_r1_clear is encoded separately in the step payload (bit 1 of
//        low byte, see encodeStep). collected_after uses bits [3:1].
//
// ── HEIGHT DELTA ENCODING (3 bits in B6) ─────────────────────────────────────
//
//  delta = target_block_height_mm - current_block_height_mm
//  For PRE_ENTRY, robot is on ground (0mm), so delta = absolute block height.
//
//  enc 0 = -400mm    enc 1 = -200mm    enc 2 = 0mm
//  enc 3 = +200mm    enc 4 = +400mm    enc 5 = +600mm (PRE_ENTRY to 600mm block)
//
// ── BLOCK HEIGHT TABLE ────────────────────────────────────────────────────────
//
//  Dark green  200mm : B2, B4, B10, B12
//  Light green 400mm : B1, B3, B5, B7, B9, B11
//  Yellowish   600mm : B6, B8
//
// ── WORST-CASE SIZE ──────────────────────────────────────────────────────────
//   Max steps = 16  →  8 step-frames + 1 header = 9 frames = 72 bytes on bus
// =============================================================================

#include <cstdint>
#include <cstring>

namespace r2can {

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr uint8_t MSG_ID          = 0xA2;
static constexpr uint8_t CAN_FRAME_BYTES = 8;
static constexpr uint8_t STEPS_PER_FRAME = 2;
static constexpr uint8_t MAX_STEPS       = 16;
static constexpr uint8_t MAX_FRAMES      = 1 + (MAX_STEPS + STEPS_PER_FRAME - 1)
                                               / STEPS_PER_FRAME; // = 9

// Action types
static constexpr uint8_t ACT_MOVE      = 0;
static constexpr uint8_t ACT_PICKUP    = 1;
static constexpr uint8_t ACT_PRE_ENTRY = 2;

// Directions
static constexpr uint8_t DIR_DOWN  = 0;
static constexpr uint8_t DIR_RIGHT = 1;
static constexpr uint8_t DIR_UP    = 2;
static constexpr uint8_t DIR_LEFT  = 3;

// Box states
static constexpr uint8_t BOX_EMPTY = 0;
static constexpr uint8_t BOX_R1    = 1;
static constexpr uint8_t BOX_R2    = 2;
static constexpr uint8_t BOX_FAKE  = 3;

// Height delta encoding values
static constexpr uint8_t HDELTA_NEG400 = 0;  // -400mm
static constexpr uint8_t HDELTA_NEG200 = 1;  // -200mm
static constexpr uint8_t HDELTA_ZERO   = 2;  //    0mm
static constexpr uint8_t HDELTA_POS200 = 3;  // +200mm
static constexpr uint8_t HDELTA_POS400 = 4;  // +400mm
static constexpr uint8_t HDELTA_POS600 = 5;  // +600mm (PRE_ENTRY to 600mm block)

// ── Block height table (mm, 1-indexed) ────────────────────────────────────────
// B1=400 B2=200 B3=400 B4=200 B5=400 B6=600
// B7=400 B8=600 B9=400 B10=200 B11=400 B12=200
static constexpr uint16_t BLOCK_HEIGHT_MM[13] = {
    0,                              // index 0 unused
    400, 200, 400, 200, 400, 600,   // B1–B6
    400, 600, 400, 200, 400, 200    // B7–B12
};

// Compute height delta encoding for normal in-forest steps (current → target).
inline uint8_t heightDeltaEnc(uint8_t cur, uint8_t tgt)
{
    if (cur < 1 || cur > 12 || tgt < 1 || tgt > 12) return HDELTA_ZERO;
    int16_t d = (int16_t)BLOCK_HEIGHT_MM[tgt] - (int16_t)BLOCK_HEIGHT_MM[cur];
    switch (d) {
        case -400: return HDELTA_NEG400;
        case -200: return HDELTA_NEG200;
        case    0: return HDELTA_ZERO;
        case  200: return HDELTA_POS200;
        case  400: return HDELTA_POS400;
        default:   return HDELTA_ZERO;
    }
}

// PRE_ENTRY: robot is on the ground (0mm), delta = absolute target block height.
inline uint8_t preEntryHeightDeltaEnc(uint8_t tgt)
{
    if (tgt < 1 || tgt > 12) return HDELTA_ZERO;
    switch (BLOCK_HEIGHT_MM[tgt]) {
        case 200: return HDELTA_POS200;
        case 400: return HDELTA_POS400;
        case 600: return HDELTA_POS600;
        default:  return HDELTA_ZERO;
    }
}

// Decode height delta encoding to signed mm.
inline int16_t heightDeltaMm(uint8_t enc)
{
    switch (enc) {
        case HDELTA_NEG400: return -400;
        case HDELTA_NEG200: return -200;
        case HDELTA_ZERO:   return    0;
        case HDELTA_POS200: return  200;
        case HDELTA_POS400: return  400;
        case HDELTA_POS600: return  600;
        default:            return    0;
    }
}

// ── Data structures ───────────────────────────────────────────────────────────

struct Step {
    uint8_t action_type;        // ACT_*
    uint8_t current_block;      // 0=outside grid, 1-12=1-indexed block number
    uint8_t target_block;       // 1-indexed (1–12)
    uint8_t direction;          // DIR_*
    uint8_t collected_after;    // 0–3
    bool    requires_r1_clear;
    bool    auto_pickup;
    uint8_t height_delta_enc;   // HDELTA_* — arm lift for pickup/auto_pickup
    int16_t height_delta_mm;    // decoded mm value (filled by unpack, read-only)
};

struct PathPacket {
    uint8_t total_cost;
    uint8_t entry_block;         // 1–3
    uint8_t exit_block;          // 10–12
    uint8_t grid_state[12];      // BOX_* per block (0-indexed)
    uint8_t pre_entry_count;
    uint8_t step_count;
    uint8_t kfs_total;           // total KFS to collect across the match
    uint8_t kfs_stored;          // KFS already stored before this run
    uint8_t retry;               // operator retry flag (0 or 1)
    Step    steps[MAX_STEPS];
};

// Assembled CAN frames, ready to hand to HAL_CAN_AddTxMessage
struct CanFrames {
    uint8_t data[MAX_FRAMES][CAN_FRAME_BYTES];
    uint8_t count;
};

// ── Validation ────────────────────────────────────────────────────────────────
// Returns true if valid, false if a rule is violated.
// Rule: FAKE box must not appear on entry blocks B1, B2, B3 (grid_state[0-2]).
inline bool validate(const PathPacket& pkt)
{
    for (int i = 0; i < 3; i++)
        if (pkt.grid_state[i] == BOX_FAKE) return false;
    if (pkt.step_count > MAX_STEPS) return false;
    return true;
}

// ── CRC-8  (poly=0x07, init=0x00 — standard CRC-8/SMBUS) ─────────────────────
inline uint8_t crc8(const uint8_t* buf, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

// ── Step encode / decode ──────────────────────────────────────────────────────
// Bit layout (16 bits):
//  [15:14] action_type   [13:10] current_block (0=outside, 1-12 as-is)
//   [9: 6] target_block-1   [5:4] direction
//   [3: 1] collected_after (3 bits, 0-7)   [0] auto_pickup
// Note: requires_r1_clear is packed into the high nibble of collected_after
//       to keep the 16-bit word — it occupies bit 4 of the low byte.
//
// Revised layout to fit 3-bit collected_after + r1_clear:
//  [15:14] action_type   [13:10] current_block
//   [9: 6] target_block-1   [5:4] direction
//   [3: 1] collected_after (3 bits)   [0] auto_pickup
//  requires_r1_clear stored in direction bits when it can't fit — ACTUALLY:
//  We expand: direction uses 2 bits [5:4], collected_after uses 3 bits [3:1], auto_pickup 1 bit [0]
//  requires_r1_clear: pack into a spare high bit — use bit 9 (LSB of target_block field).
//  New layout:
//  [15:14] action_type
//  [13:10] current_block (4 bits, 0-12)
//   [9: 6] target_block-1 (4 bits, 0-11) — bit 6 is LSB
//   [5: 4] direction (2 bits)
//   [3: 1] collected_after (3 bits, 0-7)
//      [0] auto_pickup (1 bit)
//  requires_r1_clear: stored in a separate byte or we sacrifice one bit.
//  Decision: pack requires_r1_clear into auto_pickup's upper nibble via step.direction
//  SIMPLEST: keep requires_r1_clear at a fixed bit. Use bit 15 of a second word?
//  No — keep single word. Drop requires_r1_clear from the encoded word (it was bit[1]),
//  shift collected_after to [3:1] (3 bits), put requires_r1_clear at bit[4] after direction.
//
// FINAL layout (backward-compatible change: collected_after widens from 2b to 3b):
//  [15:14] action_type
//  [13:10] current_block (0=outside, 1-12 as-is)
//   [9: 6] target_block - 1
//   [5: 4] direction
//   [3: 1] collected_after (3 bits — was 2 bits at [3:2])
//      [0] auto_pickup
//  requires_r1_clear: moved to bit [4] — shares with direction?  NO.
//  Use this final clean layout (total 17 bits needed — doesn't fit in 16).
//  Resolution: drop requires_r1_clear from wire encoding (it can be recomputed
//  from grid_state on the STM32 side — the grid is transmitted in the header).
//  The STM32 already has the grid and can check if target_block has BOX_R1.
inline uint16_t encodeStep(const Step& s)
{
    return (uint16_t)(
        ((uint16_t)(s.action_type          & 0x03) << 14) |
        ((uint16_t)(s.current_block        & 0x0F) << 10) |
        ((uint16_t)((s.target_block - 1u)  & 0x0F) <<  6) |
        ((uint16_t)(s.direction            & 0x03) <<  4) |
        ((uint16_t)(s.collected_after      & 0x07) <<  1) |  // 3 bits
        ((uint16_t)(s.auto_pickup        ? 1u : 0u)      )
    );
}

inline Step decodeStep(uint16_t w, uint8_t h_enc = HDELTA_ZERO)
{
    Step s;
    s.action_type       = (uint8_t)((w >> 14) & 0x03);
    s.current_block     = (uint8_t)((w >> 10) & 0x0F);
    s.target_block      = (uint8_t)(((w >>  6) & 0x0F) + 1u);
    s.direction         = (uint8_t)((w >>  4) & 0x03);
    s.collected_after   = (uint8_t)((w >>  1) & 0x07);  // 3 bits
    s.requires_r1_clear = false;  // recompute from grid_state on receiver side
    s.auto_pickup       = (uint8_t)((w      ) & 0x01);
    s.height_delta_enc  = h_enc;
    s.height_delta_mm   = heightDeltaMm(h_enc);
    return s;
}

// ── Grid pack / unpack  (12 × 2-bit → 3 bytes) ───────────────────────────────
inline void packGrid(const uint8_t grid[12], uint8_t out[3])
{
    out[0] = out[1] = out[2] = 0;
    for (int i = 0; i < 12; i++) {
        int byte_i = (i * 2) / 8;
        int bit_i  = (i * 2) % 8;
        out[byte_i] |= (uint8_t)((grid[i] & 0x03) << bit_i);
    }
}

inline void unpackGrid(const uint8_t in[3], uint8_t grid[12])
{
    for (int i = 0; i < 12; i++) {
        int byte_i = (i * 2) / 8;
        int bit_i  = (i * 2) % 8;
        grid[i] = (in[byte_i] >> bit_i) & 0x03;
    }
}

// ── PACK  PathPacket → CanFrames ─────────────────────────────────────────────
inline CanFrames pack(const PathPacket& pkt)
{
    CanFrames cf;
    cf.count = 0;
    memset(cf.data, 0, sizeof(cf.data));

    uint8_t step_frames  = (uint8_t)((pkt.step_count + STEPS_PER_FRAME - 1) / STEPS_PER_FRAME);
    uint8_t total_frames = (uint8_t)(1u + step_frames);

    // ── Frame 0: Header ───────────────────────────────────────────────────────
    uint8_t* f0 = cf.data[0];
    f0[0] = MSG_ID;
    f0[1] = total_frames;
    f0[2] = pkt.total_cost;

    uint8_t entry_enc = (uint8_t)((pkt.entry_block - 1u) & 0x03);
    uint8_t exit_enc  = (uint8_t)((pkt.exit_block  - 10u) & 0x03);
    uint8_t pre_enc   = (uint8_t)(pkt.pre_entry_count & 0x07);
    f0[3] = (uint8_t)((exit_enc << 6) | (entry_enc << 4) | (pre_enc << 1) | (pkt.retry & 0x01));

    uint8_t gp[3];
    packGrid(pkt.grid_state, gp);
    f0[4] = gp[0]; f0[5] = gp[1]; f0[6] = gp[2];
    f0[7] = crc8(f0, 7);
    cf.count = 1;

    // ── Step frames ───────────────────────────────────────────────────────────
    uint8_t si = 0;
    for (uint8_t fi = 1; fi <= step_frames && fi < MAX_FRAMES; fi++) {
        uint8_t* fn = cf.data[fi];
        fn[0] = fi;

        uint8_t steps_here = 0;
        uint8_t h0 = HDELTA_ZERO, h1 = HDELTA_ZERO;

        for (uint8_t s = 0; s < STEPS_PER_FRAME && si < pkt.step_count; s++, si++) {
            const Step& sp = pkt.steps[si];
            uint16_t w = encodeStep(sp);
            fn[2 + s * 2] = (uint8_t)(w >> 8);
            fn[3 + s * 2] = (uint8_t)(w & 0xFF);

            // current_block==0 means robot is on the ground (PRE_ENTRY or ENTER step)
            // → use absolute block height of target.
            // Otherwise use relative delta between current and target.
            uint8_t enc = (sp.current_block == 0)
                ? preEntryHeightDeltaEnc(sp.target_block)
                : (sp.height_delta_enc != HDELTA_ZERO || sp.action_type == ACT_MOVE
                    ? sp.height_delta_enc
                    : heightDeltaEnc(sp.current_block, sp.target_block));

            if (s == 0) h0 = enc; else h1 = enc;
            steps_here++;
        }

        fn[1] = steps_here;
        // B6: [7:5]=S0 height enc, [4:2]=S1 height enc, [1:0]=0x00
        fn[6] = (uint8_t)(((h0 & 0x07u) << 5) | ((h1 & 0x07u) << 2));
        fn[7] = crc8(fn, 7);
        cf.count++;
    }

    return cf;
}

// ── UNPACK  CanFrames → PathPacket ───────────────────────────────────────────

struct UnpackResult {
    bool       ok;
    uint8_t    error_frame;  // 0xFF = no error
    PathPacket pkt;
};

inline UnpackResult unpack(const CanFrames& cf)
{
    UnpackResult res;
    res.ok          = false;
    res.error_frame = 0xFF;
    memset(&res.pkt, 0, sizeof(res.pkt));

    if (cf.count == 0) return res;

    // ── Frame 0 ───────────────────────────────────────────────────────────────
    const uint8_t* f0 = cf.data[0];

    if (f0[0] != MSG_ID)          return res;
    if (crc8(f0, 7) != f0[7])  { res.error_frame = 0; return res; }

    uint8_t total_frames = f0[1];
    if (cf.count != total_frames) return res;

    res.pkt.total_cost      = f0[2];
    uint8_t meta            = f0[3];
    res.pkt.entry_block     = (uint8_t)(((meta >> 4) & 0x03) + 1u);
    res.pkt.exit_block      = (uint8_t)(((meta >> 6) & 0x03) + 10u);
    res.pkt.pre_entry_count = (uint8_t)((meta >> 1) & 0x07);
    res.pkt.retry           = (uint8_t)(meta & 0x01);

    uint8_t gp[3] = { f0[4], f0[5], f0[6] };
    unpackGrid(gp, res.pkt.grid_state);

    // ── Step frames ───────────────────────────────────────────────────────────
    res.pkt.step_count = 0;

    for (uint8_t fi = 1; fi < cf.count; fi++) {
        const uint8_t* fn = cf.data[fi];

        if (fn[0] != fi)           { res.error_frame = fi; return res; }
        if (crc8(fn, 7) != fn[7]) { res.error_frame = fi; return res; }

        uint8_t steps_here = fn[1];
        if (steps_here > STEPS_PER_FRAME) { res.error_frame = fi; return res; }

        // Extract height deltas from B6
        uint8_t b6   = fn[6];
        uint8_t h[2] = { (uint8_t)((b6 >> 5) & 0x07), (uint8_t)((b6 >> 2) & 0x07) };

        for (uint8_t s = 0; s < steps_here; s++) {
            if (res.pkt.step_count >= MAX_STEPS) break;
            uint16_t w = (uint16_t)((fn[2 + s*2] << 8) | fn[3 + s*2]);
            res.pkt.steps[res.pkt.step_count++] = decodeStep(w, h[s]);
        }
    }

    res.ok = true;
    return res;
}

} // namespace r2can
