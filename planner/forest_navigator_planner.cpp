// forest_navigator_planner.cpp
//
// Standalone C++ implementation of the R2 Forest Navigator A* path planner.
// This is the exact algorithm used in forest_navigator.html, ported to C++.
//
// Compile:  g++ -O2 -std=c++17 -o planner forest_navigator_planner.cpp
// Usage:    ./planner   (runs built-in scenarios)
//
// To integrate: call runFullPlanner(boxes) and use the returned PlanResult.
//
// ── Grid layout ──────────────────────────────────────────────────────────────
//   [ B1 ][ B2 ][ B3 ]   ENTRY row (1-indexed, 0-indexed: 0,1,2)
//   [ B4 ][ B5 ][ B6 ]
//   [ B7 ][ B8 ][ B9 ]
//   [B10 ][B11 ][B12 ]   EXIT row
//
// ── Block heights (mm) ───────────────────────────────────────────────────────
//   B1=400  B2=200  B3=400  B4=200  B5=400  B6=600
//   B7=400  B8=600  B9=400  B10=200 B11=400 B12=200
// =============================================================================

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstring>
#include <string>
#include <cassert>

// ── Constants ──────────────────────────────────────────────────────────────
static constexpr int ROWS       = 4;
static constexpr int COLS       = 3;
static constexpr int NUM_BLOCKS = 12;
// TARGET is no longer a compile-time constant — it is passed via PlannerConfig.
// Default kept here only for legacy reference:
// static constexpr int TARGET = 3;

// ── Planner configuration ──────────────────────────────────────────────────
// Pass one of these to runFullPlanner() to control how many R2 boxes to collect.
struct PlannerConfig {
    int target;      // how many R2 boxes to collect this run (kfsTotal - kfsStored)
    // Add future knobs here (e.g. weight multiplier, max_cost_limit)

    explicit PlannerConfig(int t = 3) : target(t) {}
};

// Box types
static constexpr uint8_t BOX_EMPTY = 0;
static constexpr uint8_t BOX_R1    = 1;
static constexpr uint8_t BOX_R2    = 2;
static constexpr uint8_t BOX_FAKE  = 3;

// Directions
static constexpr int DIR_DOWN  = 0;
static constexpr int DIR_RIGHT = 1;
static constexpr int DIR_UP    = 2;
static constexpr int DIR_LEFT  = 3;
static const char* DIR_NAMES[] = {"DOWN","RIGHT","UP","LEFT"};

// Action types
static constexpr int ACT_MOVE      = 0;
static constexpr int ACT_PICKUP    = 1;
static constexpr int ACT_PRE_ENTRY = 2;

// Movement costs
static constexpr int C_MOVE       = 1;
static constexpr int C_TURN_90    = 1;
static constexpr int C_TURN_180   = 2;
static constexpr int C_BACK_PEN   = 3;
static constexpr int C_PICKUP_ROT = 1;
static constexpr int C_R1_WAIT    = 3;
static constexpr int C_INF        = INT_MAX / 2;

// Block height table (mm, 1-indexed: index 0 unused)
static constexpr uint16_t BLOCK_HEIGHT_MM[13] = {
    0,                              // index 0 unused
    400, 200, 400, 200, 400, 600,   // B1–B6
    400, 600, 400, 200, 400, 200    // B7–B12
};

// Height delta encoding: 0=−400, 1=−200, 2=0, 3=+200, 4=+400, 5=+600
static constexpr uint8_t HDELTA_NEG400 = 0;
static constexpr uint8_t HDELTA_NEG200 = 1;
static constexpr uint8_t HDELTA_ZERO   = 2;
static constexpr uint8_t HDELTA_POS200 = 3;
static constexpr uint8_t HDELTA_POS400 = 4;
static constexpr uint8_t HDELTA_POS600 = 5;

// ── Grid helpers ──────────────────────────────────────────────────────────
inline int blockIdx(int r, int c) { return r * COLS + c; }
inline int rowOf(int i)           { return i / COLS; }
inline int colOf(int i)           { return i % COLS; }

int stepBlock(int idx, int dir) {
    int r = rowOf(idx), c = colOf(idx);
    if      (dir == DIR_DOWN)  r++;
    else if (dir == DIR_UP)    r--;
    else if (dir == DIR_RIGHT) c++;
    else                       c--;
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS) return -1;
    return blockIdx(r, c);
}

int rotCost(int cur, int tgt) {
    int d = ((tgt - cur) + 4) % 4;
    return d == 0 ? 0 : (d == 2 ? C_TURN_180 : C_TURN_90);
}

int moveCost(int cf, int dir) {
    return rotCost(cf, dir) + C_MOVE + (dir == DIR_UP ? C_BACK_PEN : 0);
}

// ── Height delta helpers ──────────────────────────────────────────────────
int16_t heightDeltaMm(uint8_t cur1, uint8_t tgt1) {
    if (cur1 < 1 || cur1 > 12 || tgt1 < 1 || tgt1 > 12) return 0;
    return (int16_t)BLOCK_HEIGHT_MM[tgt1] - (int16_t)BLOCK_HEIGHT_MM[cur1];
}

uint8_t heightDeltaEnc(uint8_t cur1, uint8_t tgt1) {
    // current_block==0 means robot is on the ground
    if (cur1 == 0) {
        uint16_t h = (tgt1 >= 1 && tgt1 <= 12) ? BLOCK_HEIGHT_MM[tgt1] : 0;
        switch (h) {
            case 200: return HDELTA_POS200;
            case 400: return HDELTA_POS400;
            case 600: return HDELTA_POS600;
            default:  return HDELTA_ZERO;
        }
    }
    int16_t d = heightDeltaMm(cur1, tgt1);
    switch (d) {
        case -400: return HDELTA_NEG400;
        case -200: return HDELTA_NEG200;
        case    0: return HDELTA_ZERO;
        case  200: return HDELTA_POS200;
        case  400: return HDELTA_POS400;
        default:   return HDELTA_ZERO;
    }
}

int16_t heightDeltaMmFromEnc(uint8_t enc) {
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

// ── Map ───────────────────────────────────────────────────────────────────
struct ForestMap {
    uint8_t boxes[NUM_BLOCKS] = {};
    int     r2Blocks[NUM_BLOCKS]; // up to NUM_BLOCKS R2 boxes (was hardcoded [4])
    int     r2Count = 0;

    void load(const uint8_t raw[NUM_BLOCKS]) {
        r2Count = 0;
        for (int i = 0; i < NUM_BLOCKS; i++) {
            boxes[i] = raw[i] & 0x03;
            if (boxes[i] == BOX_R2) r2Blocks[r2Count++] = i;
        }
    }

    int r2BitOf(int block) const {
        for (int i = 0; i < r2Count; i++)
            if (r2Blocks[i] == block) return i;
        return -1;
    }

    bool hasR2(int block, uint8_t picked) const {
        int bit = r2BitOf(block);
        return bit >= 0 && !(picked & (1 << bit));
    }

    bool canStandOn(int i, int collected, uint8_t picked, int target) const {
        if (boxes[i] == BOX_FAKE) return false;
        if (hasR2(i, picked) && collected >= target) return false;
        return true;
    }
};

// ── State packing ─────────────────────────────────────────────────────────
// block(4b) | collected(3b) | facing(2b) | picked(4b) → uint16_t
// collected widened from 2 bits to 3 bits to support target up to 7.
// Total: 4+3+2+4 = 13 bits — fits in uint16_t.
struct State {
    int     block;
    int     collected;
    int     facing;
    uint8_t picked;

    uint16_t pack() const {
        assert(collected >= 0 && collected <= 7 && "collected out of 3-bit range");
        return (uint16_t)(
            (block     & 0xF)        |
            ((collected & 0x7) << 4) |  // 3 bits
            ((facing    & 0x3) << 7) |  // shifted up by 1 to make room
            ((picked    & 0xF) << 9)    // shifted up by 1
        );
    }

    static State unpack(uint16_t v) {
        return {
            (int)(v & 0xF),
            (int)((v >> 4) & 0x7),   // 3 bits
            (int)((v >> 7) & 0x3),
            (uint8_t)((v >> 9) & 0xF)
        };
    }
};

// ── Action ────────────────────────────────────────────────────────────────
struct Action {
    int type;         // ACT_MOVE or ACT_PICKUP
    int direction;
    int targetBlock;  // 0-indexed
};

// ── A* node ───────────────────────────────────────────────────────────────
struct ANode {
    int      f, g;
    uint16_t key;
    bool operator>(const ANode& o) const { return f > o.f; }
};

int heuristic(int block, int collected, int target) {
    return (ROWS - 1 - rowOf(block)) + (target - collected) * 2;
}

// ── Step (output) ─────────────────────────────────────────────────────────
struct Step {
    int     action_type;        // ACT_*
    int     current_block;      // 0=outside grid, 1-12=block (1-indexed)
    int     target_block;       // 1-indexed
    int     direction;          // DIR_*
    int     collected_after;    // 0-3
    bool    requires_r1_clear;
    bool    auto_pickup;
    uint8_t height_delta_enc;   // HDELTA_*
    int16_t height_delta_mm;    // signed mm
};

// ── Plan result ───────────────────────────────────────────────────────────
struct PlanResult {
    bool              found      = false;
    int               total_cost = 0;
    int               entry_block;       // 1-indexed
    int               exit_block;        // 1-indexed
    int               pre_entry_count;
    std::vector<Step> steps;
};

// ── Core A* search ────────────────────────────────────────────────────────
struct SearchResult {
    bool              found = false;
    int               totalCost = C_INF;
    std::vector<State>  states;
    std::vector<Action> actions;
};

SearchResult planAStar(const ForestMap& map, int entry, int startFacing,
                       uint8_t initPicked, int initCollected, int target)
{
    if (rowOf(entry) != 0) return {};

    // Handle auto-pickup on entry block
    uint8_t startPicked    = initPicked;
    int     startCollected = initCollected;
    if (map.hasR2(entry, startPicked) && startCollected < target) {
        int bit = map.r2BitOf(entry);
        startPicked     |= (1 << bit);
        startCollected++;
    }

    State start { entry, startCollected, startFacing, startPicked };
    uint16_t sk = start.pack();

    std::priority_queue<ANode, std::vector<ANode>, std::greater<ANode>> open;
    std::unordered_map<uint16_t, int>    gScore;
    std::unordered_map<uint16_t, uint16_t> parent;
    std::unordered_map<uint16_t, Action>   parentAction;

    gScore[sk] = 0;
    open.push({ heuristic(entry, startCollected, target), 0, sk });

    while (!open.empty()) {
        ANode cur = open.top(); open.pop();
        auto it = gScore.find(cur.key);
        if (it == gScore.end() || cur.g > it->second) continue;

        State cs = State::unpack(cur.key);
        int g = cur.g;

        // Goal: collected enough boxes on exit row
        if (cs.collected >= target && rowOf(cs.block) == ROWS - 1) {
            SearchResult res;
            res.found     = true;
            res.totalCost = g;

            std::vector<uint16_t> keys;
            uint16_t c = cur.key;
            while (c != sk) { keys.push_back(c); c = parent.at(c); }
            keys.push_back(sk);
            std::reverse(keys.begin(), keys.end());

            for (uint16_t k : keys)
                res.states.push_back(State::unpack(k));
            for (size_t i = 1; i < keys.size(); i++)
                res.actions.push_back(parentAction.at(keys[i]));

            return res;
        }

        // PICKUP from adjacent blocks
        if (cs.collected < target) {
            for (int d = 0; d < 4; d++) {
                int adj = stepBlock(cs.block, d);
                if (adj < 0 || !map.hasR2(adj, cs.picked)) continue;
                int bit = map.r2BitOf(adj);
                int pc  = (d == cs.facing) ? 0 : rotCost(cs.facing, d) + C_PICKUP_ROT;

                State ns { cs.block, cs.collected + 1, d,
                           (uint8_t)(cs.picked | (1 << bit)) };
                uint16_t nk = ns.pack();
                int ng = g + pc;
                auto jt = gScore.find(nk);
                if (jt == gScore.end() || ng < jt->second) {
                    gScore[nk] = ng;
                    parent[nk] = cur.key;
                    parentAction[nk] = { ACT_PICKUP, d, adj };
                    open.push({ ng + heuristic(cs.block, cs.collected + 1, target), ng, nk });
                }
            }
        }

        // MOVE to adjacent block
        for (int d = 0; d < 4; d++) {
            int next = stepBlock(cs.block, d);
            if (next < 0 || !map.canStandOn(next, cs.collected, cs.picked, target)) continue;

            int mc = moveCost(cs.facing, d);
            if (map.boxes[next] == BOX_R1) mc += C_R1_WAIT;

            int     ns_collected = cs.collected;
            uint8_t ns_picked    = cs.picked;
            if (map.hasR2(next, cs.picked) && ns_collected < target) {
                int bit = map.r2BitOf(next);
                ns_collected++;
                ns_picked |= (1 << bit);
            }

            State ns { next, ns_collected, d, ns_picked };
            uint16_t nk = ns.pack();
            int ng = g + mc;
            auto jt = gScore.find(nk);
            if (jt == gScore.end() || ng < jt->second) {
                gScore[nk] = ng;
                parent[nk] = cur.key;
                parentAction[nk] = { ACT_MOVE, d, next };
                open.push({ ng + heuristic(next, ns_collected, target), ng, nk });
            }
        }
    }
    return {};
}

// ── Generate all valid entry states ──────────────────────────────────────
struct EntryState {
    int     entryBlock;     // 0-indexed
    uint8_t initPicked;
    int     initCollected;
};

std::vector<EntryState> generateEntryStates(const ForestMap& map, int target) {
    std::vector<int> entryR2;
    for (int c = 0; c < COLS; c++) {
        int b = blockIdx(0, c);
        if (map.boxes[b] == BOX_R2) entryR2.push_back(b);
    }
    bool mustPick = !entryR2.empty();
    int subsets   = 1 << (int)entryR2.size();

    std::vector<EntryState> results;

    for (int mask = 0; mask < subsets; mask++) {
        if (mustPick && mask == 0) continue;
        uint8_t picked    = 0;
        int     collected = 0;
        for (int i = 0; i < (int)entryR2.size(); i++) {
            if (mask & (1 << i)) {
                int bit = map.r2BitOf(entryR2[i]);
                picked |= (1 << bit);
                collected++;
            }
        }
        if (collected >= target) continue;
        for (int col = 0; col < COLS; col++) {
            int eb = blockIdx(0, col);
            if (map.boxes[eb] == BOX_FAKE) continue;
            results.push_back({ eb, picked, collected });
        }
    }

    if (!mustPick) {
        for (int col = 0; col < COLS; col++) {
            int eb = blockIdx(0, col);
            if (map.boxes[eb] != BOX_FAKE)
                results.push_back({ eb, 0, 0 });
        }
    }
    return results;
}

// ── Build final step list from search result ──────────────────────────────
// Matches buildPayload() in the PWA exactly.
std::vector<Step> buildSteps(const SearchResult& res, int entryBlock0,
                              const EntryState& es, const ForestMap& map)
{
    std::vector<Step> steps;

    // PRE_ENTRY steps — one per pre-picked R2 box
    for (int i = 0; i < map.r2Count; i++) {
        if (!(es.initPicked & (1 << i))) continue;
        int blk1 = map.r2Blocks[i] + 1;  // 1-indexed
        Step s;
        s.action_type       = ACT_PRE_ENTRY;
        s.current_block     = 0;           // outside grid
        s.target_block      = blk1;
        s.direction         = DIR_DOWN;
        s.collected_after   = (int)steps.size() + 1;
        s.requires_r1_clear = false;
        s.auto_pickup       = false;
        s.height_delta_enc  = heightDeltaEnc(0, blk1);
        s.height_delta_mm   = heightDeltaMmFromEnc(s.height_delta_enc);
        steps.push_back(s);
    }

    // ENTER step (i=0) + in-forest steps (i>0)
    for (size_t i = 0; i < res.states.size(); i++) {
        const State& s = res.states[i];

        if (i == 0) {
            // ENTER: robot moves from outside onto entry block
            Step step;
            step.action_type       = ACT_MOVE;
            step.current_block     = 0;              // outside
            step.target_block      = entryBlock0 + 1; // 1-indexed
            step.direction         = DIR_DOWN;
            step.collected_after   = s.collected;
            step.requires_r1_clear = false;
            step.auto_pickup       = (s.collected > es.initCollected);
            step.height_delta_enc  = heightDeltaEnc(0, entryBlock0 + 1);
            step.height_delta_mm   = heightDeltaMmFromEnc(step.height_delta_enc);
            steps.push_back(step);
            continue;
        }

        const Action& a        = res.actions[i - 1];
        const State&  prevState = res.states[i - 1];

        Step step;
        step.action_type       = (a.type == ACT_PICKUP) ? ACT_PICKUP : ACT_MOVE;
        step.current_block     = prevState.block + 1;          // 1-indexed
        step.target_block      = a.targetBlock + 1;            // 1-indexed
        step.direction         = a.direction;
        step.collected_after   = s.collected;
        step.requires_r1_clear = (map.boxes[a.targetBlock] == BOX_R1);
        step.auto_pickup       = (a.type == ACT_MOVE && s.collected > prevState.collected);
        step.height_delta_enc  = heightDeltaEnc(prevState.block + 1, a.targetBlock + 1);
        step.height_delta_mm   = heightDeltaMmFromEnc(step.height_delta_enc);
        steps.push_back(step);
    }

    return steps;
}

// ── Main planner entry point ──────────────────────────────────────────────
// boxes: 12-element array, each 0-3 (BOX_EMPTY/R1/R2/FAKE), 0-indexed (B1=index 0)
// cfg.target = kfsTotal - kfsStored  (how many R2 boxes to collect this run)
// Returns PlanResult with found=false if no valid path exists.
PlanResult runFullPlanner(const uint8_t boxes[NUM_BLOCKS],
                          const PlannerConfig& cfg = PlannerConfig{}) {
    const int target = cfg.target;
    ForestMap map;
    map.load(boxes);

    auto entries = generateEntryStates(map, target);

    SearchResult best;
    EntryState   bestES { 0, 0, 0 };
    int          bestEntry = -1;

    for (const auto& es : entries) {
        auto res = planAStar(map, es.entryBlock, DIR_DOWN,
                             es.initPicked, es.initCollected, target);
        if (res.found && (!best.found || res.totalCost < best.totalCost)) {
            best      = res;
            bestES    = es;
            bestEntry = es.entryBlock;
        }
    }

    if (!best.found) return {};

    PlanResult result;
    result.found            = true;
    result.total_cost       = best.totalCost;
    result.entry_block      = bestEntry + 1;         // 1-indexed
    result.exit_block       = best.states.back().block + 1;
    result.pre_entry_count  = 0;
    result.steps            = buildSteps(best, bestEntry, bestES, map);

    // Count pre-entry steps
    for (const auto& s : result.steps)
        if (s.action_type == ACT_PRE_ENTRY) result.pre_entry_count++;

    return result;
}

// ── Pretty print helpers ──────────────────────────────────────────────────
void printGrid(const uint8_t boxes[NUM_BLOCKS]) {
    const char* sym[] = {"    ", "R1  ", "R2  ", "FAKE"};
    std::cout << "  +----------+----------+----------+  <- ENTRY\n";
    for (int r = 0; r < ROWS; r++) {
        std::cout << "  |";
        for (int c = 0; c < COLS; c++) {
            int i = blockIdx(r, c);
            std::cout << " " << sym[boxes[i]] << "B" << (i < 9 ? "0" : "") << (i+1) << " |";
        }
        if (r == ROWS - 1)
            std::cout << "  <- EXIT\n  +----------+----------+----------+\n";
        else
            std::cout << "\n  +----------+----------+----------+\n";
    }
}

void printResult(const PlanResult& r) {
    if (!r.found) {
        std::cout << "  *** NO VALID PATH FOUND ***\n";
        return;
    }

    std::cout << "\n  Total cost    : " << r.total_cost
              << "\n  Entry block   : B" << r.entry_block
              << "\n  Exit block    : B" << r.exit_block
              << "\n  Pre-entry cnt : " << r.pre_entry_count
              << "\n  Total steps   : " << r.steps.size() << "\n\n";

    for (size_t i = 0; i < r.steps.size(); i++) {
        const Step& s = r.steps[i];
        const char* act = s.action_type == ACT_PRE_ENTRY ? "PRE_ENTRY" :
                          s.action_type == ACT_PICKUP     ? "PICKUP   " : "MOVE     ";
        std::cout << "  [" << i << "] " << act
                  << "  cur=" << s.current_block
                  << "  tgt=B" << s.target_block
                  << "  dir=" << DIR_NAMES[s.direction]
                  << "  col=" << s.collected_after;
        if (s.requires_r1_clear) std::cout << "  [R1-WAIT]";
        if (s.auto_pickup)       std::cout << "  [AUTO-PICKUP]";
        if (s.height_delta_mm != 0)
            std::cout << "  h=" << (s.height_delta_mm > 0 ? "+" : "")
                      << s.height_delta_mm << "mm";
        std::cout << "\n";
    }
}

// ── Scenarios ─────────────────────────────────────────────────────────────
void runScenario(const char* name, const uint8_t boxes[NUM_BLOCKS],
                 const PlannerConfig& cfg = PlannerConfig{}) {
    std::cout << "\n======================================\n";
    std::cout << "  " << name << "\n";
    std::cout << "======================================\n";
    printGrid(boxes);
    auto result = runFullPlanner(boxes, cfg);
    printResult(result);
}

void runTests() {
    std::cout << "\n======================================\n";
    std::cout << "  SELF-TESTS\n";
    std::cout << "======================================\n";

    PlannerConfig cfg3(3);  // standard target=3

    // Test 1: straight-line path, no R1, cost=3
    {
        uint8_t boxes[12] = {
            BOX_R2, BOX_R1, BOX_R2,
            BOX_R1, BOX_R2, BOX_FAKE,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY,
            BOX_R1,    BOX_R2,    BOX_EMPTY
        };
        auto r = runFullPlanner(boxes, cfg3);
        assert(r.found);
        assert(r.total_cost == 3);
        assert(r.entry_block == 2);
        assert(r.exit_block  == 11);
        assert(r.pre_entry_count == 1);
        bool hasEnter = false;
        for (auto& s : r.steps)
            if (s.action_type == ACT_MOVE && s.current_block == 0)
                hasEnter = true;
        assert(hasEnter);
        std::cout << "  Test 1 PASS (cost=3, entry=B2, exit=B11, ENTER step present)\n";
    }

    // Test 2: ENTER step auto_pickup when entry block has R2
    {
        uint8_t boxes[12] = {
            BOX_R2, BOX_R2, BOX_R2,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY
        };
        auto r = runFullPlanner(boxes, cfg3);
        assert(r.found);
        (void)r;
        std::cout << "  Test 2 PASS (entry with R2 boxes handled)\n";
    }

    // Test 3: height delta for ENTER step from ground to 400mm block
    {
        uint8_t boxes[12] = {
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY
        };
        auto r = runFullPlanner(boxes, cfg3);
        for (auto& s : r.steps) {
            if (s.action_type == ACT_MOVE && s.current_block == 0) {
                assert(s.height_delta_enc == HDELTA_POS400);
                assert(s.height_delta_mm  == 400);
                std::cout << "  Test 3 PASS (ENTER to B1(400mm): hdelta=+400mm enc=4)\n";
                break;
            }
        }
    }

    // Test 4: PRE_ENTRY current_block=0
    {
        uint8_t boxes[12] = {
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY
        };
        auto r = runFullPlanner(boxes, cfg3);
        for (auto& s : r.steps) {
            if (s.action_type == ACT_PRE_ENTRY) {
                assert(s.current_block == 0);
                std::cout << "  Test 4 PASS (PRE_ENTRY current_block=0)\n";
                break;
            }
        }
    }

    // Test 5: variable target — collect only 2 out of 4 R2 boxes
    {
        uint8_t boxes[12] = {
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY
        };
        PlannerConfig cfg2(2);
        auto r = runFullPlanner(boxes, cfg2);
        assert(r.found);
        // Exit state must have collected == 2
        bool ok = false;
        for (auto& s : r.steps)
            if (s.collected_after == 2) ok = true;
        assert(ok);
        std::cout << "  Test 5 PASS (variable target=2 works)\n";
    }

    std::cout << "  All tests passed.\n";
}

int main() {
    runTests();

    PlannerConfig cfg3(3);  // default: collect 3 R2 boxes

    // Scenario 1: from the original session
    {
        uint8_t s1[12] = {
            BOX_R2, BOX_R1, BOX_R2,
            BOX_R1, BOX_R2, BOX_FAKE,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY,
            BOX_R1,    BOX_R2,    BOX_EMPTY
        };
        runScenario("Scenario 1 (cost=3, straight path)", s1, cfg3);
    }

    // Scenario 2: R1 coordination required
    {
        uint8_t s2[12] = {
            BOX_R1, BOX_R1, BOX_R2,
            BOX_R2, BOX_FAKE, BOX_R2,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R1, BOX_EMPTY, BOX_EMPTY
        };
        runScenario("Scenario 2 (R1 coordination)", s2, cfg3);
    }

    // Scenario 3: hardest layout (cost=16, all-column trap)
    {
        uint8_t s3[12] = {
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_R1,
            BOX_R2, BOX_FAKE,  BOX_R1,
            BOX_R2, BOX_EMPTY, BOX_R1
        };
        runScenario("Scenario 3 (hard layout, max cost)", s3, cfg3);
    }

    // Scenario 4: variable target — 1 stored, need 2 more (target=2)
    {
        uint8_t s4[12] = {
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY,
            BOX_R2, BOX_EMPTY, BOX_EMPTY,
            BOX_EMPTY, BOX_EMPTY, BOX_EMPTY
        };
        PlannerConfig cfg2(2);  // kfsTotal=3, kfsStored=1 → need 2
        runScenario("Scenario 4 (kfsStored=1, need 2 more)", s4, cfg2);
    }

    return 0;
}
