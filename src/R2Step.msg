uint8  step_index
uint8  action_type
uint8  current_block    # 0=outside grid (PRE_ENTRY/ENTER), 1-12=block number
uint8  target_block
uint8  direction
uint8  collected_after  # 0-7 (3 bits, widened to support kfs_total up to 7)
bool   requires_r1_clear
bool   auto_pickup
uint8  height_delta_enc   # HDELTA_* encoded value (0-5): 0=-400 1=-200 2=0 3=+200 4=+400 5=+600
int16  height_delta_mm    # decoded signed mm value (filled by receiver, read-only)
