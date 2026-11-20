std_msgs/Header header
uint8   total_cost
uint8   entry_block           # 1-3
uint8   exit_block            # 10-12
uint8   kfs_total             # total KFS to collect across the match
uint8   kfs_stored            # KFS already stored before this run
uint8   boxes_to_collect      # = kfs_total - kfs_stored (derived, for convenience)
bool    retry                 # operator retry flag
uint8[12] grid_state          # BOX_* per block (0-indexed): 0=EMPTY 1=R1 2=R2 3=FAKE
uint8[]   pre_entry_pickups   # 1-indexed block numbers of pre-entry R2 pickups
r2_navigation/R2Step[] steps
