#pragma once
#include <stdint.h>
#include <zephyr/storage/flash_map.h>
struct fcb { const struct flash_area *fap; uint32_t f_magic; uint8_t f_erase_value; void *f_sectors; uint32_t f_sector_cnt; };
struct fcb_entry { void *fe_sector; uint32_t fe_elem_off; };
#define FCB_ENTRY_FA_DATA_OFF(loc) ((loc).fe_elem_off)
extern "C" {
int fcb_init(int f_area_id, struct fcb *fcb);
int fcb_getnext(struct fcb *fcb, struct fcb_entry *loc);
int fcb_append(struct fcb *fcb, uint16_t len, struct fcb_entry *loc);
int fcb_append_finish(struct fcb *fcb, struct fcb_entry *append_loc);
int fcb_rotate(struct fcb *fcb);
}
