#pragma once
#include <stdint.h>
#include <sys/types.h>
#define FIXED_PARTITION_ID(...) 0
#define DT_FIXED_PARTITION_ID(...) 0
#define FIXED_PARTITION_DEVICE(...) nullptr
#define FIXED_PARTITION_OFFSET(...) 0
struct flash_sector { uint32_t fs_off; uint32_t fs_size; };
struct flash_area { uint8_t fa_id; size_t fa_size; };
extern "C" {
int flash_area_open(uint8_t id, const struct flash_area **fa);
void flash_area_close(const struct flash_area *fa);
int flash_area_read(const struct flash_area *fa, off_t off, void *dst, size_t len);
int flash_area_write(const struct flash_area *fa, off_t off, const void *src, size_t len);
int flash_area_erase(const struct flash_area *fa, off_t off, size_t len);
int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors);
}
