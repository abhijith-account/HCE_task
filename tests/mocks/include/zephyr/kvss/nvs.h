#pragma once
#include <stdint.h>
#include <sys/types.h>
struct device;
struct nvs_fs { const struct device *flash_device; off_t offset; uint32_t sector_size; uint32_t sector_count; };
extern "C" {
int nvs_mount(struct nvs_fs *fs);
ssize_t nvs_read(struct nvs_fs *fs, uint16_t id, void *data, size_t len);
ssize_t nvs_write(struct nvs_fs *fs, uint16_t id, const void *data, size_t len);
}
