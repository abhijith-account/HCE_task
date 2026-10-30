#pragma once
#include <sys/types.h>
extern "C" {
int flash_read(const struct device *dev, off_t offset, void *data, size_t len);
int flash_write(const struct device *dev, off_t offset, const void *data, size_t len);
int flash_erase(const struct device *dev, off_t offset, size_t size);
}
