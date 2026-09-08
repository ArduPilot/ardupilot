/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Zephyr flash-driver facade over the BootROM flash API, so flash_map and
 * img_mgmt work without enabling the FlexSPI driver that breaks XIP. */

#define DT_DRV_COMPAT ardupilot_rt1176_romapi_flash

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/cache.h>
#include <string.h>

#include "rt1176_romapi_flash.h"

#define ROMAPI_FLASH_SIZE  (64U * 1024U * 1024U)   /* MX25UM51345G, 64 MB */

static int romapi_dev_read(const struct device *dev, off_t offset,
			   void *data, size_t len)
{
	ARG_UNUSED(dev);
	if (offset < 0 || (size_t)offset + len > ROMAPI_FLASH_SIZE) {
		return -EINVAL;
	}
	const void *src = (const void *)(uintptr_t)
		(RT1176_FLASH_MEMMAP_BASE + (uint32_t)offset);
	sys_cache_data_invd_range((void *)src, len);
	memcpy(data, src, len);
	return 0;
}

static int romapi_dev_write(const struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	ARG_UNUSED(dev);
	if (offset < 0 || (size_t)offset + len > ROMAPI_FLASH_SIZE) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	return rt1176_flash_program((uint32_t)offset, data, len) == 0 ? 0 : -EIO;
}

static int romapi_dev_erase(const struct device *dev, off_t offset,
			    size_t size)
{
	ARG_UNUSED(dev);
	if (offset < 0 || (size_t)offset + size > ROMAPI_FLASH_SIZE) {
		return -EINVAL;
	}
	if ((offset % RT1176_FLASH_ERASE_CHUNK) != 0 ||
	    (size % RT1176_FLASH_ERASE_CHUNK) != 0) {
		return -EINVAL;
	}
	return rt1176_flash_erase((uint32_t)offset, size) == 0 ? 0 : -EIO;
}

static const struct flash_parameters romapi_dev_parameters = {
	/* Callers may write with any 4-byte alignment; rt1176_flash_program
	   assembles full 256-byte pages internally (0xFF filler programs no
	   bits). Sequential writers (stream_flash/flash_img) never touch a
	   page twice between erases, which is the part's real constraint. */
	.write_block_size = 4,
	.erase_value = 0xff,
};

static const struct flash_parameters *
romapi_dev_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &romapi_dev_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout romapi_dev_pages_layout = {
	.pages_count = ROMAPI_FLASH_SIZE / RT1176_FLASH_ERASE_CHUNK,
	.pages_size  = RT1176_FLASH_ERASE_CHUNK,   /* 4 KB uniform */
};

static void romapi_dev_page_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	ARG_UNUSED(dev);
	*layout = &romapi_dev_pages_layout;
	*layout_size = 1;
}
#endif

static DEVICE_API(flash, romapi_dev_api) = {
	.read = romapi_dev_read,
	.write = romapi_dev_write,
	.erase = romapi_dev_erase,
	.get_parameters = romapi_dev_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = romapi_dev_page_layout,
#endif
};

static int romapi_dev_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* rt1176_flash_init() is lazy inside erase/program; nothing to do
	   here, and failing init would only take flash_map down with it. */
	return 0;
}

DEVICE_DT_INST_DEFINE(0, romapi_dev_init, NULL, NULL, NULL,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &romapi_dev_api);
