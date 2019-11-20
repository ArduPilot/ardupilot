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
#define HAL_ESP32_SDCARD 1

#ifdef HAL_ESP32_SDCARD
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "SdCard.h"

#include "esp_vfs_fat.h"
#include "esp_ota_ops.h"
#include "driver/sdmmc_host.h"
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "SPIDevice.h"

sdmmc_card_t* card = nullptr;

const size_t buffer_size = 8*1024;
const char* fw_name = "/SDCARD/APM/ardupilot.bin";

void update_fw()
{
    FILE *f = fopen(fw_name, "r");
    void *buffer = malloc(buffer_size);
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    size_t nread = 0;
    if (f == nullptr || buffer == nullptr || update_partition == nullptr) {
        goto done;
    }
    printf("updating firmware...\n");
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle) != ESP_OK) {
        goto done;
    }
    do {
        nread = fread(buffer, 1, buffer_size, f);
        if (nread > 0) {
            if (esp_ota_write(update_handle, buffer, nread) != ESP_OK) {
                goto done;
            }
        }
    } while (nread > 0);
done:
    if (update_handle != 0) {
        if (esp_ota_end(update_handle) == ESP_OK &&
            esp_ota_set_boot_partition(update_partition) == ESP_OK) {
            printf("firmware updated\n");
        }
    }
    if (f != nullptr) {
        fclose(f);
    }
    if (buffer != nullptr) {
        free(buffer);
    }
    unlink(fw_name);
}

#ifdef HAL_ESP32_SDSPI
ESP32::SPIBusDesc bus_ = HAL_ESP32_SDSPI;
#endif

void mount_sdcard()
{
	printf("Mounting sd \n");
	//sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = bus_.miso;
    slot_config.gpio_mosi = bus_.mosi;
    slot_config.gpio_sck  = bus_.sclk;
    slot_config.gpio_cs   = bus_.cs;

    //host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_DDR;
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 512
    };
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/SDCARD", &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK) {
        mkdir("/SDCARD/APM", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
    } else {
        printf("sdcard is not mounted\n");
    }
}

void unmount_sdcard()
{
    if (card != nullptr) {
        esp_vfs_fat_sdmmc_unmount();
    }
}
#else

void mount_sdcard()
{
	printf("No sdcard setup.\n");
}

void unmount_sdcard()
{
}

#endif

