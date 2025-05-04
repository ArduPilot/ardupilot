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
//#define HAL_ESP32_SDCARD 1


#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "SdCard.h"

#include "esp_vfs_fat.h"
#include "esp_ota_ops.h"

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "SPIDevice.h"

#ifdef HAL_ESP32_SDCARD

#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#define SPI_DMA_CHAN    host.slot
#else
#define SPI_DMA_CHAN    1
#endif

sdmmc_card_t* card = nullptr;

const size_t buffer_size = 8*1024;
const char* fw_name = "/SDCARD/APM/ardupilot.bin";

static bool sdcard_running;
static HAL_Semaphore sem;

void update_fw()
{
    FILE *f = fopen(fw_name, "r");
    void *buffer = calloc(1, buffer_size);
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

#ifdef HAL_ESP32_SDMMC

void mount_sdcard_mmc()
{
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    /*
    https://docs.espressif.com/projects/esp-idf/en/v4.1/api-reference/peripherals/sdmmc_host.html

    // we take the MMC parts from this example:
    https://github.com/espressif/esp-idf/blob/release/v4.1/examples/storage/sd_card/main/sd_card_example_main.c

    hardcoded SDMMC gpio options....

    Slot 0 (SDMMC_HOST_SLOT_0) is an 8-bit slot. It uses HS1_* signals in the PIN MUX.- dont use slot0, is used for SPI-flash chip.

    Slot 1 (SDMMC_HOST_SLOT_1) is a 4-bit slot. It uses HS2_* signals in the PIN MUX.

    this is the full list, but u can get away without some (2 or 3) of these in some cases:
    Signal	Slot 1
      CMD	GPIO15
      CLK	GPIO14
      D0	GPIO2
      D1	GPIO4
      D2	GPIO12
      D3	GPIO13

    */

    //  the dedicated SDMMC host peripheral on the esp32 is about twice as fast as SD card SPI interface in '1-line' mode and somewht faster again in '4-line' mode.  we've using 1-line mode in this driver, so we need less gpio's

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDMMC");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode (this driver does), uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    //gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    //gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    //
    // Pin 13 / chip-select  - is an interesting one, because if its the only thing on this
    //   spi bus(it is), then NOT connecting the SD to this pin, and instead directly to a pull-up
    //   also asserts the CS pin 'permanently high' to the SD card, without the micro being involved..
    //   which means pin 13 on micro can be re-used elsewhere. If one of these isn't true for u,
    //   then uncomment this line and connect it electrically to the CS pin on the SDcard.
    //gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes


    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };

    //https://docs.espressif.com/projects/esp-idf/en/v4.1/api-reference/peripherals/sdmmc_host.html


    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
        sdcard_running = true;
    } else {
        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}

void mount_sdcard()
{
    mount_sdcard_mmc();
}

#endif // emd mmc


#ifdef HAL_ESP32_SDSPI
ESP32::SPIBusDesc bus_ = HAL_ESP32_SDSPI;

void mount_sdcard_spi()
{

    esp_err_t ret;
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    //  In SPI mode, pins can be customized...

    // and 'SPI bus sharing with SDSPI has been added in 067f3d2 — please see the new sdspi_host_init_device, sdspi_host_remove_device functions'. https://github.com/espressif/esp-idf/issues/1597 buzz..: thats in idf circa 4.3-dev tho.

    // readme shows esp32 pinouts and pullups needed for different modes and 1 vs 4 line gotchass...
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/README.md
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/main/sd_card_example_main.c

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDSPI");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    //TODO change to sdspi_host_init_device for spi sharing
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = bus_.mosi,
        .miso_io_num = bus_.miso,
        .sclk_io_num = bus_.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = bus_.cs;
    slot_config.host_id = (spi_host_device_t)host.slot;

    //host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_DDR;
    host.max_freq_khz = SDMMC_FREQ_PROBING;
    ret = esp_vfs_fat_sdspi_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        // Card has been initialized, print its properties

        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
        sdcard_running = true;
    } else {
        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}
void mount_sdcard()
{
    mount_sdcard_spi();
}

#endif // end spi

bool sdcard_retry(void)
{
    if (!sdcard_running) {
        mount_sdcard();
    }
    return sdcard_running;
}

void unmount_sdcard()
{
    if (card != nullptr) {
        esp_vfs_fat_sdcard_unmount( "/SDCARD", card);
    }
    sdcard_running = false;
}

#else
// empty impl's
void mount_sdcard()
{
    printf("No sdcard setup.\n");
}
void unmount_sdcard()
{
}
bool sdcard_retry(void)
{
    return true;
}
#endif




