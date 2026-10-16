/*
   GCS MAVLink functions related to component metadata

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "GCS_config.h"

#if AP_MAVLINK_COMPONENT_INFORMATION_ENABLED

#include "GCS.h"

#include <AP_Common/ExpandingString.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Math/crc.h>

#if AP_LOGGER_MAVLINK_EVENTS_METADATA_ROMFS_ENABLED
// returns true if the events metadata is embedded in ROMFS
bool GCS::romfs_events_metadata_crc(uint32_t &crc)
{
    return AP::FS().crc32("@ROMFS/events.json.xz", crc);
}
#endif

/*
  GCSs cache metadata files keyed by the CRC we advertise for them, so
  the CRCs must change whenever the files do.  Files are only looked
  for at boot; a file put on the SD card takes effect after a reboot
 */
void GCS::setup_component_metadata()
{
#if AP_LOGGER_MAVLINK_EVENTS_ENABLED
    // a file on the SD card replaces the one embedded in ROMFS
    if (AP::FS().crc32("events.json.xz", events_metadata.crc)) {
        events_metadata.uri = "mftp://events.json.xz";
#if AP_LOGGER_MAVLINK_EVENTS_METADATA_ROMFS_ENABLED
    } else if (romfs_events_metadata_crc(events_metadata.crc)) {
        events_metadata.uri = "mftp://@ROMFS/events.json.xz";
#endif
    }
#endif  // AP_LOGGER_MAVLINK_EVENTS_ENABLED

    ExpandingString str;
    general_metadata(str);
    if (str.has_failed_allocation()) {
        return;
    }
    _general_metadata_crc = crc_crc32(0, (const uint8_t *)str.get_string(), str.get_length());
}

void GCS::general_metadata(ExpandingString &str) const
{
    str.printf("{\"version\":1,\"metadataTypes\":[");
#if AP_LOGGER_MAVLINK_EVENTS_ENABLED
    if (events_metadata.uri != nullptr) {
        str.printf("{\"type\":%u,\"uri\":\"%s\",\"fileCrc\":%lu}",
                   unsigned(COMP_METADATA_TYPE_EVENTS),
                   events_metadata.uri,
                   (unsigned long)events_metadata.crc);
    }
#endif
    str.printf("]}");
}

#endif  // AP_MAVLINK_COMPONENT_INFORMATION_ENABLED
