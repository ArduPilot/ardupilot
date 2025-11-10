/*
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
/*
  simple buzzer simulator class
*/


#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#include "SIM_Buzzer.h"

#ifdef WITH_SITL_TONEALARM

#ifdef HAVE_SFML_AUDIO_H
#include <SFML/Audio.h>
#else
#include <SFML/Audio.hpp>
#endif

// need to namespace this or we end up with multiple definitions of
// things from Synth.hpp
namespace BuzzerSynth {
#include <AP_HAL_SITL/Synth.hpp>
};

#endif

using namespace SITL;

#ifdef WITH_SITL_TONEALARM

// table of user settable parameters
const AP_Param::GroupInfo Buzzer::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Buzzer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the simulated buzzer
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, Buzzer, _enable, 0),

    // @Param: PIN
    // @DisplayName: buzzer pin
    // @Description: The pin number that the Buzzer is connected to (start at 1)
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("PIN", 1, Buzzer, _pin, -1),

    AP_GROUPEND
};

static sf::SoundBuffer xsoundBuffer;
static sf::Sound xdemoSound;

static uint32_t duration_ms = 10000;

Buzzer::Buzzer() {
    AP_Param::setup_object_defaults(this, var_info);
};

void Buzzer::update(const struct sitl_input &input)
{
    // prepare buzz tone
//    BuzzerSynth::Synth::sEnvelope xenvelope;
    if (!prep_done) {
        BuzzerSynth::Synth::sEnvelope xenvelope;
        xenvelope.dAttackTime = 0.000001;
        xenvelope.dDecayTime = 0.000001;
        xenvelope.dSustainTime = 10;
        xenvelope.dReleaseTime = 0.00001;
        xenvelope.dStartAmplitude = 1.0;
        xenvelope.dSustainAmplitude = 1.0;

        const uint32_t frequency = 2000;
        BuzzerSynth::Synth::sTone tone;
        tone.waveType = BuzzerSynth::Synth::OSC_SQUARE;
        tone.dStartFrequency = frequency;
        tone.dEndFrequency = frequency;
        tone.dAmplitude = 1;

        xenvelope.dSustainTime = duration_ms/1000.0f;

        BuzzerSynth::Synth::generate(&xsoundBuffer, xenvelope, tone, 20000, 44100);
        xdemoSound.setBuffer(xsoundBuffer);
        prep_done = true;
    }

    const bool on = _pin >= 1 && (AP::sitl()->pin_mask.get() & (1<<_pin));
    const uint32_t now = AP_HAL::millis();
    if (on) {
        if (!was_on) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%u: Buzzer on", now);
            on_time = now;
            was_on = true;
            xdemoSound.play();
        }
        if (now - on_time > duration_ms/2) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%u: Buzzer on again", now);
            on_time = now;
            xdemoSound.play();
        }
    } else {
        if (was_on) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%u: Buzzer off", now);
            xdemoSound.stop();
            was_on = false;
        }
    }

}

#else

using namespace SITL;

const AP_Param::GroupInfo Buzzer::var_info[] = { AP_GROUPEND };

Buzzer::Buzzer() { };

void Buzzer::update(const struct sitl_input &input) {}

#endif
