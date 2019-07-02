#ifdef WITH_SITL_TONEALARM

#include "ToneAlarm_SF.h"

#include <stdio.h>

#ifdef HAVE_SFML_AUDIO_H
#include <SFML/Audio.h>
#else
#include <SFML/Audio.hpp>
#endif

#include "Synth.hpp"

using namespace HALSITL;

sf::SoundBuffer soundBuffer;
sf::Sound demoSound;
Synth::sEnvelope envelope;

void ToneAlarm_SF::set_buzzer_tone(float frequency, float volume, float duration_ms)
{
    if (frequency <= 0) {
        return;
    }

    Synth::sTone tone;
    tone.waveType = Synth::OSC_SQUARE;
    tone.dStartFrequency = frequency;
    tone.dEndFrequency = frequency;
    tone.dAmplitude = 1;

    envelope.dSustainTime = duration_ms/1000.0f;

    Synth::generate(&soundBuffer, envelope, tone, 20000, 44100);
    demoSound.setBuffer(soundBuffer);
    demoSound.play();
}

bool ToneAlarm_SF::init()
{
    // LASER
    envelope.dAttackTime = 0.00001;
    envelope.dDecayTime = 0.00001;
    envelope.dSustainTime = 0.3;
    envelope.dReleaseTime = 0.00001;
    envelope.dStartAmplitude = 1.0;
    envelope.dSustainAmplitude = 1.0;

    return true;
}

#endif
