//
// Created by Lucho on 2020-07-02.
//


class SPEC_SmartAudioDevice {
private:
    uint8_t channel;
    uint8_t powerLevel;
    uint8_t operationMode;
    uint8_t currentFrecuency;
    uint8_t version;
    uint8_t mode;
    uint8_t orfreq;
    uint8_t powerLevelsCount;
    uint8_t *allowedPowerLevels;
    bool willBootIntoPitMode=true;
    bool initialized=false;
    bool updating=false;

public:
    SPEC_SmartAudioDevice  (){
        this->initialized=false;
    }

    SPEC_SmartAudioDevice  (uint8_t version,bool bootIntoPitMode){
        this->setVersion(version);
        this->willBootIntoPitMode=bootIntoPitMode;
    }

    bool isInitialized(){
        return this->initialized;
    }



     bool isUpdating(){
        return this->updating;
    }

    uint8_t getVersion(){
        return this->channel;
    }

    SPEC_SmartAudioDevice * setVersion(uint8_t value){
        this->version=value;
        return this;
    }

    uint8_t getChannel(){
        return this->channel;
    }
    uint8_t getPowerLevel(){
        return this->powerLevel;
    }
    uint8_t getOperationMode(){
        return this->currentFrecuency;
    }
    uint8_t getCurrentFrequency(){
        return this->currentFrecuency;
    }

    SPEC_SmartAudioDevice * setChannel(uint8_t value){
        this->channel=value;
        return this;
    }

    SPEC_SmartAudioDevice * setPowerLevel(uint8_t value){
        this->powerLevel=value;
        return this;
    }
    SPEC_SmartAudioDevice * setOperationMode(uint8_t value){
        this->operationMode=value;
        return this;
    }
    SPEC_SmartAudioDevice * setCurrentFrequency(uint8_t value){
        this->currentFrecuency=value;
        return this;
    }

    uint8_t getPowerLevelsCount(){
        return this->currentFrecuency;
    }

    SPEC_SmartAudioDevice * setPowerLevelsCount(uint8_t value){
        this->powerLevelsCount=value;
        return this;
    }


    uint8_t* getAllowedPowerLevels(){
        return this->allowedPowerLevels;
    }

    SPEC_SmartAudioDevice * setAllowedPowerLevels(uint8_t value){
        this->allowedPowerLevels=&value;
        return this;
    }

};


