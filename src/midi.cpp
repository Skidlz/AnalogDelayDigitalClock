#include "midi.h"

void MIDI::handleByte(uint8_t currentByte) {
    const uint8_t MAX_DATA_BYTES = 2;

    if (currentByte >= 0xf8) { //System Real Time Message
        handleRealTime(currentByte);
    } else {
        if (currentByte & 0x80) { //command
            lastStatusByte = currentByte;
            byteCount = 0;
        } else if (byteCount < MAX_DATA_BYTES) {
            dataBytes[byteCount++] = currentByte;
        }

        if (lastStatusByte >= 0xf0) { //System Common Message
            byteCount = handleSysComMess();
        } else { //Channel Voice Message
            if (chan == 0xff) chan = lastStatusByte & 0x0f; //learn chan
            byteCount = ((lastStatusByte & 0x0f) == chan) ? handleChanMess() : skipMessage(); //skip if wrong chan
        }
    }
}

uint8_t MIDI::skipMessage() { //ignore message on a different channel
    switch ((message)lastStatusByte & 0xf0) {
        case NOTE_ON: //Note On
        case NOTE_OFF: //Note Off
        case CC: //Control Change
        case BEND: //Pitch Bend Change
        case AFT_TCH: //Polyphonic Key Pressure (Aftertouch)
            if (byteCount < 2) return byteCount;

            return 0;
        case PRG_CHNG: //Program Change
        case CHN_AT: //Channel Pressure (Aftertouch)
            if (byteCount < 1) return byteCount;

            return 0;
        default:
            return 0;
    }
}

uint8_t MIDI::handleChanMess() { //handle Channel Voice Message
    switch ((message)(lastStatusByte & 0xf0)) {
        case NOTE_ON: { //Note On
            if (byteCount < 2) return byteCount;

            uint8_t note = dataBytes[0];
            uint8_t vel = dataBytes[1];
            if (vel > 0 && noteOn) noteOn(note, vel);
            else if (vel == 0 && noteOff) noteOff(note, vel);
            return 0; //zero args remain
        }
        case NOTE_OFF: { //Note Off
            if (byteCount < 2) return byteCount;

            if (noteOff) noteOff(dataBytes[0], dataBytes[1]);
            return 0;
        }
        case CC: //Control Change
            if (byteCount < 2) return byteCount;

            if (controlChange) controlChange(dataBytes[0], dataBytes[1]); //cc, value
            return 0;
        case BEND: //Pitch Bend Change
            if (byteCount < 2) return byteCount;

            if (pitchBend)
                pitchBend(dataBytes[0] | ((uint16_t)dataBytes[1] << 7)); //msb | lsb
            return 0;
        case AFT_TCH: //Polyphonic Key Pressure (Aftertouch)
            if (byteCount < 2) return byteCount;

            return 0;
        case PRG_CHNG: //Program Change
            if (byteCount < 1) return byteCount;

            if (prgChange) prgChange(dataBytes[0]);
            return 0;
        case CHN_AT: //Channel Pressure (Aftertouch)
            if (byteCount < 1) return byteCount;

            return 0;
        default: //error
            return byteCount;
    }
}

uint8_t MIDI::handleSysComMess() { //handle System Common Message
    switch ((message)lastStatusByte) {
        case SYSEX: //SysEx
            return 1;
        case POS_PNT: //Song Position Pointer
            if (byteCount < 2) return byteCount;

            if (songPosPtr) songPosPtr(dataBytes[0] | ((uint16_t)dataBytes[1] << 7));
            return 0;
        case QRT_FRAME: //MIDI Time Code Quarter Frame
        case SONG_SEL: //Song Select
            if (byteCount < 1) return byteCount;

            return 0;
        case EOX: //SysEx off
            break;
        case TUNE: //Tune Request
        default: //Undefined
            return 0;
    }
    return 0; //shut up compiler warnings
}

void MIDI::handleRealTime(uint8_t mess) { //handle System Real Time Message
    switch ((message)mess) {
        case CLOCK: //Timing Clock
            if (rtClock) rtClock();
            break;
        case START: //Start Sequence
            if (rtStart) rtStart();
            break;
        case CONT: //Continue Sequence
            if (rtCont) rtCont();
            break;
        case STOP: //Stop Sequence
            if (rtStop) rtStop();
            break;
    }
}