#include <stdint.h>

enum class Waveform
{
    eSinusoid,
    eSawtooth,
    eTriangle,
    eSquare,
};

enum class Resolution
{
    e8Bit           = 0x00FF,
    e12Bit_Lower    = 0x0FFF,
    e12Bit_Upper    = 0xFFF0,
    e16Bit          = 0xFFFF,
};

class WaveGenerator
{
    //---DATA---
    public:
        static const int DURATION_FOREVER = 0xFFFFFFFF;
        
    protected:
        Waveform wave_type;
        int wave_freq_Hz;
        int duration_ms;
        /** Normalized value 0-1 this is a acoefficidnt to the resolution,
         *    1 means full swing
         *  0.5 means half swing (128 for 8 bit, 2048 for 12 bit, 32768 for 16)
         */    
        float amplitude;
        /** amplitude offset is -0.5 to 0.5 like amplitude it works in conjungtion
         * with the output resolution. so offset_amplitude = 0 means centered in
         * the output range
         * 0.5 would shift the waveform up so the range is 0.5*resolution to
         * 1.5*resolution
         */
        float offset_amplitude;
        float phase_offset;
        Resolution res;
        int sampling_freq_Hz;
        
    
    private:
        /** todo: at 96kHz sampling rate this would overflow after 12.4 hours
         *of continuous use and there could be a discontinuity in output as it
         *rolls over to 0, in the future you may reset the sample number when a
         *full period occurs to avoid this issue
         */
        uint32_t sample_number;
        volatile uint32_t total_samples;
        uint16_t last_sample;
        float normalized_val;
    
        //Keil only supports const int, so we must specify these another way
        //unfortunately we lose type checking
        #define DEFAULT_WAVE Waveform::eSinusoid
        #define DEFAULT_AMPLITUDE 1.0
        #define DEFAULT_RESOLUTION Resolution::e12Bit_Lower
        #define DEFAULT_AMPLITUDE_OFFSET 0.0
        #define DEFAULT_PHASE_OFFSET 0.0
    
        static const int DEFAULT_FREQ_HZ = 1000;        
        static const int DEFAULT_DURATION = DURATION_FOREVER;        
        static const int DEFAULT_SAMPLING_FREQ_HZ = 48000;
    
    //---CONSTRUCTOR---
    public:
        WaveGenerator(Waveform my_wave, int wave_freq_Hz, float val_n1_to_p1, int duration_ms, Resolution res);
        WaveGenerator(Waveform my_wave, int wave_freq_Hz, float val_n1_to_p1, int duration_ms);
        WaveGenerator(Waveform my_wave, int wave_freq_Hz, float val_n1_to_p1);
        WaveGenerator(Waveform my_wave, int wave_freq_Hz);
        WaveGenerator(Waveform my_wave);
        WaveGenerator();

    //---METHODS---
    public:
        void setWave(Waveform val);
        void setWaveFreqHz(int val);
        void setAmplitude(float val_0_to_1);
        void setResolution(Resolution val);
        void setSamplingFrequencyHz(int val);
        void setAmplitudeOffset(float val_0_to_1);
        void setPhaseOffset(float val_n1_to_p1);
        void setDurationMs(uint32_t val);
        void setDefaults();
    
        uint16_t generateNextSample();
        uint16_t generateSample(uint32_t sample_number);
    protected:
        
    private:
        
};
