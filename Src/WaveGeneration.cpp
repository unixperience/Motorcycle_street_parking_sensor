#include "WaveGeneration.h"
#include <math.h>

#define PI 3.14159265

//---CONSTRUCTOR---
WaveGenerator::WaveGenerator(Waveform my_wave,
                            int wave_freq_Hz,
                            float amplitude, 
                            int duration_ms,
                            Resolution res)
{
    setDefaults();
    this->wave_type = my_wave;
    this->wave_freq_Hz = wave_freq_Hz;
    this->amplitude = amplitude;
    this->duration_ms = duration_ms;
    this->res = res;
}

WaveGenerator::WaveGenerator(Waveform my_wave,
                            int wave_freq_Hz,
                            float amplitude, 
                            int duration_ms)
{
    setDefaults();
    this->wave_type = my_wave;
    this->wave_freq_Hz = wave_freq_Hz;
    this->amplitude = amplitude;
    this->duration_ms = duration_ms;
}

WaveGenerator::WaveGenerator(Waveform my_wave, int wave_freq_Hz, float amplitude)
{
    setDefaults();
    this->wave_type = my_wave;
    this->wave_freq_Hz = wave_freq_Hz;
    this->amplitude = amplitude;
}

WaveGenerator::WaveGenerator(Waveform my_wave, int wave_freq_Hz)
{
    setDefaults();
    this->wave_type = my_wave;
    this->wave_freq_Hz = wave_freq_Hz;
}

WaveGenerator::WaveGenerator(Waveform my_wave)
{
    setDefaults();
    this->wave_type = my_wave;
}

WaveGenerator::WaveGenerator()
{
    setDefaults();
}

//---ACCESORS---
void WaveGenerator::setWave(Waveform val)
{
    wave_type = val;
    
    this->sample_number = 0;    
    this->total_samples = 0;
}

void WaveGenerator::setWaveFreqHz(int val)
{
        if (val <0)
    {
        wave_freq_Hz = -1*val;
    }
    else
    {
        wave_freq_Hz = val;
    }
}

void WaveGenerator::setAmplitude(float val_0_to_1)
{
    if (val_0_to_1  < 0)
    {
        amplitude = 0.0;
    }
    else if (val_0_to_1 > 1)
    {
        amplitude = 1.0;
    }
    else
    {
        amplitude = val_0_to_1;
    }
}

void WaveGenerator::setResolution(Resolution val)
{
    res = val;
    this->sample_number = 0;    
    this->total_samples = 0;
}

void WaveGenerator::setSamplingFrequencyHz(int val)
{
    if (val <0)
    {
        sampling_freq_Hz = -1*val;
    }
    else
    {
        sampling_freq_Hz = val;
    }
    this->sample_number = 0;    
    this->total_samples = 0;
}

void WaveGenerator::setAmplitudeOffset(float val_n1_to_p1)
{
    if (val_n1_to_p1  < -1.0)
    {
        offset_amplitude = -1.0;
    }
    else if (val_n1_to_p1 > 1)
    {
        offset_amplitude = 1.0;
    }
    else
    {
        offset_amplitude = val_n1_to_p1;
    }
}

void WaveGenerator::setPhaseOffset(float val_n1_to_p1)
{
    if (val_n1_to_p1  < -1.0)
    {
        phase_offset = -1.0;
    }
    else if (val_n1_to_p1 > 1)
    {
        phase_offset = 1.0;
    }
    else
    {
        phase_offset = val_n1_to_p1;
    }
}

void WaveGenerator::setDurationMs(uint32_t val)
{
    duration_ms = val;
    
    //you may change this in the future to start from the current sample and
    //roll over
    total_samples = 0;
}

void WaveGenerator::setDefaults()
{
    this->wave_type = DEFAULT_WAVE;
    this->wave_freq_Hz = DEFAULT_FREQ_HZ;
    this->amplitude = DEFAULT_AMPLITUDE;
    this->duration_ms = DEFAULT_DURATION;
    this->res = DEFAULT_RESOLUTION;
    this->sampling_freq_Hz = DEFAULT_SAMPLING_FREQ_HZ;
    this->offset_amplitude = DEFAULT_AMPLITUDE_OFFSET;
    this->phase_offset = DEFAULT_PHASE_OFFSET;
    this->sample_number = 0;    
    this->total_samples = 0;
}
//---METHODS---
uint16_t WaveGenerator::generateNextSample()
{
    int32_t next_sample;
    float my_offset;
    //float normalized_val;
    
    //make sure we are in the valid time range
     if ((duration_ms != DURATION_FOREVER) 
         &&((float)total_samples/sampling_freq_Hz) > (float)duration_ms/1000 )
    {
        //we will return a value in the middle of the waveform when it is off
        return (amplitude * (uint16_t)res * (my_offset + 0.5));
    }
    
    total_samples++;
    
    //all waveforms should nomially output values between 0-1
    //take input of samples_number
    //if there is periodicity, the wave should also reset "sample_number" to 0 
    //to avoid overflow
    if (wave_type == Waveform::eSinusoid)
    {
        normalized_val = 0.5 //cos is -1 to +1, so swing is 2, we need it to be 1
                       * cos( 2*PI*((float)wave_freq_Hz/sampling_freq_Hz) * sample_number 
                              + 2*PI*phase_offset)
                       + 0.5;
        
        my_offset = offset_amplitude;
    }
    else if (wave_type == Waveform::eSawtooth)
    {
        normalized_val = fmod(sample_number*(float)wave_freq_Hz/sampling_freq_Hz + phase_offset, 1);
        
        my_offset = offset_amplitude;
    }
    else if (wave_type ==  Waveform::eTriangle)
    {
        //std::abs says call is ambiguous (likely due to modulus operator) even
        //if you cast to (float), so I seperated it into 2 operations
        normalized_val = (2*(phase_offset+sample_number*(float)wave_freq_Hz/sampling_freq_Hz));      
        normalized_val = fmod(normalized_val, 2) -1;        
        normalized_val = std::abs(normalized_val);
        
        my_offset = offset_amplitude;
    }
    else if (wave_type ==  Waveform::eSquare)
    {
        if ( (sample_number+phase_offset) < ((float)sampling_freq_Hz/wave_freq_Hz)/2)
        {
            normalized_val = 0;
        }
        else
        {
            normalized_val = 1;
        }
        
        my_offset = offset_amplitude;
    }
    else
    {
        return 0;
    }
    
    //if there is a 
    
    if (res == Resolution::e12Bit_Upper)
    {
        //modifies value from 0-1 normalized to the selected resolution
        // adds amplitude offset
        next_sample = amplitude * (uint16_t)Resolution::e12Bit_Lower * (my_offset + normalized_val);
            
        //clipping of values
        if (next_sample > (uint16_t)Resolution::e12Bit_Lower )
        {
            next_sample = (uint16_t)Resolution::e12Bit_Lower ;
        }
        else if (next_sample < 0)
        {
            next_sample = 0;
        }
        
        //shift it up to the top 12 bits
        next_sample = next_sample << 4;
    }
    else
    {
        //modifies value from 0-1 normalized to the selected resolution
        // adds amplitude offset
        next_sample = amplitude * (uint16_t)res * (my_offset + normalized_val);
            
        //clipping of values
        if (next_sample > (uint16_t)res)
        {
            next_sample = (uint16_t)res;
        }
        else if (next_sample < 0)
        {
            next_sample = 0;
        }
    }
    
    //prepare for next sample
    sample_number++;    
    
    last_sample = next_sample &0x0000FFFF;
    return next_sample;
}

uint16_t WaveGenerator::generateSample(uint32_t sample_number)
{
    this->sample_number = sample_number;
    
    return(generateNextSample());
}
