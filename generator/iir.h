class IIR 
{
public:
  IIR(int32_t coeff) { this->coeff = coeff; prevSample = 0; };
  int32_t newSample(int32_t sample) {
    double vout_currentD = (prevSample * coeff) + (sample * (256 - coeff));
    vout_currentD /= 256.0;
    currentSample = vout_currentD;
    prevSample = currentSample;
    return currentSample;
  }
  int32_t getValue() { return currentSample; }
private:
  int32_t coeff;
  int32_t prevSample, currentSample;
};
