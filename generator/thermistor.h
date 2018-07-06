class Thermistor
{
public:
  Thermistor(double nominal, double reference, double gainErr)
  {
    this->nominal = nominal;
    this->reference = reference;
    this->gainErr = gainErr;
  }
  double calculate(int adcValue)
  {
    double voltage = (double)adcValue / 1023.0 * refV;
    double I = (refV - voltage) / reference;
    double resistance = (refV / I) - reference;
    double rDiffRel = -(resistance - nominal) / nominal;
//    Serial.print(resistance);
//    Serial.print(" ");
//    Serial.print(nominal);
//    Serial.print(" ");
    Serial.print(rDiffRel*1000);
    Serial.print(" ");
    return rDiffRel*1000;
  }
private:
  double gainErr;
  double nominal, reference;
  double refV = 5000;
};

