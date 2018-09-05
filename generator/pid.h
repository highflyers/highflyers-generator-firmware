class Pid
{
public:
	Pid(double p, double i, double d, double minU, double maxU)
	{
		pP = p;
		pI = i;
		pD = d;
    uMin = minU;
    uMax = maxU;
		aggE = 0.0;
	};

	double loop(double e)
	{
		ret = 0.0;
		aggE += e;
    aggE = aggE > uMax/pI ? uMax/pI : aggE;
    aggE = aggE < -uMax/pI ? -uMax/pI : aggE;
    ret = (pP * e) + (pI * aggE);
		return ret;
	}

 void reset()
 {
  aggE = 0.0;
 }

public:
	double pP, pI, pD, uMin, uMax;
	double aggE, ret;
};
