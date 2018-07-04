class Pid
{
public:
	Pid(double p, double i, double d)
	{
		pP = p;
		pI = i;
		pD = d;
		aggE = 0.0;
	};

	double loop(double e)
	{
		double ret = 0.0;
		aggE += e;
		aggE = aggE > 1000.0/pI ? 1000.0/pI : aggE;
		aggE = aggE < 0 ? 0 : aggE;
		ret = pP * (e + pI * aggE);

		return ret;
	}

private:
	double pP, pI, pD;
	double aggE;
};