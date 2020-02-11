#ifndef DSL_RANDGEN_H
#define DSL_RANDGEN_H

// {{SMILE_PUBLIC_HEADER}}

class DSL_randGen
{
public:
	DSL_randGen(unsigned initSeed = 0);
	void Init(unsigned initSeed = 0);

	void Spin(int count);

	// returns double from [0..1)
	double GetDouble();
	
	// returns int from [0..range-1]
	int GetInt(unsigned range);
	int operator()(unsigned range) { return GetInt(range); }

	double Uniform(double lo, double hi);
	double Normal(double mean, double stdDev);
	double TruncNormal(double mean, double stdDev, double vmin);
	double Gamma(double alpha, double beta);
	double Weibull(double lambda, double k);
	double Beta(double a, double b);
	double Triangular(double vmin, double vmode, double vmax);
	double Bernoulli(double p);
	double Exponential(double lambda);
	int Binomial(int n, double p);
    int Poisson(double lambda);
	double Custom(int n, const double *v, const double *p);
	double Steps(int n, const double *v, const double *p);
	
	// returns int from [0..count-1]
	// probs are assumed to sum up to 1.0
	static int Roulette(double x, const double *probs, int count);
	int Roulette(const double *probs, int count)
	{
		return Roulette(GetDouble(), probs, count);
	}

private:
	unsigned x, y, z, c;
	double spareNormal;
	bool normalAvailable;
	
	unsigned Next();
};

#endif
