#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <vector>

class MyMath {
private:
	const double kPi = 3.14156;

public:
	double sum(double a, double b);
	double square(double a);

	// TODO: implement these functions in calculator.cc
	double subtract(double a, double b);
	double divide(double a, double b);
	double sum(const std::vector<double>& values);
	double surface(double radius);
	std::vector<double> crossProduct(const std::vector<double>& a,
		const std::vector<double>& b);
	std::vector<std::vector<double>> matrixMultiply(
		const std::vector<std::vector<double>>& A,
		const std::vector<std::vector<double>>& B);
};

#endif  // CALCULATOR_H