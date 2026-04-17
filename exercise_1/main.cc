#include <iostream>
#include <vector>

#include "calculator.h"

int main() {
	MyMath math;

	std::cout << "sum(1.0, 1.0) = " << math.sum(1.0, 1.0) << std::endl;

	// Optional: uncomment these after implementing all functions
	/*
	std::vector<double> values = {1.0, 2.0, 3.0, 4.0};
	std::cout << "sum(vector) = " << math.sum(values) << std::endl;

	std::cout << "subtract(5.0, 2.0) = " << math.subtract(5.0, 2.0) << std::endl;
	std::cout << "divide(6.0, 2.0) = " << math.divide(6.0, 2.0) << std::endl;
	std::cout << "surface(2.0) = " << math.surface(2.0) << std::endl;

	std::vector<double> a = {1.0, 2.0, 3.0};
	std::vector<double> b = {4.0, 5.0, 6.0};
	std::vector<double> cross = math.crossProduct(a, b);

	std::cout << "crossProduct = {"
			  << cross[0] << ", " << cross[1] << ", " << cross[2] << "}"
			  << std::endl;

	std::vector<std::vector<double>> A = {{1.0, 2.0}, {3.0, 4.0}};
	std::vector<std::vector<double>> B = {{5.0, 6.0}, {7.0, 8.0}};
	std::vector<std::vector<double>> C = math.matrixMultiply(A, B);

	std::cout << "matrixMultiply result:" << std::endl;
	std::cout << C[0][0] << " " << C[0][1] << std::endl;
	std::cout << C[1][0] << " " << C[1][1] << std::endl;
	*/

	return 0;
}