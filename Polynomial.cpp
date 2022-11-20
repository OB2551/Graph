#include "Polynomial.h"
#include <vector>
#include <iostream>
#include<string>
#include<algorithm>
#include <functional>
/*basic class for polynomials*/
Polynomial::Polynomial(std::vector<int> C)
{
	coefficients = C;
	while (coefficients.back() == 0 && coefficients.size() > 1) { coefficients.pop_back(); }


}
Polynomial::Polynomial() {}
int Polynomial::degree() {
	return coefficients.size() - 1;
}
Polynomial Polynomial::operator+(Polynomial& v) {
	std::vector<int> c1 = this->coefficients, *c2 = &v.coefficients;
	auto size = std::max(c1.size(), c2->size());
	c1.resize(size);
	c2->resize(size);
	auto scab = std::vector<int>(size);
	int i = 0;
	while (i < size) {
		scab[i] = c1[i] + c2->at(i);
		i++;
	}
	return Polynomial(scab);

}
Polynomial Polynomial::operator-(Polynomial& v) {
	std::vector<int> c1 = this->coefficients, c2 = v.coefficients;
	auto size = std::max(c1.size(), c2.size());
	c1.resize(size);
	c2.resize(size);
	auto scab = std::vector<int>(size);
	int i = 0;
	while (i < size) {
		scab[i] = (c1[i] - c2[i]);
		i++;
	}
	return Polynomial(scab);
}
Polynomial Polynomial::operator-() {
	int i = 0, sz = this->coefficients.size();
	std::vector<int> d = this->coefficients,
		scab = std::vector<int>(sz);
	while (i < sz) {
		scab[i] = -d[i];
		i++;
	}
	return Polynomial(scab);
}
Polynomial Polynomial::operator*(Polynomial& v) {
	int m = v.degree(), n = this->degree();
	int deg = m + n;
	std::vector<int> Zc(deg + 1, 0);
	for (int i = 0; i <= m; i++) {
		for (int j = 0; j <= n; j++) {
			Zc[i + j] = Zc[i + j] + (this->coefficients[j] * v.coefficients[i]);
		}
	}
	return Polynomial(Zc);
}
Polynomial Polynomial::zero() {
	std::vector<int> Zero{ 0}; return Polynomial(Zero);
}
Polynomial Polynomial::one() { std::vector<int> One{ 1 }; return Polynomial(One); }
Polynomial Polynomial::x() { std::vector<int> x{ 0,1 }; return Polynomial(x); }
Polynomial Polynomial::pow(int m)
{
	Polynomial P = this->one();
	Polynomial R(this->coefficients);
	while (m > 0)
	{
		if (m % 2 == 1)
		{
			P = (P * R);
		}
		m = m / 2;
		R = R * R;
	}
	return P;
}
std::string Polynomial::toString() {
	std::string s;
	for (int i = this->degree(); 0<=i ; i--) {
		if (i < this->degree())
		{
			if (this->coefficients[i] != 0)
			{
				s.append(std::to_string(this->coefficients[i]) + "x^" + std::to_string(i) + " + ");
			}
		}
		if (i == this->degree()) { s.append(std::to_string(this->coefficients[i]) + "x^" + std::to_string(i)); }
	}
	return s;
}

Polynomial Polynomial::shift(int m) {
	std::vector<int> c1 = this->coefficients;
	c1.insert(c1.begin(), m, 0);
	return Polynomial(c1);
}
int Polynomial::evaluate(int m) {
	int res = 0;
	for (int i = 0; i <= degree(); i++) {
		res = res + (this->coefficients[i] * (std::pow(m, i)));
	}
	return res;
}