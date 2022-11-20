#pragma once
#include<vector>
#include<iostream>
#include<string>
class Polynomial {
public:
	
	std::vector<int> coefficients;
	Polynomial(std::vector<int> C);
	Polynomial();
	int degree();
	Polynomial operator+(Polynomial& v);
	Polynomial operator-(Polynomial& v);
	Polynomial operator-();
	Polynomial operator*(Polynomial& v);
	Polynomial zero();
	Polynomial one();
	Polynomial x();
	Polynomial pow(int m);
	std::string toString();
	Polynomial shift(int m);
	int evaluate(int m);
};
