
/*
Parser data structures

Zacharia Crumley

This file contains the data structure definitions and typedef's that I use in my L-system parser
*/

#ifndef __parser_data_structures_h__
#define __parser_data_structures_h__

#include <vector>

using namespace std;

//a single symbol in the L-system with associated value (if it has one)
//if the associated value is in fact a formula, that is stored as well
struct Symbol
{
	//the symbol's textual representation
	string name;
	
	//if it has a set value, then this is it.
	double value;
	//this stores if the symbol does have a start value
	bool hasValue;
	
	//the formula associated with the value
	string formula;
	//this stores if the symbol does have a formula
	bool hasFormula;
};

//a sequence of symbols
typedef vector<Symbol> Word;

//a single rule of the L-system, ie a production of one symbol to multiple
//also stores context-sensitive info, 
struct Rule
{
	//the two basic components
	Symbol LHS;
	
	//these store whether the rule has a predecessor and/or successor
	bool hasPred;
	bool hasSucc;
	//these store the predecessor/successor if they exist
	Symbol predecessor;
	Symbol successor;
	
	//this stores the probabilities of each of the possible RHS's
	vector<double> wordProbs;
	//this vector stores all of the possible RHS's
	vector<Word> possWords;
};

//this is for holding all of the rules in one data structure
typedef vector<Rule> LSystem;

#endif
