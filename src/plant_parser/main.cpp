/*
L-System Parser - main.cpp

Zacharia Crumley

This file has the main method for the parser program.
*/

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>

#include "lex.yy.c"
#include "parser_data_structures.h"

using namespace std;

//this stores all of rules in the L-system
LSystem lsystem;
//this stores the start symbol of the L-system
Symbol start;

vector<int> inputStreamTypes;
vector<string> inputStreamText;

//stores whether to be verbose or not
//had to put this here because stuff outside main needs to access it
bool verbose = false;

/*
this converts a double into a string
*/
string numToString(double n)
{
	stringstream in;
	in << n;
	return in.str();
}


/*
sets up the flex code to read from the input file
*/
void initializeInput(string filename)
{
	FILE* input = fopen(filename.c_str(), "r");
	yyrestart(input);
}


/*
uses flex to go through all of the input and put it into vectors for easier processing.
the first vector stores the type of each input token.
the second vector stores the actual text of each input token.
*/
void getInputStreams()
{
	int in = yylex();
	while (in != -1)
	{
		inputStreamTypes.push_back(in);
		inputStreamText.push_back(yytext);
		in = yylex();
	}
}


/*
makes a string of the contents of the vectors created by getInputStreams()
*/
void displayInputStreams()
{
	cout << "Tokens:\n";
	int temp;
	string label;
	for (vector<int>::iterator i = inputStreamTypes.begin(); i != inputStreamTypes.end(); i++)
	{
		temp = *i;
		switch (temp)
		{
			case 1:
				label = "PROBABILITY_SYMBOL";
				break;
			case 2:
				label = "ASSIGN";
				break;
			case 3:
				label = "CONTEXT";
				break;
			case 4:
				label = "PREDECESSOR_SYMBOL";
				break;
			case 5:
				label = "SUCCESSOR_SYMBOL";
				break;
			case 6:
				label = "PLUS";
				break;
			case 7:
				label = "MINUS";
				break;
			case 8:
				label = "DIVIDE";
				break;
			case 9:
				label = "MULTIPLY";
				break;
			case 10:
				label = "RULE_SEPARATOR";
				break;
			case 11:
				label = "START_SYMBOL";
				break;
			case 12:
				label = "OPEN_BRACKET";
				break;
			case 13:
				label = "CLOSE_BRACKET";
				break;
			case 14:
				label = "STACK_PUSH";
				break;
			case 15:
				label = "STACK_POP";
				break;
			case 16:
				label = "WORD";
				break;
			case 17:
				label = "NUMBER";
				break;
		}
		cout << label << " ";
	}
	/*cout << "\nText:\n";
	for (vector<string>::iterator i = inputStreamText.begin(); i != inputStreamText.end(); i++)
	{
		cout << *i << " ";
	}*/
	cout << "\n";
}


/*
this method converts the input (stored in the vectors inputStreamTypes and inputStreamText) into
the data structures to store the L-system's rules.
*/
void parseInput()
{
	/*
	first divide up the input into it's separate rules so that we can process each rule one at a time
	also, get the starting symbol while we're doing this
	*/
	
	//stores the start and end point of each rule (inclusive)
	vector<int> indexes;
	//a temp used for counting index positions of the start and end of each rule
	int index = 0;
	//keeps track of how many rules there are in the L-system
	int ruleCount = 1;
	//stores the location of the start symbol
	int startSymbolPos;
	
	//push back the start of the first rule (it has to be the first thing)
	indexes.push_back(index);
	
	for (vector<int>::iterator i = inputStreamTypes.begin(); i != inputStreamTypes.end(); i++)
	{
		//if we come across a rule separator
		if ((*i) == RULE_SEPARATOR)
		{
			//then the last thing was the final symbol in a rule so we add it to the rule delimiter tracking index
			indexes.push_back(index - 1);
			indexes.push_back(index + 1);
			ruleCount++;
		}
		if ((*i) == START_SYMBOL)
		{
			indexes.push_back(index -1);
			startSymbolPos = index + 1;
		}
		index++;
	}
	
	/*
	set the start symbol to the value given in the rules input
	*/
	
	//get the start symbol's value
	start.name = inputStreamText.at(startSymbolPos);
	//if there's something after the start symbol (ie it has a parameter), then get it's associated parameter
	if (inputStreamText.size() > (startSymbolPos+1))
	{
		start.value = atof( (inputStreamText.at(startSymbolPos + 2)).c_str() );
		start.hasValue = true;
	}
	
	if (verbose)
	{
		cout << "Start symbol: " << start.name << "\n";
		if (start.hasValue)
			cout << "with value: " << start.value << "\n";
		cout << "\n";
	}
	
	/*
	next process each rule individually
	*/
	
	if (verbose)
	{
		//display the rules
		cout << "Number of rules: " << ruleCount << "\n";
		cout << "The rules are:\n";
		for (vector<int>::iterator i = indexes.begin(); i != indexes.end(); i++)
		{
			int start = *i;
			i++;
			int end = *i;
			
			for (int j = start; j <= end; j++)
				cout << inputStreamText.at(j) << " ";
			cout << "\n";
		}
		cout << "\n";
	}
	
	//go through all the rules
	for (vector<int>::iterator i = indexes.begin(); i != indexes.end(); i++)
	{
		//for holding the rule currently being worked on
		Rule temp;
		//temp.numWords = 0;
		temp.hasPred = false;
		temp.hasSucc = false;
		temp.LHS.name = "";
		temp.LHS.hasValue = false;
		temp.LHS.hasFormula = false;
		temp.wordProbs.clear();
		temp.possWords.clear();
		
		int start = *i;
		i++;
		int end = *i;
		
		//this stores the current index of the rule, initialized to the beginning of the rule
		int curr = start;
		
		//get the LHS of the rule
		temp.LHS.name = inputStreamText.at(curr);
		curr++;
		//if the LHS has a bracket (which is not allowed by the way), then skip past the bracket
		if (inputStreamTypes.at(curr) == OPEN_BRACKET)
		{
			while (inputStreamTypes.at(curr) != CLOSE_BRACKET)
				curr++;
			curr++;
		}
		
		//NOTE: it all starts going pear shaped after this point.
		
		//loop through the symbols in the rule
		while (curr <= end)
		{
			//if we come across a symbol indicating the rule has context sensitivity
			if (inputStreamTypes.at(curr) == CONTEXT)
			{
				curr++;
				if (inputStreamTypes.at(curr) == PREDECESSOR_SYMBOL)
				{
					temp.hasPred = true;
					curr++;
					if (inputStreamTypes.at(curr) == WORD)
					{
						temp.predecessor.name = inputStreamText.at(curr);
					}
					else
						throw 1;
					
					if (inputStreamTypes.at(curr+1) == SUCCESSOR_SYMBOL)
					{
						temp.hasSucc = true;
						curr += 2;
						if (inputStreamTypes.at(curr) == WORD)
						{
							temp.successor.name = inputStreamText.at(curr);
						}
						else
							throw 1;
					}
					else
						temp.hasSucc = false;
				}
				else if (inputStreamTypes.at(curr) == SUCCESSOR_SYMBOL)
				{
					temp.hasSucc = true;
					temp.hasPred = false;
					curr++;
					if (inputStreamTypes.at(curr) == WORD)
					{
						temp.successor.name = inputStreamText.at(curr);
					}
					else
						throw 1;
				}
				
				//FIXME: need to set the bools in temp
			}
			//if we come across a symbol indicating the rule has stochastic elements
			else if (inputStreamTypes.at(curr) == PROBABILITY_SYMBOL)
			{
				if (inputStreamTypes.at(curr+1) == NUMBER)
				{
					curr++;
					temp.wordProbs.push_back( atof(inputStreamText.at(curr).c_str()) );
				}
				else
					throw 1;
				
				if (inputStreamTypes.at(curr+1) != ASSIGN)
					throw 1;
			}
			//if we come across a symbol indicating that the RHS of the production is next
			else if (inputStreamTypes.at(curr) == ASSIGN)
			{
				Symbol tempSymbol;
				Word tempWord;
				
				curr++;
				
				while (inputStreamTypes.at(curr) == WORD)
				{
					tempSymbol.name = "";
					tempSymbol.hasValue = false;
					tempSymbol.value = 0;
					tempSymbol.hasFormula = false;
					tempSymbol.formula = "";
					
					tempSymbol.name = inputStreamText.at(curr);
					if (inputStreamTypes.at(curr+1) == OPEN_BRACKET)
					{
						curr += 2;
						
						if ((inputStreamTypes.at(curr) == NUMBER) && (inputStreamTypes.at(curr+1) == CLOSE_BRACKET))
						{
							tempSymbol.hasValue = true;
							tempSymbol.hasFormula = false;
							tempSymbol.value = atof( inputStreamText.at(curr).c_str() );
							tempSymbol.formula = "";
							curr++;
						}
						else
						{
							tempSymbol.hasValue = false;
							tempSymbol.value = 0;
							tempSymbol.hasFormula = true;
							
							while (inputStreamTypes.at(curr) != CLOSE_BRACKET)
							{
								tempSymbol.formula.append( inputStreamText.at(curr) + " " );
								curr++;
							}
						}
					}
					tempWord.push_back(tempSymbol);
					curr++;
				}
				temp.possWords.push_back(tempWord);
				curr--;
			}
			curr++;
		}
		
		lsystem.push_back(temp);
	}
}


/*
this outputs the rules of the L-system to the screen.
*/
void outputLSystem()
{
	cout << "The L-system:\n\n";
	
	//cout <<  << "\n"; //FIXME: temp
	
	for (LSystem::iterator i = lsystem.begin(); i != lsystem.end(); i++)
	{
		cout << "RULE:\n";
		cout << "The LHS: " << (*i).LHS.name << "\n";
		if ((*i).hasPred)
			cout << "It's required predecessor: " << (*i).predecessor.name << "\n";
		if ((*i).hasSucc)
			cout << "It's required successor: " << (*i).successor.name << "\n";
		if ((*i).possWords.size() <= 1)
		{
			cout << "the RHS: ";
			for (Word::iterator j = (*i).possWords.at(0).begin(); j != (*i).possWords.at(0).end(); j++)
			{
				cout << (*j).name;
				if ((*j).hasValue)
					cout << "(" << (*j).value << ")";
				else if ((*j).hasFormula)
					cout << "(" << (*j).formula << ")";
				cout << " ";
			}
		}
		else
		{
			for (int j = 0; j < (*i).possWords.size(); j++)
			{
				cout << "Probability: " << (*i).wordProbs.at(j) << " - ";
				for (Word::iterator k = (*i).possWords.at(j).begin(); k != (*i).possWords.at(j).end(); k++)
				{
					cout << (*k).name;
					if ((*k).hasValue)
						cout << "(" << (*k).value << ")";
					else if ((*k).hasFormula)
						cout << "(" << (*k).formula << ")";
					cout << " ";
				}
				cout << "| ";
			}
		}
		cout << "\n\n";
	}
}


/*
FIXME: add the ability to recognize set values from a file or something in this (as suggested in the email
FIXME: This doesn't follow proper precedence yet. That's something to be added later. At the moment it just does left to right evaluation
*/
double evalFormula(double input, string formula)
{
	double runningValue = 0.0;
	stringstream in;
	in << formula;
	string temp;
	double tempVal = 0;
	char lastOperator = '+';
	
	while (!in.eof())
	{
		in >> temp >> ws;
		tempVal = atof(temp.c_str());
		
		if ((tempVal == 0.0) && (temp.at(0) != '0'))
		{
			if ((temp == "+") || (temp == "-") || (temp == "*") || (temp == "/"))
				lastOperator = temp.at(0);
			else
			{
				if ((temp == "n") || (temp == "N"))
					tempVal = input;
			
				switch (lastOperator)
				{
					case '+':
						runningValue += tempVal;
						break;
					case '-':
						runningValue -= tempVal;
						break;
					case '*':
						runningValue *= tempVal;
						break;
					case '/':
						runningValue /= tempVal;
						break;
				}
			}
		}
		else
		{
			switch (lastOperator)
			{
				case '+':
					runningValue += tempVal;
					break;
				case '-':
					runningValue -= tempVal;
					break;
				case '*':
					runningValue *= tempVal;
					break;
				case '/':
					runningValue /= tempVal;
					break;
			}
		}
	}
	
	return runningValue;
}


/*
this method uses the L-system to generate strings and outputs them to a text file
*/
void generate(string filename, int num, int maxIterations)
{
	ofstream out(filename.c_str());
	if (out)
	{
		//this stores the output sentence while it's being worked on
		Word output;
		//this is the Word that stores the updated output word in each iteration and overrides output at the end of each iteration
		Word working;
		
		for (int i = 0; i < num; i++)
		{
			output.clear();
			output.push_back(start);
			
			bool nonterminalsExist = true;
			bool hitMaxIterations = false;
			int iterations = 0;
			
			while ((nonterminalsExist) && (!hitMaxIterations))
			{
				//do the transformation
				for (Word::iterator j = output.begin(); j != output.end(); j++)
				{
					bool transformed = false;
					for (LSystem::iterator k = lsystem.begin(); k != lsystem.end(); k++)
					{
						if ((*j).name == (*k).LHS.name)
						{
							//this block of code determines if the rule matches any context-sensitivity criteria it may have
							//the bool contextMatches is used to track whether the context matches. It starts off true and if
							//there are any context sensitive things that don't match in the current word then it's set to false.
							bool contextMatches = true;
							if ((*k).hasPred)
							{
								if (j != output.begin())
								{
									j--;
									if ((*j).name != (*k).predecessor.name)
										contextMatches = false;
									j++;
								}
								else
									contextMatches = false;
							}
							if ((*k).hasSucc)
							{
								if (j != --output.end())
								{
									j++;
									if ((*j).name != (*k).successor.name)
										contextMatches = false;
									j--;
								}
								else
									contextMatches = false;
							}
							
							//if the context sensitivity is correct, or there isn't any context sensitivity
							if (contextMatches)
							{
								transformed = true;
								int chosenWord;
								if ( (*k).possWords.size() <= 1 )
									chosenWord = 0;
								else
								{
									double randNum = (rand() % 1000000000) / (double)(1000000000);
									double runningTot = 0.0;
									chosenWord = 0;
									
									for (int l = 0; l < (*k).possWords.size(); l++)
									{
										runningTot += (*k).wordProbs.at(l);
										if (randNum <= runningTot)
										{
											chosenWord = l;
											break;
										}
									}
								}
								
								for (Word::iterator l = (*k).possWords.at(chosenWord).begin(); l != (*k).possWords.at(chosenWord).end(); l++)
								{
									Symbol temp;
									temp.name = (*l).name;
									if ((*l).hasValue)
									{
										temp.hasValue = true;
										temp.hasFormula = false;
										temp.value = (*l).value;
									}
									else if ((*l).hasFormula)
									{
										temp.hasValue = true;
										temp.hasFormula = false;
										temp.value = evalFormula( (*j).value, (*l).formula );
									}
									else
									{
										temp.hasValue = false;
										temp.hasFormula = false;
									}
									working.push_back(temp);
								}
							}
						}
					}
					if (!transformed)
					{
						working.push_back(*j);
					}
				}
				
				//check if any nonterminals remain and change the var if they don't
				nonterminalsExist = false;
				for (Word::iterator j = working.begin(); j != working.end(); j++)
				{
					for (LSystem::iterator k = lsystem.begin(); k != lsystem.end(); k++)
					{
						if ((*j).name == (*k).LHS.name)
							nonterminalsExist = true;
					}
				}
				
				//copy the latest iteration's results into the output word
				output.clear();
				for (Word::iterator j = working.begin(); j != working.end(); j++)
				{
					output.push_back(*j);
				}
				working.clear();
				
				//increase the iterations counter
				iterations++;
				
				if (verbose)
				{
					cout << "iteration: " << iterations << " Output: ";
					for (Word::iterator i = output.begin(); i != output.end(); i++)
					{
						cout << (*i).name << " ";
						if ((*i).hasValue)
							cout << "( " << (*i).value << " ) ";
					}
					cout << "\n";
				}
				
				//check for whether we've hit max # if iterations, and if we care change the bool to stop looping
				if ((maxIterations >= 0) && (iterations >= maxIterations))
					hitMaxIterations = true;
			}
			
			for (Word::iterator i = output.begin(); i != output.end(); i++)
			{
				out << (*i).name << " ";
				if ((*i).hasValue)
					out << "( " << (*i).value << " ) ";
			}
			
			out << "\n\n";
		}
		out.close();
	}
	else
	{
		throw 1;
	}
}


/*
the main method of the program
*/
int main(int argc, char** argv)
{
	try
	{
		string inputFile = "input";
		string outputFile = "output";
		int numSentences = 1;
		int maxIterations = -1;
		
		srand( time(NULL) );
		
		//parse the arguments
		try
		{
			if (argc <= 1)
			{
				if (verbose)
					cout << "No arguments given. Using only default values.\n\n";
			}
			else
			{
				int count = 1;
				while (count < argc)
				{
					if ( string(argv[count] ) == "-o")
					{
						outputFile = argv[count+1];
						count += 2;
					}
					else if ( string(argv[count] ) == "-n")
					{
						numSentences = atoi(argv[count+1]);
						if (numSentences < 0)
						{
							cout << "number of sentences to generate must be positive.\n";
							throw 1;
						}
						count += 2;
					}
					else if ( string(argv[count] ) == "-m")
					{
						maxIterations = atoi(argv[count+1]);
						if (numSentences < 0)
						{
							cout << "WARNING: a negative max iterations means infinite to this program.\n";
						}
						count += 2;
					}
					else if ( string(argv[count] ) == "-v")
					{
						verbose = true;
						count++;
					}
					else if ( string(argv[count] ) == "-i")
					{
						inputFile = argv[count+1];
						count += 2;
					}
					else
					{
						cout << "Unrecognized argument: " << argv[count] << "\n";
						throw 1;
					}
				}
				
				if (verbose)
				{
					cout << "\n===OPTIONS===\n";
					cout << "Input file: " << inputFile << "\n";
					cout << "Output file: " << outputFile << "\n";
					cout << "Number of sentences to generate: " << numSentences << "\n";
					cout << "Maximum number of iterations allowed: " << maxIterations << "\n";
					cout << "verbosity level: " << verbose << "\n";
					cout << "\n";
				}
			}
		}
		catch (...)
		{
			//TODO: output the format the argument is meant to be in here
			cout << "Error parsing arguments. Please correct the problem.\nProgram terminating.\n\n";
			exit(1);
		}
		
		//initialize the input file
		try
		{
			if (verbose)
				cout << "===opening the input file " << inputFile << " for reading===\n";
			initializeInput(inputFile);
		}
		catch (...)
		{
			cout << "Error opening input file: " << inputFile << "\nProgram terminating.\n\n";
			exit(1);
		}
		
		//parse the input and create the L-system data structures
		try
		{
			if (verbose)
				cout << "===reading the input file===\n\n";
			getInputStreams();
			if (verbose)
				cout << "\n";
			if (verbose)
				displayInputStreams();
			if (verbose)
				cout << "\n===parsing the input into rules===\n\n";
			parseInput();
			if (verbose)
				outputLSystem();
		}
		catch (...)
		{
			cout << "Error parsing the rules from the input file.\nProgram terminating.\n\n";
			exit(1);
		}
		
		//generate the output file according to the various options.
		try
		{
			if (verbose)
				cout << "===generating the output sentences and writing them to file===\n\n";
			generate(outputFile, numSentences, maxIterations);
		}
		catch (...)
		{
			cout << "Error generating and outputting the sentences.\nProgram terminating.\n\n";
			exit(1);
		}
	
		return 0;
	}
	catch (...)
	{
		cout << "An error occured during the program's execution. Program terminating.\n\n";
		exit(1);
	}
}
