# L-System Parser - makefile
# Zacharia Crumley
# This is the makefile for my L-system parser

CC=g++

#put everything together into an executable
main: tokenizer
	$(CC) -g -o LParser.o main.cpp

#use flex to make the tokenizer source code
tokenizer:
	flex lparser.l

#clean rule
clean:
	rm -f *.c *.o