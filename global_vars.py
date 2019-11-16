#!/usr/bin/env python

"""Harry Sarkis Terkanian; November 13, 2019; Script to demonstrate global vs local variables."""

a = 1


def f():
	a = 2	#assignment of a creates a local variable, masking the global
	print('f(): Local variable a created. Within function f the value of a is: ', a)


def g():
	#no assignment of a, the global variable a remains visible within the function
	print('g(): No variable assignement within function g, the value of a is: ', a)


def h():
	global a	#identify variable a within the function as a global
	a = 3
	print('h(): a is declared as a global variable within function h, the value of global a is:' , a)


if __name__ == '__main__':
	print('At the global level the value of a is: ', a, '\n')
	
	f()

	print('\nAt the global level the value of a remains: ', a, '\n')

	g()

	print('\nAt the global level the value of a remains: ', a, '\n')

	h()

	print('\nAt the global level the value of a has changed to: ', a, '\n')
