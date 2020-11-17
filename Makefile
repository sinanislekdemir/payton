# targets

.PHONY: default
default: help ;

UNAME := $(shell uname -s)

requirements:
	@pip install -r requirements.txt

development:
	@python setup.py develop

clean:
	@find . | grep -E "(__pycache__|\.pyc|\.pyo$\)" | xargs rm -rf

check:
	@echo "If mypy fails miserably, check your MYPYPATH to include SDL and stuff"
	@mypy payton
	@flake8 payton
	@pylama payton --options ./setup.cfg

help:
	@echo "Make Payton"
	@echo "make requirements  # Install all requirements"
	@echo "make development   # Setup as development environment"
	@echo "make check         # Run all tests"
	@echo "make clean         # Remove __pycache__ dirs"
