# targets

.PHONY: default
default: help ;

UNAME := $(shell uname -s)

requirements:
	@pip install -r requirements.txt

development:
	@pip install --editable .

clean:
	@find . | grep -E "(__pycache__|\.pyc|\.pyo$\)" | xargs rm -rf

lint:
	@echo "If mypy fails miserably, check your MYPYPATH to include SDL and stuff"
	@mypy payton --disallow-untyped-calls --disallow-untyped-defs --disallow-incomplete-defs
	@ruff check payton

fmt:
	isort .
	ruff check payton --fix

help:
	@echo "Make Payton"
	@echo "make requirements  # Install all requirements"
	@echo "make development   # Setup as development environment"
	@echo "make lint          # Run code linters for errors"
	@echo "make clean         # Remove __pycache__ dirs"
	@echo "make fmt           # Format the source code using black + isort"
