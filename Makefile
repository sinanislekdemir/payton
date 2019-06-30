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

docs:
	@pip install pdoc
	@PYTHONPATH=. pdoc --html-dir ./doc --html --overwrite --all-submodules payton
ifeq ($(UNAME), Linux)
		@xdg-open ./doc/payton/index.html
endif
ifeq ($(UNAME), Darwin)
		open ./doc/payton/index.html
endif

check:
	@echo "If mypy fails miserably, check your MYPYPATH to include SDL and stuff"
	@read -n 1 -s -r -p "Press any key to continue"
	@mypy payton

help:
	@echo "Make ROH Service"
	@echo "make requirements  # Install all requirements"
	@echo "make development   # Setup as development environment"
	@echo "make check         # Run all tests"
	@echo "make clean         # Remove __pycache__ dirs"
	@echo "make docs          # Generate Docs. Note, this will additionally install pdoc package which is not installed through requirements."

