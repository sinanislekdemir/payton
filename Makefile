# targets

.PHONY: default docs
default: help ;

UNAME := $(shell uname -s)
DOCSPATH ?= "./docs"

requirements:
	@pip install -r requirements.txt

development:
	@python setup.py develop

clean:
	@find . | grep -E "(__pycache__|\.pyc|\.pyo$\)" | xargs rm -rf

docs:
	@pip install pdoc3
	@PYTHONPATH=. pdoc3 --output-dir $(DOCSPATH) --html --force payton --template-dir ../pdoc_templates
ifeq ($(UNAME), Linux)
	@xdg-open $(DOCSPATH)/payton/index.html
endif
ifeq ($(UNAME), Darwin)
	open $(DOCSPATH)/payton/index.html
endif

check:
	@echo "If mypy fails miserably, check your MYPYPATH to include SDL and stuff"
	@mypy payton

help:
	@echo "Make ROH Service"
	@echo "make requirements  # Install all requirements"
	@echo "make development   # Setup as development environment"
	@echo "make check         # Run all tests"
	@echo "make clean         # Remove __pycache__ dirs"
	@echo "make docs          # Generate Docs. Note, this will additionally install pdoc "
	@echo "                   # package which is not installed through requirements."
	@echo "                   # Use DOCSPATH to set the folder where to generate the documentation. Defaults to ./docs"

