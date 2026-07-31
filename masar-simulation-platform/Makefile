.PHONY: test lint compile

test:
	python3 -m pytest tests/unit -q

lint:
	python3 -m ruff check backend tests

compile:
	python3 -m compileall -q backend agents
