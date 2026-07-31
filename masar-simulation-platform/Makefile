.PHONY: test lint compile

test:
	python3 -m pytest tests/unit -q

lint:
	python3 -m ruff check backend tests

compile:
	@paths="backend"; \
	if [ -d agents ]; then paths="$$paths agents"; fi; \
	python3 -m compileall -q $$paths
