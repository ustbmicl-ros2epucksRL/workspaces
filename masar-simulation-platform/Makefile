.PHONY: test lint compile docker-init docker-start docker-stop docker-status docker-logs docker-doctor

test:
	python3 -m pytest tests/unit -q

lint:
	python3 -m ruff check backend tests

compile:
	@paths="backend"; \
	if [ -d agents ]; then paths="$$paths agents"; fi; \
	python3 -m compileall -q $$paths

docker-init:
	@./deploy/platform.sh init

docker-start:
	@./deploy/platform.sh start

docker-stop:
	@./deploy/platform.sh stop

docker-status:
	@./deploy/platform.sh status

docker-logs:
	@./deploy/platform.sh logs $(SERVICE)

docker-doctor:
	@./deploy/platform.sh doctor
