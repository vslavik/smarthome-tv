FROM ghcr.io/astral-sh/uv:python3.14-alpine AS builder

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

# Synchronize DEPENDENCIES without the application itself.
WORKDIR /app
COPY pyproject.toml uv.lock /app/
RUN uv sync --locked --no-dev --no-install-project

# Now install the application itself, w/o dependencies.
COPY *.py /app/

FROM python:3.14-alpine
WORKDIR /app
COPY --from=builder /app /app

ENV PATH="/app/.venv/bin:$PATH"

ENV PS5_HOST=""
ENV MQTT_HOST=""

STOPSIGNAL SIGINT
CMD ["/app/ps5_mqtt.py"]
