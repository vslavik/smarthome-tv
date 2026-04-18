FROM ghcr.io/astral-sh/uv:python3.14-alpine AS builder

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

WORKDIR /app
COPY pyproject.toml uv.lock /app/
RUN uv sync --locked --no-dev --no-install-project

COPY *.py /app/

FROM python:3.14-alpine
WORKDIR /app
COPY --from=builder /app /app

ENV PATH="/app/.venv/bin:$PATH"

ENV STREAMING_SWITCH_HOST=""
ENV STREAMING_MONITORED_PORT=""
ENV MQTT_HOST=""
ENV SNMP_COMMUNITY="public"

STOPSIGNAL SIGINT
CMD ["/app/streaming_mqtt.py"]
