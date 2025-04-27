
FROM ghcr.io/astral-sh/uv:python3.13-alpine AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

# Synchronize DEPENDENCIES without the application itself.
WORKDIR /app
COPY pyproject.toml uv.lock /app/
RUN uv sync --locked --no-dev --no-install-project

# Now install the application itself, w/o dependencies.
COPY *.py /app/

FROM python:3.13-alpine
WORKDIR /app
COPY --from=builder /app /app

ENV PATH="/app/.venv/bin:$PATH"

STOPSIGNAL SIGINT
CMD ["/app/main.py"]
