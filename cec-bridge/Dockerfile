#
# We intentionally don't use python or uv images here, because libcec's python
# binding is not available through PyPi and needs to be installed as a system
# package.
#

FROM alpine

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=never

ENV PATH="/app/.venv/bin:$PATH"

# Install system dependencies for libcec
RUN apk add --no-cache uv py3-libcec libcec-tools

# Synchronize DEPENDENCIES without the application itself.
WORKDIR /app
COPY pyproject.toml uv.lock /app/
RUN uv venv --python /usr/bin/python3 --system-site-packages /app/.venv
RUN uv sync --locked --no-dev --no-install-project

# Now install the application itself, w/o dependencies.
COPY *.py /app/

STOPSIGNAL SIGINT
CMD ["python3", "/app/mqtt_bridge.py"]
#CMD ["tail", "-f", "/dev/null"]

