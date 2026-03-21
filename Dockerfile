FROM ghcr.io/arpanghosh8453/open-dronelog:latest

# Add the python packages to requirements.txt
COPY requirements.txt /app/requirements.txt
# install the requirements into the python venv
RUN python3 -m venv /opt/parser-venv && \
    /opt/parser-venv/bin/python -m pip install --no-cache-dir -U pip && \
    /opt/parser-venv/bin/python -m pip install --no-cache-dir -r /app/requirements.txt

# copy parser from here to the right location
COPY ulogParser /app/plugins/ulogParser
COPY parsers.json /app/plugins/parsers.json

# Thats it. We inherit the rest from base-image