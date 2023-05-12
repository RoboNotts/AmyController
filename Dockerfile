FROM pypy:3.9-7.3.11-bullseye

RUN export DEBIAN_FRONTEND=noninteractive \
  && apt update \
  && apt install -y \
  && pip install pipenv \
  && rm -rf /var/lib/apt/lists/*

COPY Pipfile /
WORKDIR /
RUN sed -i -e "s/python_version = \".*\"/python_version = \"3.9\"/" \
  -e "s/python_full_version = \".*\"/python_full_version = \"3.9.16\"/" \
  Pipfile \
  && pipenv lock \
  && pipenv requirements > requirements.txt \
  && pip install -r requirements.txt

COPY src /app

ENTRYPOINT [ "uvicorn", "--app-dir", "/app", "webui:app" ]
