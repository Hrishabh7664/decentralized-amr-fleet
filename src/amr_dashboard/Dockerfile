FROM python:3.10-slim

WORKDIR /app
COPY public /app/public

EXPOSE 3000

CMD ["python3", "-m", "http.server", "3000", "--directory", "/app/public"]
