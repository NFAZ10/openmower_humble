FROM python:3.11-slim
RUN pip install flask
WORKDIR /app
COPY server.py .
COPY templates/ templates/
CMD ["python", "server.py"]
