# Use a slim Python base
FROM python:3.8-slim

# Set working directory
WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY src/ ./src

# Default command: fast mode
CMD ["python3", "src/main.py", "--mode", "fast"]
