FROM python:3.11-bookworm

# Install system build dependencies not found on the user's mac
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install tinympc
RUN pip install numpy tinympc

# Set working directory
WORKDIR /app

# The generation script will be mounted or copied here
CMD ["python", "generate_tinympc.py"]
