#!/bin/bash
# start_ollama.sh
#
# Starts the Ollama server in the background and polls until it's actually
# accepting connections, instead of guessing with a fixed `sleep N` (which
# is exactly what caused the "Connection refused" error -- running
# llm_interpreter.py before the server had finished binding its port).
#
# IMPORTANT: run this with `source`, not `./start_ollama.sh` or
# `bash start_ollama.sh` -- sourcing keeps the backgrounded `ollama serve`
# process attached to your current shell session so it doesn't get
# reparented/killed when the script itself finishes. Usage:
#
#   source /root/project/start_ollama.sh

echo "== Starting Ollama server =="
ollama serve > /root/project/ollama_server.log 2>&1 &

echo "== Waiting for Ollama to accept connections =="
for i in $(seq 1 30); do
    if curl -s -o /dev/null http://localhost:11434/; then
        echo "== Ollama is up and responding =="
        return 0 2>/dev/null || exit 0
    fi
    sleep 1
done

echo "== Ollama did not become ready within 30s -- check ollama_server.log =="
return 1 2>/dev/null || exit 1
