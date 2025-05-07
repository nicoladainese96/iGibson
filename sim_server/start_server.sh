#! /usr/bin/env bash

# default port
PORT=8000

while [[ $# -gt 0 ]]; do
  case $1 in
    --port)
      PORT="$2"
      shift 2
      ;;
    *)
      echo "Unknown parameter: $1"
      exit 1
      ;;
  esac
done

# activate & start
source myenv/bin/activate
cd iGibson/sim_server
uvicorn igibson_server:app --host 0.0.0.0 --port "${PORT}" --reload
