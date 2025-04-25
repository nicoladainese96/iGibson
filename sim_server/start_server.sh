#! /bin/bash
source myenv/bin/activate
cd iGibson/sim_server
uvicorn igibson_server:app --host 0.0.0.0 --port 8000 --reload
