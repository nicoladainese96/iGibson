## Assume the file Structure is as follows:

```
root/
├── behavior/
├── iGibson/
│   └── sim_server/
│       ├── igibson_server.py
│       └── start_server.sh
├── Makefile
├── igibson_latest.sif
├── myenv/
├── ...
└── README.md
```

THe content of Makefile is as follows:

```
.PHONY: shell run
shell:
	srun --gpus=1 --mem=40GB --pty apptainer exec --nv ./igibson_latest.sif bash	

run:
	srun --gpus=1 --mem=40GB --pty apptainer exec --nv ./igibson_latest.sif bash ./iGibson/sim_server/start_server.sh
```

Note: by default, the requested time is half an hour. If you need more time, add for example `--time=02:00:00` to the `srun` command.

## Start the server
From the root directory, run the following commands:
1. `make shell` to enter the shell of the container to do some interactive work, e.g. activate the python environment and install some packages: uvicorn, fastapi, etc.
2. `make run` to start the server

## Test the server
1. run `slurm q` to get the gpu node name
2. Modify the `base_url` in the `http_request_examples.py` file to the gpu node name
3. Run `python3 http_request_examples.py` to test the server
