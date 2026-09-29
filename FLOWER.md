# Flower coordinator

`run_flower.py` starts a local Flower simulation with two workers. Each worker observes the same simulated grid world from its assigned position and proposes a one cell move toward a distinct frontier. The server merges observations and applies the movement safety gate. It prints `FLOWER_SUMMARY` on success.

```sh
.venv/bin/python run_flower.py --steps 3
```

To include one saved JPEG in the first worker observation, create `.env` in the project root with `OPENAI_API_KEY=...`, capture a frame into `runs/`, then run:

```sh
.venv/bin/python run_flower.py --steps 1 --vlm
```

This makes one OpenAI image request. The response is logged as `FLOWER_VLM`. It is a description and action proposal only; it does not alter the simulated map or control motors. The second worker remains simulated. The model and image request are in `flower_explore/openai_observer.py`.

A saved frame avoids contention with a live camera preview. OpenAI access and a saved JPEG are required only for `--vlm`.
