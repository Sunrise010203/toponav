"""Small exclusive-output, hashing, resource and deterministic-seed helpers."""
import hashlib
import json
import os
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'configs/experiments/block7'


def canonical(data):
    return json.dumps(data, sort_keys=True, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).encode()


def digest(data):
    return hashlib.sha256(canonical(data)).hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def seed(*parts):
    return int.from_bytes(hashlib.sha256(canonical(parts)).digest()[:8], 'big')


def read_json(path):
    return json.loads(Path(path).read_text())


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def write_json(path, data):
    with Path(path).open('x') as file:
        json.dump(data, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write('\n')


def write_rows(path, rows):
    with Path(path).open('x') as file:
        for row in rows:
            file.write(canonical(row).decode() + '\n')


def atomic_json(path, data):
    """Only for the evolving ledger; never used to overwrite old evidence."""
    path = Path(path)
    temporary = path.with_name(path.name + '.new')
    with temporary.open('x') as file:
        file.write(canonical(data).decode() + '\n')
        file.flush()
        os.fsync(file.fileno())
    temporary.replace(path)


def check_resources(start=True, path=ROOT):
    import yaml
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())['resources']
    free = shutil.disk_usage(path).free
    memory = int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines()
                      if line.startswith('MemAvailable:'))) * 1024
    threshold = budget['min_start_free_disk_bytes'] if start else budget['stop_free_disk_bytes']
    if free < threshold or (start and memory < budget['min_start_available_memory_bytes']):
        raise RuntimeError('Insufficient declared disk/memory budget; no new episode may start')
    return {'free_disk_bytes': free, 'available_memory_bytes': memory}
