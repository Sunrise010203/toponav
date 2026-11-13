import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from roadvision.contract import CLASS_IDS
from roadvision.scene import LAYOUTS, World, layout_fingerprint


def validate_manifest(rows):
    layouts, batches, paths = {}, {}, set()
    for row in rows:
        for table, key in ((layouts, "layout_id"), (batches, "batch_id")):
            identity = row[key]
            if identity in table and table[identity] != row["split"]:
                raise ValueError(f"Split leakage: {identity}")
            table[identity] = row["split"]
        if row["path"] in paths or not 0 <= row["class_index"] < 17:
            raise ValueError("Duplicate sample or invalid label")
        paths.add(row["path"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    counts = {"train": 40, "val": 16, "test": 24}
    fingerprints = {name: layout_fingerprint(layout) for name, layout in LAYOUTS.items()}
    if len(set(fingerprints.values())) != len(fingerprints):
        raise ValueError("Repeated layout topology")
    for layout_number, (name, layout) in enumerate(LAYOUTS.items()):
        if args.preview and layout_number:
            break
        with World(name) as world:
            panels = []
            for label in range(17):
                amount = 1 if args.preview else counts[layout["split"]]
                for sample in range(amount):
                    batch = sample % 4
                    seed = 4000+layout_number*100000+label*1000+sample
                    image, audit = world.sample(label, np.random.default_rng(seed))
                    filename = f"{name}_{label:02}_{sample:04}.png"
                    Image.fromarray(image).save(args.output/filename)
                    rows.append({"path": filename, "layout_id": name, "batch_id": f"{name}_b{batch}",
                                 "split": layout["split"], "class_index": label, "frame_id": filename[:-4],
                                 "captured_at_ns": time.time_ns(), "seed": seed, "audit": audit,
                                 "image_sha256": hashlib.sha256((args.output/filename).read_bytes()).hexdigest()})
                    if args.preview:
                        panels.append(Image.fromarray(image))
            print(f"Captured {name}: {amount*17} physical four-view images", flush=True)
            if args.preview:
                sheet = Image.new("RGB", (1024, 5*280), "white")
                draw = ImageDraw.Draw(sheet)
                for index, panel in enumerate(panels):
                    x, y = (index % 4)*256, (index//4)*280
                    sheet.paste(panel, (x, y))
                    draw.text((x+4, y+258), CLASS_IDS[index], fill="black")
                sheet.save(args.output/"preview_contact_sheet.png")
    validate_manifest(rows)
    manifest = {"renderer": "PyBullet TinyRenderer 3.2.7", "class_ids": CLASS_IDS,
                "layout_fingerprints": fingerprints, "counts_per_class_per_layout": counts,
                "view_order": ["front", "left", "right", "back"],
                "label_policy": "Stable orthogonal junctions: front/left/right/back exit bits; approaching, passing, turning, occluded or missing view: unobservable",
                "preview_only": args.preview, "rows": rows}
    data = (json.dumps(manifest, indent=2)+"\n").encode()
    (args.output/"manifest.json").write_bytes(data)
    (args.output/"manifest.sha256").write_text(hashlib.sha256(data).hexdigest()+"\n")
    print(f"Saved {len(rows)} samples; preview={args.preview}")


if __name__ == "__main__":
    main()
