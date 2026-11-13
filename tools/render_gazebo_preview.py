"""Create a GIF from captured Gazebo overview/RGB PNGs, not a synthetic path."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def route_preview(run, output, rows):
    """Only saved native sensor pixels; never reconstruct movement from poses."""
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 14)
    selected = rows[::8]
    if selected[-1] != rows[-1]:selected.append(rows[-1])
    frames=[]
    for row in selected:
        canvas=Image.new('RGB',(1064,640),'#111827');draw=ImageDraw.Draw(canvas)
        draw.text((10,8),'GAZEBO / DART | actual native cameras | 4x replay | landmark-assisted baseline',font=small,fill='white')
        draw.text((10,35),'Observer camera (ceiling tags hidden ONLY in observer)',font=small,fill='#cbd5e1')
        with Image.open(run/f"public/overview/{row['tick']:04d}.png") as im:
            canvas.paste(im.convert('RGB'),(8,62))
        for view,y in [('up',62),('down',350)]:
            draw.text((790,y-24),view+' camera / driver input',font=small,fill='#cbd5e1')
            with Image.open(run/'public'/row['input']['auxiliary'][view]['rgb_path']) as im:
                canvas.paste(im.convert('RGB'),(790,y))
        d=row['decision']
        draw.text((10,535),f"t={row['sim_s']:.1f}s | {d['state']} / {d['phase']}",font=font,fill='#86efac')
        draw.text((10,567),'Visually confirmed: '+' -> '.join(d['visited_nodes']),font=font,fill='white')
        draw.text((10,600),d['reason'],font=small,fill='#cbd5e1')
        frames.append(canvas)
    durations=[max(20,round((b['sim_s']-a['sim_s'])*250)) for a,b in zip(selected,selected[1:])]+[2000]
    frames[0].save(output,save_all=True,append_images=frames[1:],duration=durations,loop=0,optimize=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in (args.run / "decisions.jsonl").read_text().splitlines()]
    if rows and 'auxiliary' in rows[0]['input']:
        route_preview(args.run,args.output,rows)
        print(args.output.resolve())
        return
    selected = rows[::4]
    if selected[-1] != rows[-1]:
        selected.append(rows[-1])
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    frames = []
    for row in selected:
        canvas = Image.new("RGB", (784, 488), "#111827")
        draw = ImageDraw.Draw(canvas)
        draw.text((10, 8), "GAZEBO HARMONIC | DART physics | Ogre2 RGB | 4x replay", font=font, fill="white")
        draw.text((10, 37), "Native overhead (display only)", font=small, fill="#cbd5e1")
        draw.text((402, 37), "Controller RGB: front / left / right / back", font=small, fill="#cbd5e1")
        overview = args.run / f"public/overview/{row['tick']:04d}.png"
        with Image.open(overview) as image:
            canvas.paste(image.convert("RGB").resize((384,384)), (8,60))
        with Image.open(args.run / "public" / row["input"]["rgb_path"]) as image:
            canvas.paste(image.convert("RGB").resize((384,384), Image.Resampling.NEAREST), (400,60))
        d = row["decision"]
        draw.text((10, 452), f"t={row['sim_s']:.2f}s   {d['state']} / {d['phase']}", font=font, fill="#86efac")
        frames.append(canvas)
    durations = [max(20, round((b["sim_s"]-a["sim_s"])*1000/4)) for a,b in zip(selected,selected[1:])] + [1800]
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, optimize=False)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
