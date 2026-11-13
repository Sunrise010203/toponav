# Native Gazebo RGB regression fixtures

Both PNGs are unmodified synchronized native four-view RGB (TL front, TR left,
BL right, BR back) from development-01 under reports/gazebo-model-20261020.
start.png = public/frames/0000.png; short_corridor.png = public/frames/0519.png.
The latter is the actual safe-stop failure input, not a synthesized successful
observation. It tests extracting the visible near floor/wall join rather than
the vertical junction silhouette. These are exposed development samples, not
an independent model accuracy test set. Rig calibration: h=.8/3m, radial
offset=.18m, FOV=90deg, original .8m road, .8m ceiling; no markers or paint.
