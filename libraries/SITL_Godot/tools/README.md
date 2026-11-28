# Tools for the Godot backend

These are not part of the Godot project; `.gdignore` keeps the editor out
of this directory.

## Model generators

The vehicles in `models` were written by these scripts, which set out
every part's position and parameters in one place. Each takes no
arguments and writes its model over the one in `models`; what they share
is in `gltf_model.py`:

```sh
./make_plane.py
./make_plane3d.py
./make_quadx.py
```

Editing a model in the Godot editor and exporting it works as well; a
model changed that way no longer matches its script.

The generators write their models through `json_pretify.py`, which keeps
anything that fits in 200 columns on one line. It is a filter, so
anything else which writes a model can be piped through it:

```sh
./json_pretify.py < exported.gltf > ../models/mine.gltf
```

## Flight logs

`tlog_landing.py` prints each landing in a telemetry log from its flare
until the vehicle disarms. The autotest suite leaves a `.tlog` for each
test at the top of the tree, which this reads after a landing fails. It
needs pymavlink.

```sh
./tlog_landing.py ../../../autotest-1234.tlog
```

## Tutorial video

`tutorial/godot_vehicle_editor_tutorial.gif` is a captioned walk through
editing a vehicle in the Godot editor, the steps listed under "Editing
vehicles" in the project's README. It is an animated GIF at one frame a
second, so it plays anywhere, and small enough to keep in the tree, which
costs it its legibility.
