#!/usr/bin/env bash
# Source this file; all paths are private to this project, not /usr or /opt/ros.
_toponav_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export TOPONAV_GAZEBO_PREFIX="$_toponav_root/.simulators/gazebo-harmonic"
_gz_usr="$TOPONAV_GAZEBO_PREFIX/usr"
export PATH="$_gz_usr/bin:$PATH"
export LD_LIBRARY_PATH="$_gz_usr/lib/x86_64-linux-gnu:$_gz_usr/lib/x86_64-linux-gnu/OGRE-2.3:$_gz_usr/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$_gz_usr/lib/python3/dist-packages:$_toponav_root:${PYTHONPATH:-}"
export GZ_CONFIG_PATH="$_gz_usr/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$_gz_usr/lib/x86_64-linux-gnu/gz-sim-8/plugins"
export GZ_SIM_RESOURCE_PATH="$_gz_usr/share/gz/gz-sim8:$_gz_usr/share"
export GZ_RENDERING_PLUGIN_PATH="$_gz_usr/lib/x86_64-linux-gnu/gz-rendering-8/engine-plugins"
export GZ_PHYSICS_ENGINE_PLUGIN_PATH="$_gz_usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins"
export GZ_SIM_PHYSICS_ENGINE_PATH="$GZ_PHYSICS_ENGINE_PLUGIN_PATH"
export GZ_RENDERING_RESOURCE_PATH="$_gz_usr/share/gz/gz-rendering8"
export OGRE2_RESOURCE_PATH="$_gz_usr/lib/x86_64-linux-gnu/OGRE-2.3/OGRE"
export GZ_GUI_PLUGIN_PATH="$_gz_usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui:$_gz_usr/lib/x86_64-linux-gnu/gz-gui-8/plugins"
export GZ_GUI_RESOURCE_PATH="$_gz_usr/share/gz/gz-gui8:$_gz_usr/share/gz/gz-sim8/gui"
export QML2_IMPORT_PATH="$_gz_usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
export RUBYLIB="$_gz_usr/lib/ruby/3.0.0:$_gz_usr/lib/x86_64-linux-gnu/ruby/3.0.0:${RUBYLIB:-}"
export LIBGL_ALWAYS_SOFTWARE=1
# Ubuntu's Gazebo .deb protos predate the venv protobuf runtime. Use its
# supported pure-Python parser in this process only; never downgrade the venv.
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
unset _gz_usr _toponav_root
