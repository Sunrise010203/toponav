"""Native scene chooser + topology viewer. Gazebo remains the actual simulator."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import uuid

from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout,
    QHBoxLayout, QLabel, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTabWidget, QSizePolicy)

from simulation.gazebo_scenes import SCENARIOS, ROOT, scenario_pair, topology_image
from simulation.dijkstra import plan


class ScenarioWindow(QMainWindow):
    def __init__(self, output_root, auto_open=True):
        super().__init__()
        self.setWindowTitle("TopoNav · Gazebo 场景与拓扑")
        self.resize(820,min(980,QApplication.primaryScreen().availableGeometry().height()-50))
        screen = QApplication.primaryScreen().availableGeometry()
        self.move(max(0, screen.width()-840),40)
        self.output_root = Path(output_root).resolve()
        self.output_root.mkdir(parents=True,exist_ok=True)
        self.folder = None
        self.worker = QProcess(self)
        self.worker.finished.connect(self.finished)
        self.worker.errorOccurred.connect(self.failed)
        self.closing = False
        self.restart_requested = False
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        title = QLabel("Gazebo 场景中心")
        title.setStyleSheet("font-size:22px;font-weight:600")
        layout.addWidget(title)
        self.scenario = QComboBox()
        for key, title in SCENARIOS.items():
            self.scenario.addItem(title,key)
        self.scenario.currentIndexChanged.connect(self.refresh)
        layout.addWidget(self.scenario)
        self.note = QLabel()
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.goal = QComboBox()
        graph,_ = scenario_pair("topology")
        for node in graph["nodes"]:
            if node["node_id"] != graph["task"]["start_node"]:
                self.goal.addItem(node["node_id"])
        self.goal.setCurrentText(graph["task"]["goal_node"])
        self.goal.currentTextChanged.connect(self.refresh)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("规划目标"))
        controls.addWidget(self.goal)
        self.closed = []
        controls.addWidget(QLabel("封闭通道"))
        for edge in graph["edges"]:
            box = QCheckBox(edge["channel_id"])
            box.toggled.connect(self.refresh)
            self.closed.append(box)
            controls.addWidget(box)
        layout.addLayout(controls)
        self.map = QLabel()
        self.map.setAlignment(Qt.AlignCenter)
        self.map.setMinimumHeight(260)
        self.view_tabs = QTabWidget()
        self.view_tabs.addTab(self.map, '拓扑图')
        fpv_page = QWidget()
        fpv_layout = QVBoxLayout(fpv_page)
        self.fpv_selector = QComboBox()
        for key, caption in [('front','前方 · 小车第一视角'),('left','左侧'),('right','右侧'),('back','后方')]:
            self.fpv_selector.addItem(caption,key)
        self.fpv_selector.currentIndexChanged.connect(self.refresh_fpv)
        fpv_layout.addWidget(self.fpv_selector)
        self.fpv = QLabel('等待小车相机')
        self.fpv.setAlignment(Qt.AlignCenter)
        self.fpv.setMinimumHeight(260)
        self.fpv.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Expanding)
        self.fpv.setStyleSheet('background:#111827;color:#cbd5e1')
        fpv_layout.addWidget(self.fpv,1)
        self.camera_stamp = QLabel('原生水平相机 · 仅展示，不是标记辅助导航的驾驶输入')
        self.camera_stamp.setWordWrap(True)
        fpv_layout.addWidget(self.camera_stamp)
        self.view_tabs.addTab(fpv_page, '小车视角')
        quad_page = QWidget()
        quad_layout = QGridLayout(quad_page)
        self.horizontal_views = {}
        for i,(view,caption) in enumerate([('front','前方'),('left','左侧'),('right','右侧'),('back','后方')]):
            group = QVBoxLayout()
            group.addWidget(QLabel(caption+' · 真实相机'))
            label = QLabel('等待相机')
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(1,1)
            label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Expanding)
            group.addWidget(label,1)
            quad_layout.addLayout(group,i//2,i%2)
            self.horizontal_views[view] = label
        self.view_tabs.addTab(quad_page,'四向相机')
        self.view_tabs.setCurrentIndex(1)
        self.horizontal_pixels = {}
        layout.addWidget(self.view_tabs,1)
        cameras = QHBoxLayout()
        self.camera_views = {}
        for view, caption in [('up','向上真实相机 · 识别节点/朝向'),('down','向下真实相机 · 黄线跟随')]:
            column = QVBoxLayout()
            column.addWidget(QLabel(caption))
            label = QLabel('等待相机')
            label.setAlignment(Qt.AlignCenter)
            label.setFixedHeight(130)
            self.camera_views[view] = label
            column.addWidget(label)
            cameras.addLayout(column)
        self.auxiliary_group = QWidget()
        self.auxiliary_group.setLayout(cameras)
        layout.addWidget(self.auxiliary_group)
        self.model_status = QLabel()
        self.model_status.setWordWrap(True)
        layout.addWidget(self.model_status)
        self.route = QLabel()
        self.route.setWordWrap(True)
        layout.addWidget(self.route)
        self.confirmed = QCheckBox("已确认仿真初始前方至少 0.30m 净空（仅局部控制测试需要）")
        layout.addWidget(self.confirmed)
        self.probe = QCheckBox('单独授权直走受限探测（默认不授权；确认净空不等于运动许可）')
        layout.addWidget(self.probe)
        self.scenario.currentIndexChanged.connect(lambda *_:self.probe.setChecked(False))
        buttons = QHBoxLayout()
        self.start = QPushButton("打开 Gazebo 场景")
        self.start.clicked.connect(self.launch)
        self.navigate = QPushButton('开始导航')
        self.navigate.clicked.connect(self.begin_navigation)
        self.navigate.setEnabled(False)
        self.stop = QPushButton("停止 / 切换场景")
        self.stop.clicked.connect(self.stop_worker)
        self.stop.setEnabled(False)
        buttons.addWidget(self.start)
        buttons.addWidget(self.navigate)
        buttons.addWidget(self.stop)
        layout.addLayout(buttons)
        self.status = QLabel("就绪")
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.status)
        self.refresh()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)
        if auto_open:
            QTimer.singleShot(500,self.launch)

    def selection(self):
        name = self.scenario.currentData()
        disabled = [box.text() for box in self.closed if box.isChecked()] if name in ("topology", "route", "vision") else []
        goal = self.goal.currentText() if name in ("topology", "route", "vision") else None
        return name,disabled,goal

    def refresh(self):
        # Signals can be emitted while constructor controls are still forming.
        if not hasattr(self,"route"):
            return
        name,disabled,goal = self.selection()
        topology = name in ("topology", "route", "vision")
        self.auxiliary_group.setVisible(name=='route')
        self.model_status.setVisible(name=='vision')
        self.model_status.setText('原17类模型：等待本次四向RGB推理')
        idle = self.worker.state() == QProcess.NotRunning
        self.goal.setEnabled(topology and idle)
        for box in self.closed:
            box.setEnabled(topology and idle)
        self.confirmed.setEnabled(not topology and idle)
        self.probe.setVisible(name in ('straight','blocked'))
        self.probe.setEnabled(name in ('straight','blocked') and idle)
        if name not in ('straight','blocked'):self.probe.setChecked(False)
        self.note.setText("新增标记辅助全流程：原六节点/0.8m 道路 + 真实天花路标/地面黄线 + 上下相机。相机识别节点→拓扑选路→跟线/原地转弯→目标停车；不是原无标记模型验收。窗口就绪后点击“开始导航”。" if name == 'route' else "六节点场景来自项目 graph.json / scene.json。当前仅展示道路、任务规划与封路重规划；车保持零速，不冒充全程自主导航。"
            if topology else "原控制器的局部 RGB 实验，不代表完成全图导航。T 型新增场景需看实测结果；直走/阻挡可能因证据不足拒绝或中止。")
        if name=='vision':
            self.note.setText('原冻结17类模型 → 原候选定位/规划 → RGB几何行驶/转弯 → 新模型观测确认到站。无ArUco、无黄线、无上下相机、无位姿驾驶；使用显式相机/蓝灰地面标定的受限实验场景，不等于通用视觉或S2/S6验收。')
        graph,scene = scenario_pair(name,disabled,goal)
        pixels = topology_image(graph,scene,label="预览：尚未启动；无车辆定位输出")
        self.set_pixels(pixels)
        route = plan(graph)
        path = " → ".join(route["nodes"]) if route["status"] == "reachable" else "不可达"
        self.route.setText("计划路径（执行状态见视觉确认）："+path)

    def set_pixels(self, pixels):
        height,width = pixels.shape[:2]
        image = QImage(pixels.data,width,height,width*3,QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image)
        self.map.setPixmap(pixmap.scaled(760,500,Qt.KeepAspectRatio,Qt.SmoothTransformation))

    def launch(self):
        if self.worker.state() != QProcess.NotRunning:
            if self.start.isEnabled():
                self.restart_requested = True
                self.stop_worker()
            return
        name,disabled,goal = self.selection()
        if name not in ("topology", "route", "vision") and not self.confirmed.isChecked():
            self.status.setText("请先确认仿真初始净空；未确认时不会启动局部驾驶。")
            return
        self.folder = self.output_root / (name+"-"+uuid.uuid4().hex[:12])
        self.horizontal_pixels.clear()
        self.fpv.clear()
        self.fpv.setText('等待本次小车相机')
        self.camera_stamp.setText('等待本次原始采集时间 · 仅展示')
        for label in (*self.horizontal_views.values(), *self.camera_views.values()):
            label.clear()
            label.setText('等待本次相机')
        command = [str(ROOT/"tools/run_gazebo_cross_validation.sh"),"--gui","--scenario",name,
                   "--output",str(self.folder)]
        if name in ('route','vision'):
            command += ['--wait-for-start']
        if self.confirmed.isChecked() and name != "topology":
            command.append("--confirm-initial-clearance")
        if name in ('straight','blocked'):
            command += ['--motion-permission','bounded_probe' if self.probe.isChecked() else 'no_motion']
        if goal:
            command += ["--goal",goal]
        for channel in disabled:
            command += ["--disable",channel]
        self.worker.setWorkingDirectory(str(ROOT))
        self.worker.setProcessChannelMode(QProcess.MergedChannels)
        self.worker.setStandardOutputFile(str(self.output_root/(self.folder.name+".log")))
        self.worker.start("bash",command)
        self.start.setEnabled(False)
        self.start.setText('打开 Gazebo 场景')
        self.stop.setEnabled(True)
        self.scenario.setEnabled(False)
        self.goal.setEnabled(False)
        self.confirmed.setEnabled(False)
        self.probe.setEnabled(False)
        for box in self.closed:
            box.setEnabled(False)
        self.status.setText("正在启动原生 Gazebo 窗口…\n结果："+str(self.folder))

    def begin_navigation(self):
        if self.folder and (self.folder/'ready.json').exists() and self.worker.state() != QProcess.NotRunning:
            (self.folder/'start.requested').touch()
            self.navigate.setEnabled(False)
            self.status.setText('已开始相机闭环导航；可随时点击停止。')

    def refresh_fpv(self, *_):
        if not hasattr(self, 'horizontal_pixels'):
            return
        pixels = self.horizontal_pixels.get(self.fpv_selector.currentData())
        if pixels is not None:
            self.fpv.setPixmap(pixels.scaled(max(260,self.fpv.width()-8),max(260,self.fpv.height()-8),
                                             Qt.KeepAspectRatio,Qt.FastTransformation))

    def public_image(self, relative):
        """Read a complete public PNG, never audit/pose or a half-written file."""
        root = (self.folder/'public').resolve()
        path = (root/relative).resolve()
        if not path.is_relative_to(root):
            return None
        try:
            data = path.read_bytes()
        except OSError:
            return None
        if not data.endswith(b'\x00\x00\x00\x00IEND\xaeB\x60\x82'):
            return None
        pixels = QPixmap()
        return pixels if pixels.loadFromData(data,'PNG') else None

    def show_camera_input(self, inp):
        pixels = self.public_image(inp['rgb_path'])
        if pixels is not None and pixels.width()==pixels.height()==256:
            for view,x,y in [('front',0,0),('left',128,0),('right',0,128),('back',128,128)]:
                tile = pixels.copy(x,y,128,128)
                self.horizontal_pixels[view] = tile
                self.horizontal_views[view].setPixmap(tile.scaled(190,190,Qt.KeepAspectRatio,Qt.FastTransformation))
            self.refresh_fpv()
            role = '本模式四向RGB是原17类模型的真实输入' if self.scenario.currentData()=='vision' else '仅展示；标记辅助导航仍使用上下相机'
            self.camera_stamp.setText(f"原生水平相机 · 采集 t={inp['captured_sim_s']:.3f}s · 原始128×128，仅放大显示\n"+role)
        for view,item in inp.get('auxiliary',{}).items():
            if view in self.camera_views:
                pixels = self.public_image(item['rgb_path'])
                if pixels is not None:
                    self.camera_views[view].setPixmap(pixels.scaled(130,130,Qt.KeepAspectRatio,Qt.SmoothTransformation))

    def stop_worker(self):
        if self.worker.state() != QProcess.NotRunning and self.worker.processId():
            os.kill(self.worker.processId(),signal.SIGINT)
            self.stop.setEnabled(False)
            self.navigate.setEnabled(False)
            self.status.setText("正在零速停车、保存记录并清理本窗口启动的服务…")

    def poll(self):
        if self.folder is None or self.worker.state() == QProcess.NotRunning:
            return
        if (self.folder/'ready.json').exists() and not (self.folder/'start.requested').exists() and self.stop.isEnabled():
            self.navigate.setEnabled(True)
            self.status.setText('Gazebo 已就绪：点击“开始导航”观看完整运行过程。')
        filename = self.folder / "public/topology.png"
        if filename.exists():
            # The observer may be updating this file. Never ask Qt/libpng to
            # decode an unfinished PNG; simply retain the last complete map.
            try:
                data = filename.read_bytes()
            except OSError:
                data = b''
            if data.endswith(b'\x00\x00\x00\x00IEND\xaeB\x60\x82'):
                pixmap = QPixmap()
                if pixmap.loadFromData(data, 'PNG'):
                    self.map.setPixmap(pixmap.scaled(760,500,Qt.KeepAspectRatio,Qt.SmoothTransformation))
        path = self.folder / "decisions.jsonl"
        if not path.exists():
            preview = self.folder/'public/camera_preview.json'
            if preview.exists():
                try:
                    self.show_camera_input(json.loads(preview.read_text()))
                except (ValueError,KeyError,TypeError):
                    pass
        if self.worker.state() != QProcess.NotRunning and self.stop.isEnabled() and path.exists():
            try:
                lines = path.read_text().splitlines()
                row = json.loads(lines[-1])
                d = row["decision"]
                self.show_camera_input(row['input'])
                if d.get('model'):
                    m=d['model'];loc=d.get('localization') or {};planner=d.get('planner') or {}
                    self.model_status.setText(f"原模型：{m['class_id']} · 得分 {m['score']:.3f} · {'接纳' if m['accepted'] else '拒识'}\n上次节点定位：{loc.get('status','待确认')} · 规划动作：{planner.get('action','stop')} · 当前阶段：{d['phase']}")
                visited = ' → '.join(d.get('visited_nodes',[]))
                self.status.setText(f"模拟 {row['sim_s']:.1f}s · {d['state']} · {d['reason']}\n视觉已确认：{visited or '暂无'}\n"+str(self.folder))
                if d['state'] in ('visual_stop','aborted'):
                    self.start.setText('重置 / 再运行')
                    self.start.setEnabled(True)
            except (ValueError,IndexError):
                pass  # Writer may be halfway through its latest JSON line.

    def finished(self,code,*_):
        self.navigate.setEnabled(False)
        self.start.setEnabled(True)
        self.stop.setEnabled(False)
        self.scenario.setEnabled(True)
        message = f"本次窗口已关闭，退出码 {code}。可以选择其他场景。"
        if self.folder and (self.folder/"result.json").exists():
            result = json.loads((self.folder/"result.json").read_text())
            message += "\n"+result["controller_result"]["result_type"]
        self.status.setText(message)
        self.refresh()
        if self.closing:
            self.close()
        elif self.restart_requested:
            self.restart_requested = False
            QTimer.singleShot(0,self.launch)

    def failed(self,*_):
        self.status.setText("启动失败："+self.worker.errorString())
        if self.worker.state() == QProcess.NotRunning:
            self.start.setEnabled(True)
            self.scenario.setEnabled(True)
            self.stop.setEnabled(False)

    def closeEvent(self,event):
        if self.worker.state() != QProcess.NotRunning:
            self.closing = True
            self.stop_worker()
            event.ignore()
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=ROOT/"reports/gazebo-scenarios")
    parser.add_argument("--no-auto-open", action="store_true")
    parser.add_argument('--scenario',choices=SCENARIOS,default='route')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setApplicationName("TopoNav Gazebo 场景与拓扑")
    window = ScenarioWindow(args.output_root,not args.no_auto_open)
    window.scenario.setCurrentIndex(window.scenario.findData(args.scenario))
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
