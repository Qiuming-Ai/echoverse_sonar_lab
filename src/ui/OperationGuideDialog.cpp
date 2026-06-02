#include "ui/OperationGuideDialog.hpp"

#include <QTextBrowser>
#include <QVBoxLayout>

namespace standalone_mvp {

OperationGuideDialog::OperationGuideDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(QStringLiteral("EchoVerse Sonar Lab - Operation Guide"));
    setWindowFlag(Qt::Window, true);
    resize(920, 700);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);

    auto* guide_view = new QTextBrowser(this);
    guide_view->setOpenExternalLinks(false);
    guide_view->setStyleSheet(
        "QTextBrowser{background:#0f1620;color:#e6f2ff;border:1px solid #5b7da1;padding:8px;"
        "selection-background-color:#2b6ea8;selection-color:#ffffff;}"
        "QScrollBar:vertical{background:#111a24;width:12px;margin:0px;}"
        "QScrollBar::handle:vertical{background:#3d5f83;min-height:20px;border-radius:5px;}"
        "QScrollBar::handle:vertical:hover{background:#4f78a3;}"
        "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0px;}");

    guide_view->setHtml(QStringLiteral(R"HTML(
<html>
<head>
<style>
body { font-family: "Segoe UI", sans-serif; line-height: 1.6; color: #dfefff; background: #0f1620; }
h1 { font-size: 24px; color: #9ad4ff; margin-bottom: 8px; }
h2 { font-size: 18px; color: #7ac8ff; border-bottom: 1px solid #2f4f6e; padding-bottom: 4px; margin-top: 22px; }
h3 { font-size: 15px; color: #b4dcff; margin-top: 14px; margin-bottom: 4px; }
p { margin: 8px 0; }
ul, ol { margin: 6px 0 10px 22px; }
li { margin: 4px 0; }
.card { border: 1px solid #2f4f6e; border-radius: 8px; padding: 10px 12px; margin: 10px 0; background: #101c28; }
.tip { color: #a8e1ff; }
code { color: #f4fbff; background: #1e2f42; padding: 1px 6px; border-radius: 4px; }
</style>
</head>
<body>
  <h1>EchoVerse Sonar Lab Operation Guide</h1>
  <p>This page summarizes key dashboard workflows: main camera control, mini-map usage, TCP stream, file stream, Path Mode, and split sonar windows.</p>

  <h2>1) Main Camera Control (Main Viewer)</h2>
  <div class="card">
    <h3>Keyboard Controls</h3>
    <ul>
      <li><code>W/S</code> move forward/backward, <code>A/D</code> strafe left/right, <code>I/K</code> move up/down.</li>
      <li><code>Q/E</code> yaw left/right, <code>U/J</code> pitch up/down.</li>
      <li>Main camera pose directly drives sonar observation pose according to module binding.</li>
    </ul>
    <p class="tip">Tip: stabilize your camera view before recording output, so post-processing alignment is easier.</p>
  </div>

  <h2>2) Mini-Map Operation (Path / Live Map)</h2>
  <div class="card">
    <ul>
      <li>The left mini-map supports waypoint editing and path preview.</li>
      <li>Path edits update the current Path configuration used by Path Mode startup.</li>
      <li>When scene models are updated, the mini-map refreshes scene references accordingly.</li>
    </ul>
    <p class="tip">Tip: keep at least two waypoints and verify smooth altitude/turn transitions.</p>
  </div>

  <h2>3) TCP Stream Operation</h2>
  <div class="card">
    <ul>
      <li>Enable TCP output per module in <code>Settings</code> and configure ports.</li>
      <li>After session start, the app listens on configured ports and streams sonar packets continuously.</li>
      <li>Clients can dispatch by packet magic: <code>NS2P</code> (ESL2D) / <code>NS3P</code> (ESL3D).</li>
      <li>The top-bar <code>TCP</code> indicator lights up when TCP output is active in the current session.</li>
    </ul>
  </div>

  <h2>4) File Stream Operation</h2>
  <div class="card">
    <ul>
      <li>Enable file output in <code>Settings</code>; session output is stored under <code>Sonar Data/&lt;timestamp&gt;</code> in the project directory.</li>
      <li>Per-module files are generated as <code>2d.esl2d</code> and/or <code>3d.esl3d</code>.</li>
      <li>On stop, the app writes <code>recording_summary.json</code> (duration, frame counts, config snapshots).</li>
      <li>For FLS/MBES point-cloud file output, offline post-process may generate artifacts in <code>Waveform Data</code>.</li>
      <li>The top-bar <code>FILE</code> indicator lights up when file output is active in the current session.</li>
    </ul>
  </div>

  <h2>5) Path Mode Operation</h2>
  <div class="card">
    <ol>
      <li>Click top-bar <code>Path Mode</code> to enter path workflow.</li>
      <li>Click <code>Start</code> to run automatic path following using the configured waypoints.</li>
      <li>Click <code>Stop</code> to halt path following and return to normal manual control.</li>
    </ol>
    <p>Path Mode is useful for repeatable data capture and trajectory replay experiments.</p>
  </div>

  <h2>6) Split Window Operation (Sonar Tabs)</h2>
  <div class="card">
    <ul>
      <li>Use top-bar <code>Show Sonar</code> to show/hide the sonar panel area.</li>
      <li>Multi-tab mode supports split presets (single/horizontal/vertical/one-by-three/quad).</li>
      <li>Layout and tab placement are persisted in project configuration and restored on next launch.</li>
    </ul>
    <p class="tip">Tip: keep critical modules pinned in visible panes to avoid interruption during recording.</p>
  </div>

  <h2>7) Scene Editor Operation</h2>
  <div class="card">
    <ul>
      <li>Click top-bar <code>Scene Editor</code> to open the scene management dialog.</li>
      <li>Use <code>Add Model...</code> to import a model, then set pose/scale as needed.</li>
      <li>Use <code>Edit Pose...</code> to update position and orientation of selected models.</li>
      <li>Use <code>Delete</code> to remove selected entries from the active scene list.</li>
      <li>Use <code>Import Models</code> to refresh imported assets and scene resources.</li>
      <li>Scene updates are reflected in runtime views and mini-map context after refresh.</li>
    </ul>
    <p class="tip">Tip: pause intensive capture/export before large scene edits to keep sonar output stable.</p>
  </div>

  <h2>Recommended Workflow</h2>
  <ol>
    <li>Configure module outputs (TCP/file) and ports first.</li>
    <li>Adjust main camera, mini-map path, and scene content (if needed).</li>
    <li>Optionally start Path Mode and monitor FILE/TCP indicators plus recording timer.</li>
    <li>After stop, validate <code>recording_summary.json</code> and output file completeness.</li>
  </ol>
</body>
</html>
)HTML"));

    layout->addWidget(guide_view, 1);
}

} // namespace standalone_mvp
