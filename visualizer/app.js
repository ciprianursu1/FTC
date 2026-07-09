const FIELD_WIDTH = 144;
const FIELD_HEIGHT = 288;
const VIEW_WIDTH = 288;
const VIEW_HEIGHT = 144;
const STORAGE_KEY = "fri-pedro-visualizer-state-v2";

const canvas = document.getElementById("fieldCanvas");
const ctx = canvas.getContext("2d");
const fieldImage = new Image();
fieldImage.src = "assets/fri-field-reference.svg";
const robotImage = new Image();
robotImage.src = "assets/robot-footprint.png";

const state = {
  poses: [
    { x: 125, y: 22, heading: 55, name: "Start" },
    { x: 103.178, y: 72.897, heading: 0, name: "Scan" },
    { x: 85.234, y: 58.093, heading: 40, name: "Shoot" },
  ],
  pathModes: ["Linear", "Linear"],
  selected: 0,
  dragging: null,
  history: [],
  historyIndex: -1,
  robotLengthInches: 18,
  robotWidthInches: 18,
  gridSizeInches: 12,
  gridVisible: true,
  view: { left: 0, top: 0, scale: 1 },
};

const controls = {
  addPointButton: document.getElementById("addPointButton"),
  undoButton: document.getElementById("undoButton"),
  redoButton: document.getElementById("redoButton"),
  mirrorButton: document.getElementById("mirrorButton"),
  toggleGridButton: document.getElementById("toggleGridButton"),
  clearButton: document.getElementById("clearButton"),
  deletePointButton: document.getElementById("deletePointButton"),
  xInput: document.getElementById("xInput"),
  yInput: document.getElementById("yInput"),
  headingInput: document.getElementById("headingInput"),
  nameInput: document.getElementById("nameInput"),
  robotLengthInput: document.getElementById("robotLengthInput"),
  robotWidthInput: document.getElementById("robotWidthInput"),
  gridSizeInput: document.getElementById("gridSizeInput"),
  pointCount: document.getElementById("pointCount"),
  pathLength: document.getElementById("pathLength"),
  poseList: document.getElementById("poseList"),
  importText: document.getElementById("importText"),
  importButton: document.getElementById("importButton"),
  pathNameInput: document.getElementById("pathNameInput"),
  speedInput: document.getElementById("speedInput"),
  exportText: document.getElementById("exportText"),
  copyButton: document.getElementById("copyButton"),
};

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function round(value, digits = 3) {
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
}

function loadState() {
  const saved = localStorage.getItem(STORAGE_KEY);
  if (!saved) return;

  try {
    const parsed = JSON.parse(saved);
    if (Array.isArray(parsed.poses) && parsed.poses.length > 0) {
      state.poses = parsed.poses.map((pose, index) => ({
        x: clamp(Number(pose.x) || 0, 0, FIELD_WIDTH),
        y: clamp(Number(pose.y) || 0, 0, FIELD_HEIGHT),
        heading: Number(pose.heading) || 0,
        name: pose.name || `Pose ${index + 1}`,
      }));
      state.selected = clamp(Number(parsed.selected) || 0, 0, state.poses.length - 1);
    }
    if (Array.isArray(parsed.pathModes)) {
      state.pathModes = parsed.pathModes;
    }
    const legacySize = Number(parsed.robotSizeInches) || 18;
    state.robotLengthInches = clamp(Number(parsed.robotLengthInches) || legacySize, 1, 36);
    state.robotWidthInches = clamp(Number(parsed.robotWidthInches) || legacySize, 1, 36);
    state.gridSizeInches = clamp(Number(parsed.gridSizeInches) || 12, 1, 72);
    state.gridVisible = parsed.gridVisible !== false;
  } catch {
    localStorage.removeItem(STORAGE_KEY);
  }
  ensurePathModes();
}

function saveState() {
  ensurePathModes();
  localStorage.setItem(
    STORAGE_KEY,
    JSON.stringify({
      poses: state.poses,
      pathModes: state.pathModes,
      selected: state.selected,
      robotLengthInches: state.robotLengthInches,
      robotWidthInches: state.robotWidthInches,
      gridSizeInches: state.gridSizeInches,
      gridVisible: state.gridVisible,
    }),
  );
}

function ensurePathModes() {
  const segmentCount = Math.max(0, state.poses.length - 1);
  state.pathModes = Array.from({ length: segmentCount }, (_, index) => {
    const mode = state.pathModes[index];
    return mode === "Tangential" || mode === "Constant" || mode === "Linear" ? mode : "Linear";
  });
}

function historySnapshot() {
  ensurePathModes();
  return {
    poses: state.poses.map((pose) => ({ ...pose })),
    pathModes: [...state.pathModes],
    selected: state.selected,
    robotLengthInches: state.robotLengthInches,
    robotWidthInches: state.robotWidthInches,
    gridSizeInches: state.gridSizeInches,
    gridVisible: state.gridVisible,
  };
}

function applyHistorySnapshot(snapshot) {
  state.poses = snapshot.poses.map((pose) => ({ ...pose }));
  state.pathModes = [...snapshot.pathModes];
  state.selected = clamp(snapshot.selected, 0, Math.max(0, state.poses.length - 1));
  state.robotLengthInches = snapshot.robotLengthInches;
  state.robotWidthInches = snapshot.robotWidthInches;
  state.gridSizeInches = snapshot.gridSizeInches;
  state.gridVisible = snapshot.gridVisible;
  ensurePathModes();
  syncUi();
  saveState();
  draw();
}

function commitHistory() {
  const snapshot = historySnapshot();
  const current = state.history[state.historyIndex];
  if (current && JSON.stringify(current) === JSON.stringify(snapshot)) return;

  state.history = state.history.slice(0, state.historyIndex + 1);
  state.history.push(snapshot);
  if (state.history.length > 300) {
    state.history.shift();
  }
  state.historyIndex = state.history.length - 1;
}

function resetHistory() {
  state.history = [historySnapshot()];
  state.historyIndex = 0;
}

function clearRedo() {
  state.history = state.history.slice(0, state.historyIndex + 1);
}

function finishChange() {
  commitHistory();
  syncUi();
  saveState();
  draw();
}

function updateCanvasSize() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(640, Math.round(rect.width * dpr));
  const height = Math.max(320, Math.round(rect.height * dpr));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  const padding = 0;
  const scale = Math.min((canvas.width - padding * 2) / VIEW_WIDTH, (canvas.height - padding * 2) / VIEW_HEIGHT);
  state.view.scale = scale;
  state.view.left = (canvas.width - VIEW_WIDTH * scale) / 2;
  state.view.top = (canvas.height - VIEW_HEIGHT * scale) / 2;
}

function worldToScreen(point) {
  return {
    x: state.view.left + point.y * state.view.scale,
    y: state.view.top + point.x * state.view.scale,
  };
}

function screenToWorld(point) {
  return {
    x: clamp((point.y - state.view.top) / state.view.scale, 0, FIELD_WIDTH),
    y: clamp((point.x - state.view.left) / state.view.scale, 0, FIELD_HEIGHT),
  };
}

function snapPose(pose) {
  const grid = Math.max(1, state.gridSizeInches);
  return {
    ...pose,
    x: clamp(Math.round(pose.x / grid) * grid, 0, FIELD_WIDTH),
    y: clamp(Math.round(pose.y / grid) * grid, 0, FIELD_HEIGHT),
  };
}

function pointerPosition(event) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}

function drawFieldBackground() {
  const { left, top, scale } = state.view;
  const width = VIEW_WIDTH * scale;
  const height = VIEW_HEIGHT * scale;

  ctx.save();
  ctx.fillStyle = "#2a2927";
  ctx.fillRect(left, top, width, height);

  if (fieldImage.complete && fieldImage.naturalWidth > 0) {
    ctx.drawImage(fieldImage, left, top, width, height);
  }

  ctx.strokeStyle = "#f2f0ea";
  ctx.lineWidth = 2;
  ctx.strokeRect(left, top, width, height);

  if (state.gridVisible) {
    ctx.strokeStyle = "rgba(255,255,255,0.13)";
    ctx.lineWidth = 1;
    const grid = Math.max(1, state.gridSizeInches);
    for (let x = 0; x <= FIELD_WIDTH; x += grid) {
      const sy = top + x * scale;
      ctx.beginPath();
      ctx.moveTo(left, sy);
      ctx.lineTo(left + width, sy);
      ctx.stroke();
    }
    for (let y = 0; y <= FIELD_HEIGHT; y += grid) {
      const sx = left + y * scale;
      ctx.beginPath();
      ctx.moveTo(sx, top);
      ctx.lineTo(sx, top + height);
      ctx.stroke();
    }
  }

  ctx.restore();
}

function drawPath() {
  if (state.poses.length < 2) return;

  ctx.save();
  ctx.strokeStyle = "#31c48d";
  ctx.lineWidth = 4;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  state.poses.forEach((pose, index) => {
    const point = worldToScreen(pose);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawRobot(pose, selected) {
  const point = worldToScreen(pose);
  const dpr = window.devicePixelRatio || 1;

  if (!selected) {
    ctx.save();
    ctx.fillStyle = "#fffaf1";
    ctx.strokeStyle = "rgba(0, 0, 0, 0.72)";
    ctx.lineWidth = Math.max(1.5, 1.5 * dpr);
    ctx.beginPath();
    ctx.arc(point.x, point.y, 5.5 * dpr, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
    return;
  }

  const headingRadians = screenHeadingRadians(pose.heading);
  const length = state.robotLengthInches * state.view.scale;
  const width = state.robotWidthInches * state.view.scale;
  const halfLength = length / 2;
  const halfWidth = width / 2;

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(headingRadians);
  if (robotImage.complete && robotImage.naturalWidth > 0) {
    ctx.drawImage(robotImage, -halfLength, -halfWidth, length, width);
  } else {
    ctx.fillStyle = "rgba(96, 96, 96, 0.85)";
    ctx.fillRect(-halfLength, -halfWidth, length, width);
  }
  if (selected) {
    ctx.globalCompositeOperation = "source-atop";
    ctx.fillStyle = "rgba(31, 214, 107, 0.42)";
    ctx.fillRect(-halfLength, -halfWidth, length, width);
    ctx.globalCompositeOperation = "source-over";
  }
  if (selected) {
    ctx.strokeStyle = "#20d66b";
    ctx.lineWidth = Math.max(3, 3 * dpr);
    ctx.strokeRect(-halfLength, -halfWidth, length, width);
  }
  ctx.restore();

  const handle = headingHandle(pose);
  const handleRadius = Math.hypot(handle.x - point.x, handle.y - point.y);
  ctx.save();
  ctx.strokeStyle = "rgba(255, 250, 241, 0.26)";
  ctx.lineWidth = Math.max(1.5, 1.5 * dpr);
  ctx.beginPath();
  ctx.arc(point.x, point.y, handleRadius, headingRadians - Math.PI / 2, headingRadians + Math.PI / 2);
  ctx.stroke();

  ctx.strokeStyle = selected ? "#e05d44" : "rgba(255,255,255,0.58)";
  ctx.fillStyle = selected ? "#fffaf1" : "#31c48d";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(point.x, point.y);
  ctx.lineTo(handle.x, handle.y);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(handle.x, handle.y, selected ? 6 * dpr : 4 * dpr, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function headingHandle(pose) {
  const point = worldToScreen(pose);
  const headingRadians = screenHeadingRadians(pose.heading);
  const dpr = window.devicePixelRatio || 1;
  const robotHalfLength = state.robotLengthInches * state.view.scale / 2;
  const length = robotHalfLength + 14 * dpr;
  return {
    x: point.x + Math.cos(headingRadians) * length,
    y: point.y + Math.sin(headingRadians) * length,
  };
}

function screenHeadingRadians(heading) {
  return (90 - heading) * Math.PI / 180;
}

function drawPoses() {
  ctx.save();
  state.poses.forEach((pose, index) => {
    const selected = index === state.selected;
    drawRobot(pose, selected);
    const point = worldToScreen(pose);
    ctx.fillStyle = "#fffaf1";
    ctx.font = `${12 * (window.devicePixelRatio || 1)}px Inter, sans-serif`;
    if (!selected) {
      ctx.fillText(String(index + 1), point.x + 12, point.y - 12);
    }
  });
  ctx.restore();
}

function draw() {
  updateCanvasSize();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawFieldBackground();
  drawPath();
  drawPoses();
}

function nearestPose(screenPoint) {
  const radius = 16 * (window.devicePixelRatio || 1);
  let best = { index: -1, distance: Infinity, part: "body" };

  state.poses.forEach((pose, index) => {
    const body = worldToScreen(pose);
    const handle = headingHandle(pose);
    const local = screenToRobotLocal(screenPoint, pose);
    const halfLength = state.robotLengthInches * state.view.scale / 2;
    const halfWidth = state.robotWidthInches * state.view.scale / 2;
    const insideRobot = index === state.selected && Math.abs(local.x) <= halfLength && Math.abs(local.y) <= halfWidth;
    const bodyDistance = Math.hypot(screenPoint.x - body.x, screenPoint.y - body.y);
    const handleDistance = Math.hypot(screenPoint.x - handle.x, screenPoint.y - handle.y);

    if (index === state.selected && handleDistance < radius && handleDistance < best.distance) {
      best = { index, distance: handleDistance, part: "heading" };
    }
    if (insideRobot && best.distance > 0) {
      best = { index, distance: 0, part: "body" };
    }
    if (bodyDistance < radius && bodyDistance < best.distance) {
      best = { index, distance: bodyDistance, part: "body" };
    }
  });

  return best.index === -1 ? null : best;
}

function screenToRobotLocal(screenPoint, pose) {
  const body = worldToScreen(pose);
  const radians = screenHeadingRadians(pose.heading);
  const dx = screenPoint.x - body.x;
  const dy = screenPoint.y - body.y;
  return {
    x: dx * Math.cos(radians) + dy * Math.sin(radians),
    y: -dx * Math.sin(radians) + dy * Math.cos(radians),
  };
}

function setSelected(index) {
  state.selected = clamp(index, 0, Math.max(0, state.poses.length - 1));
  syncUi();
  saveState();
  draw();
}

function selectedPose() {
  return state.poses[state.selected] || null;
}

function addPose(pose, options = {}) {
  if (!options.keepRedo) clearRedo();
  state.poses.push({
    x: round(clamp(pose.x, 0, FIELD_WIDTH)),
    y: round(clamp(pose.y, 0, FIELD_HEIGHT)),
    heading: round(pose.heading ?? 0, 1),
    name: pose.name || `Pose ${state.poses.length + 1}`,
  });
  ensurePathModes();
  state.selected = state.poses.length - 1;
  finishChange();
}

function updateSelectedPose(patch) {
  const pose = selectedPose();
  if (!pose) return;
  clearRedo();

  Object.assign(pose, patch);
  pose.x = round(clamp(Number(pose.x) || 0, 0, FIELD_WIDTH));
  pose.y = round(clamp(Number(pose.y) || 0, 0, FIELD_HEIGHT));
  pose.heading = round(Number(pose.heading) || 0, 1);
  finishChange();
}

function pathLength() {
  return state.poses.reduce((total, pose, index) => {
    if (index === 0) return 0;
    const previous = state.poses[index - 1];
    return total + Math.hypot(pose.x - previous.x, pose.y - previous.y);
  }, 0);
}

function syncInputs() {
  const pose = selectedPose();
  const disabled = !pose;
  [controls.xInput, controls.yInput, controls.headingInput, controls.nameInput].forEach((input) => {
    input.disabled = disabled;
  });

  if (!pose) {
    controls.xInput.value = "";
    controls.yInput.value = "";
    controls.headingInput.value = "";
    controls.nameInput.value = "";
    controls.robotLengthInput.value = state.robotLengthInches;
    controls.robotWidthInput.value = state.robotWidthInches;
    controls.gridSizeInput.value = state.gridSizeInches;
    return;
  }

  controls.xInput.value = pose.x;
  controls.yInput.value = pose.y;
  controls.headingInput.value = pose.heading;
  controls.nameInput.value = pose.name;
  controls.robotLengthInput.value = state.robotLengthInches;
  controls.robotWidthInput.value = state.robotWidthInches;
  controls.gridSizeInput.value = state.gridSizeInches;
}

function syncPoseList() {
  ensurePathModes();
  controls.poseList.innerHTML = "";

  state.poses.slice(1).forEach((pose, index) => {
    const previous = state.poses[index];
    const item = document.createElement("li");
    const row = document.createElement("div");
    row.className = index + 1 === state.selected ? "path-row is-selected" : "path-row";
    row.addEventListener("click", () => setSelected(index + 1));

    const number = document.createElement("span");
    number.className = "pose-index";
    number.textContent = String(index + 1).padStart(2, "0");

    const info = document.createElement("span");
    info.className = "path-info";

    const name = document.createElement("span");
    name.className = "pose-name";
    name.textContent = `Path ${index + 1}`;

    const coords = document.createElement("span");
    coords.className = "pose-coords";
    coords.textContent = `${round(previous.x, 1)}, ${round(previous.y, 1)} -> ${round(pose.x, 1)}, ${round(pose.y, 1)}`;

    const mode = document.createElement("select");
    mode.className = "path-mode";
    mode.setAttribute("aria-label", `Path ${index + 1} heading mode`);
    ["Linear", "Constant", "Tangential"].forEach((optionName) => {
      const option = document.createElement("option");
      option.value = optionName;
      option.textContent = optionName;
      option.selected = state.pathModes[index] === optionName;
      mode.append(option);
    });
    mode.addEventListener("click", (event) => event.stopPropagation());
    mode.addEventListener("change", (event) => {
      clearRedo();
      state.pathModes[index] = event.target.value;
      finishChange();
    });

    info.append(name, coords);
    row.append(number, info, mode);
    item.append(row);
    controls.poseList.append(item);
  });
}

function exportJava() {
  ensurePathModes();
  const rawBaseName = controls.pathNameInput.value.trim() || "Path";
  const baseName = rawBaseName.replace(/\d+$/, "") || rawBaseName;

  if (state.poses.length < 2) {
    return "// Add at least two poses to export a Pedro path.";
  }

  const pathNames = state.poses.slice(1).map((_, index) => `${baseName}${index + 1}`);
  const lines = [
    "public static class Paths {",
    ...pathNames.map((pathName) => `    public PathChain ${pathName};`),
    "",
    "    public Paths(Follower follower) {",
  ];

  for (let index = 1; index < state.poses.length; index += 1) {
    const previous = state.poses[index - 1];
    const pose = state.poses[index];
    const pathName = pathNames[index - 1];
    lines.push(`        ${pathName} = follower.pathBuilder().addPath(`);
    lines.push("                new BezierLine(");
    lines.push(`                        new Pose(${previous.x.toFixed(3)}, ${previous.y.toFixed(3)}),`);
    lines.push("");
    lines.push(`                        new Pose(${pose.x.toFixed(3)}, ${pose.y.toFixed(3)})`);
    lines.push("                )");
    lines.push("        )");
    lines.push(`        ${headingInterpolationJava(state.pathModes[index - 1], previous, pose)}`);
    lines.push("        .build();");
    lines.push("");
  }
  lines.push("    }");
  lines.push("}");
  return lines.join("\n");
}

function headingInterpolationJava(mode, previous, pose) {
  if (mode === "Tangential") {
    return ".setTangentHeadingInterpolation()";
  }
  if (mode === "Constant") {
    return `.setConstantHeadingInterpolation(Math.toRadians(${previous.heading.toFixed(1)}))`;
  }
  return `.setLinearHeadingInterpolation(Math.toRadians(${previous.heading.toFixed(1)}), Math.toRadians(${pose.heading.toFixed(1)}))`;
}

function syncUi() {
  syncInputs();
  syncPoseList();
  const segmentCount = Math.max(0, state.poses.length - 1);
  controls.pointCount.textContent = `${segmentCount} ${segmentCount === 1 ? "path" : "paths"}`;
  controls.pathLength.textContent = `${round(pathLength(), 1)} in`;
  controls.toggleGridButton.textContent = state.gridVisible ? "▦ Grid On" : "▢ Grid Off";
  controls.toggleGridButton.classList.toggle("is-active", state.gridVisible);
  controls.undoButton.disabled = state.historyIndex <= 0;
  controls.redoButton.disabled = state.historyIndex >= state.history.length - 1;
  controls.exportText.value = exportJava();
}

function parseImportedPoses(text) {
  const rawPoses = [];
  const poses = [];
  const modes = [];
  const posePattern = /new\s+Pose\s*\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)(?:\s*,\s*Math\.toRadians\s*\(\s*(-?\d+(?:\.\d+)?)\s*\)|\s*,\s*(-?\d+(?:\.\d+)?))?\s*\)/g;
  const headings = [];
  const headingPattern = /setLinearHeadingInterpolation\s*\(\s*Math\.toRadians\s*\(\s*(-?\d+(?:\.\d+)?)\s*\)\s*,\s*Math\.toRadians\s*\(\s*(-?\d+(?:\.\d+)?)\s*\)\s*\)/g;
  const modePattern = /set(Tangent|Constant|Linear)HeadingInterpolation/g;

  for (const match of text.matchAll(modePattern)) {
    modes.push(match[1] === "Tangent" ? "Tangential" : match[1]);
  }

  for (const match of text.matchAll(headingPattern)) {
    headings.push([Number(match[1]), Number(match[2])]);
  }

  for (const match of text.matchAll(posePattern)) {
    rawPoses.push({
      x: Number(match[1]),
      y: Number(match[2]),
      heading: Number(match[3] ?? match[4]),
    });
  }

  rawPoses.forEach((rawPose, index) => {
    const segmentIndex = Math.floor(index / 2);
    const isSegmentEnd = index % 2 === 1;
    let heading = rawPose.heading;
    if (!Number.isFinite(heading)) {
      heading = headings[segmentIndex]?.[isSegmentEnd ? 1 : 0] ?? poses[poses.length - 1]?.heading ?? 0;
    }
    const pose = {
      x: clamp(rawPose.x, 0, FIELD_WIDTH),
      y: clamp(rawPose.y, 0, FIELD_HEIGHT),
      heading,
      name: `Pose ${poses.length + 1}`,
    };
    const previous = poses[poses.length - 1];
    if (previous && previous.x === pose.x && previous.y === pose.y && previous.heading === pose.heading) return;
    if (previous && previous.x === pose.x && previous.y === pose.y) {
      modes.splice(poses.length - 1, 0, "Linear");
    }
    poses.push(pose);
  });

  return { poses, modes };
}

canvas.addEventListener("contextmenu", (event) => {
  event.preventDefault();
});

canvas.addEventListener("pointerdown", (event) => {
  const screen = pointerPosition(event);

  if (event.button === 2) {
    const world = event.shiftKey ? snapPose(screenToWorld(screen)) : screenToWorld(screen);
    addPose({ ...world, heading: selectedPose()?.heading ?? 0 });
    state.dragging = "body";
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
    return;
  }

  if (event.button !== 0) return;

  const hit = nearestPose(screen);

  if (hit) {
    setSelected(hit.index);
    state.dragging = hit.part;
  } else {
    state.dragging = null;
    return;
  }

  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener("pointermove", (event) => {
  if (!state.dragging) return;

  const screen = pointerPosition(event);
  const pose = selectedPose();
  if (!pose) return;

  if (state.dragging === "heading") {
    const body = worldToScreen(pose);
    const radians = Math.atan2(screen.y - body.y, screen.x - body.x);
    updateSelectedPose({ heading: round((90 - radians * 180 / Math.PI + 360) % 360, 1) });
  } else {
    if (!event.shiftKey) return;
    const world = snapPose(screenToWorld(screen));
    updateSelectedPose(world);
  }
});

canvas.addEventListener("pointerup", (event) => {
  state.dragging = null;
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
});

canvas.addEventListener("pointercancel", () => {
  state.dragging = null;
});

controls.addPointButton.addEventListener("click", () => {
  addPose({ x: FIELD_WIDTH / 2, y: FIELD_HEIGHT / 2, heading: 0 });
});

controls.undoButton.addEventListener("click", () => {
  if (state.historyIndex <= 0) return;
  state.historyIndex -= 1;
  applyHistorySnapshot(state.history[state.historyIndex]);
});

controls.redoButton.addEventListener("click", () => {
  if (state.historyIndex >= state.history.length - 1) return;
  state.historyIndex += 1;
  applyHistorySnapshot(state.history[state.historyIndex]);
});

controls.mirrorButton.addEventListener("click", () => {
  clearRedo();
  state.poses = state.poses.map((pose) => ({
    ...pose,
    x: round(FIELD_WIDTH - pose.x),
    heading: round((180 - pose.heading + 360) % 360, 1),
  }));
  ensurePathModes();
  finishChange();
});

controls.toggleGridButton.addEventListener("click", () => {
  clearRedo();
  state.gridVisible = !state.gridVisible;
  finishChange();
});

controls.clearButton.addEventListener("click", () => {
  clearRedo();
  state.poses = [];
  state.pathModes = [];
  state.selected = 0;
  finishChange();
});

controls.deletePointButton.addEventListener("click", () => {
  if (!selectedPose()) return;
  clearRedo();
  state.poses.splice(state.selected, 1);
  ensurePathModes();
  state.selected = clamp(state.selected, 0, Math.max(0, state.poses.length - 1));
  finishChange();
});

controls.xInput.addEventListener("change", () => updateSelectedPose({ x: Number(controls.xInput.value) }));
controls.yInput.addEventListener("change", () => updateSelectedPose({ y: Number(controls.yInput.value) }));
controls.headingInput.addEventListener("change", () => updateSelectedPose({ heading: Number(controls.headingInput.value) }));
controls.nameInput.addEventListener("change", () => updateSelectedPose({ name: controls.nameInput.value.trim() || `Pose ${state.selected + 1}` }));
controls.robotLengthInput.addEventListener("input", () => {
  clearRedo();
  state.robotLengthInches = clamp(Number(controls.robotLengthInput.value) || 18, 1, 36);
  finishChange();
});
controls.robotWidthInput.addEventListener("input", () => {
  clearRedo();
  state.robotWidthInches = clamp(Number(controls.robotWidthInput.value) || 18, 1, 36);
  finishChange();
});
controls.gridSizeInput.addEventListener("input", () => {
  clearRedo();
  state.gridSizeInches = clamp(Number(controls.gridSizeInput.value) || 12, 1, 72);
  finishChange();
});
controls.pathNameInput.addEventListener("input", syncUi);
controls.speedInput.addEventListener("input", syncUi);

controls.importButton.addEventListener("click", () => {
  const { poses, modes } = parseImportedPoses(controls.importText.value);
  if (poses.length === 0) return;
  clearRedo();
  state.poses = poses;
  state.pathModes = modes;
  ensurePathModes();
  state.selected = 0;
  finishChange();
});

controls.copyButton.addEventListener("click", async () => {
  await navigator.clipboard.writeText(controls.exportText.value);
  controls.copyButton.textContent = "✅ Copied";
  setTimeout(() => {
    controls.copyButton.textContent = "📋 Copy Java";
  }, 900);
});

window.addEventListener("resize", draw);
fieldImage.addEventListener("load", draw);
robotImage.addEventListener("load", draw);

loadState();
resetHistory();
syncUi();
draw();
