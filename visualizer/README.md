# FRI Pedro Pathing Visualizer

Standalone browser tool for planning Pedro Pathing poses on a landscape First Robotics Initiative field with a rotated 144 in x 288 in coordinate system.

- Coordinate bottom half: FTC Decode-sized 144 in x 144 in area.
- Coordinate top half: custom FRI 144 in x 144 in area.
- Origin: top-left of the landscape field view.
- Export: Java `BezierLine` path builder snippets using `Pose(x, y)` and linear heading interpolation.

Open `index.html` in a browser. Click the field to add poses, drag pose markers to move them, and drag the heading handle to aim the robot.
Set robot length and width in inches from the Selected Pose panel to match the footprint drawn on the field.
Set grid size in inches from the same panel. Hold Shift while adding or dragging a pose to snap it to the grid.
Use the Grid button to hide or show the grid without changing snap behavior.
Each exported path segment has a heading mode: Linear, Constant, or Tangential. These export to Pedro's `setLinearHeadingInterpolation`, `setConstantHeadingInterpolation`, and `setTangentHeadingInterpolation` calls.
