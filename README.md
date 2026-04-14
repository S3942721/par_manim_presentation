Build low quality video:
`manim main.py MazePathPlanning -ql`

Run and open:
`manim -pqL main.py MazePathPlanning`

Build full quality:
`manim main.py MazePathPlanning`

Convert to html (needs manim slides package):
`manim-slides convert MazePathPlanning MazePathPlanning.html -ccontrols=true`

Convert to seemingly low quality pptx (TODO: iterate and see if missing final frames of video can be fixed):
`manim-slides convert MazePathPlanning MazePathPlanning.pptx`