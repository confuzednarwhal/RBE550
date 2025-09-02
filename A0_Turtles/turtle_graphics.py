import turtle as t
from PIL import Image

radius = 200
turn_angle = 120 # turn angle for equilateral triangles
start_heading = 40 # start angle
# makes 3 triangles
headings = [start_heading, start_heading+120, start_heading+240]

turtle = t.Turtle()

# draws a single equilateral triangle
def draw_triangle(heading):
    turtle.setheading(heading)
    turtle.forward(radius)

    turtle.right(turn_angle)
    turtle.forward(radius)

    turtle.right(turn_angle)
    turtle.forward(radius)

# draws multiple triangles based on given headings
for h in headings:
    draw_triangle(h)

# generates png to be saved
canvas = t.getcanvas()
canvas.postscript(file="turtle_search.eps")

img = Image.open("turtle_search.eps")
img.save("turtle_VSpattern.png", "png")
