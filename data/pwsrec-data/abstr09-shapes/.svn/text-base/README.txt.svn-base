README for shape data from Mi et al. 2009
-----------------------------------------

Included in this directory are the shape files shown in Figure 1, 15,
16 and 18 from:

   X. Mi, D. DeCarlo and M. Stone, "Abstraction of 2D Shapes in Terms
   of Parts", NPAR 2009

They are organized into four subdirectories, one for each figure.

The filename of the shapes are as follows:
   shapename-FORMAT.dat

   where FORMAT is one of:
      orig, absN, pmN, dpN, cfN

      for N=0 (low res) or N=1 (medium res)

Not all shapes have results at N=1, and not all formats are present in
all figures (i.e. fig 18 doesn't have PM, DP, CF).

For example:
  Australia-orig.dat
  Australia-abs0.dat
  Australia-pm0.dat
  Australia-dp0.dat
  Australia-cf0.dat

Shape File Format (.dat)
------------------------

Each shape is represented in terms of a set of polygons and is saved
in an ASCII file.  Each file starts with the number of polygons,
followed by the set of polygons.  Each polygon starts with the number
of vertices and then the list of vertices.  The vertices are listed
with x, y coordinates, written one per line.  Note that the number of
vertices could be 0, representing an empty polygon.  In this case, the
next line starts a new polygon.  Note that holes in polygons are not
supported.  A line starting with # is considered a comment.

Here is a simple example:

# Three polygons with one empty polygon
3
# The first polygon
8
-4 0
-3 -1
0 -1
3 -1
4 0
3 1
0 1
-3 1
# The second polygon
8
-4 4
-3.5 2
0 2
3.5 2
4 4
3 5
0 5
-3 5
# The third polygon is empty
0

-----
