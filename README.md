# Graph
Basic GUI for graph creation and minimal colouring. Created using Qt designer with C++. 

The UI allows the user to construct a graph and has a graphics window with a representation of the graph. A graph can be created manually when setting layout to custom. Clicking on the graph window will create a node at that point. Edges can also be manuall added by clicking on two nodes to join.
Alternatively one can use the button menu to construct the graph.

The graph.cpp file contains a graph class, whose object is the underlying graph that the user is interacting with. The mainwindow.cpp file gives Graph window which contains representation of the underlying graph and methods for drawing it. The polynomial.cpp file contains a basic implementation for polynomials needed to compute the chromatic polynomial.

The geometry of the graph can be altered from Euclidean to Hyperbolic.  Using a greedy colouring algorihtm a minimal colouring for a graph can be sought. DSatur and RFL colouring methods may be added laer. One can also show connected components and spanning trees using DFS.

A single exe installation file  can be downloaded here:
https://mega.nz/file/oG9XQLia#-y3b0ndJp7tBe5wvt6xh8wbgj8vHRm9xwIj4pS67bgY

For more on graph colouring see here:
https://en.wikipedia.org/wiki/Graph_coloring

![graph1](https://user-images.githubusercontent.com/67613774/202884781-76716667-b3c7-4c9b-afde-b35ac6bcf544.png)


:![graph2](https://user-images.githubusercontent.com/67613774/202884789-17145565-a036-4a82-bebd-e3f1a5c5bd08.png)
![graph3](https://user-images.githubusercontent.com/67613774/202884790-66522172-d1c1-46b1-a2ee-9ffc3ed81028.png)






![Untitled](https://user-images.githubusercontent.com/67613774/202887448-9999537c-1376-466f-a733-e330adbfebf9.png)
