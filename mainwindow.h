#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"
#include "Graph.h"
#include <random>
#include <qundostack.h>
#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>

class Graph : public QMainWindow
{
    Q_OBJECT

public:
    Graph(QWidget *parent = nullptr);
    ~Graph();

    //Update methods:

    /*errorMessage(): If user has entered an invalid input, present error message.*/
    void errorMessage();
    
    /*update_edge_displayed(): As user constructs graph, update suggested
    edge and edge count information etc.*/
    void update_edge_displayed();

    /*update_nodes(bool add): Update node coordinates in scene if user has added or removed
    nodes. Default argument set to add.*/
    void update_nodes(bool add);

    /*updateUndoButtons(): Controls enabling/disabling of undo/redo buttons for edge addition
    depending on choices from user and command stack.*/
    void updateUndoButtons();
    
    //Draw methods:

    /*draw_nodes_coloured(std::map<int, std::vector<double> > colours):
    Draws nodes in scene coloured according to the colour map: colours.
    Node i will be coloured with colours[i] which is a vector of rgb values.*/
    void draw_nodes_coloured(std::map<int, std::vector<double> > colours);

    /*draw_nodes(): draws nodes using default pen and brush based on their coordinates.*/
    void draw_nodes();

    /*draw_node(int V, QPen pen = defaultPen, QBrush brush=defaultBrush):
    Draws a singular node V on scene using pen and brush.*/
    void draw_node(int V, QPen pen = defaultPen, QBrush brush=defaultBrush);

    /*draw_edge_line(int i, int j, QPen pen = defaultPen):
    draws a line from node i to node j with pen.*/
    void draw_edge_line(int i, int j, QPen pen = defaultPen);

    /*draw_edges_hyp_poincaire(int i, int j,QPen pen = defaultPen):
    draws hyperbolic geodesic from node i to node j, where i and j are positioned on a
    hyperbolic disc. */
    void draw_edges_hyp_poincaire(int i, int j,QPen pen = defaultPen);

    /*draw_edges_hyp(int i, int j,QPen pen = defaultPen):
    draws hyperbolic geodesic from node i to node j in uper half plane
    model of hyperbolic space.*/
    void draw_edges_hyp(int i, int j,QPen pen = defaultPen);

    /*draw_edge(int i, int j,QPen pen = defaultPen): controls edge drawing between
    node i and node j, depending on what geometry is used*/
    void draw_edge(int i, int j,QPen pen = defaultPen);

    /*redraw_edges(QPen pen = defaultPen): redraws all edges. 
    Generall called when nodes coordinates are modified.*/
    void redraw_edges(QPen pen = defaultPen);

    /*draw_connected_components(): drae connected components of a graph G.*/
    void draw_connected_components();

   /*Traverse(int u, std::map<int, bool>& visited, std::map<int,
        std::vector<int>>& new_adjs, int& edges, std::map<int, bool>& visited2,
        QPen& pen, QBrush& brush):
        Called upon when using Depth First Search (DFS) to find and draw traversal of
        connected components. */
    void Traverse(int u, std::map<int, bool>& visited, std::map<int,
        std::vector<int>>& new_adjs, int& edges, std::map<int, bool>& visited2,
        QPen& pen, QBrush& brush);

    /*DFS method. */
    void DFS();

    /*fixedDesign(): Enables add/remove vertex buttons for when constructing a graph with
    fixed layout/design.*/
    void fixedDesign();
    
    //coordinate setting methods based on graph layouts
    void set_node_coords();
    void setLatticeCoords();
    void setRandomCircCoords();
    void setUniformCircCoords();
    void setRandomCoords();

    /*getAllEdges(int n): returns a list of all possible
    edges given n nodes*/
    std::vector<std::vector< int>> getAllEdges(int n);

    /*Gets list of rgb colours for each node from colour pallete 
    given coloursNeeded and colouring label for each node */
    std::map<int, std::vector<double> >getColours(int coloursNeeded, std::map<int, int> colouring);
    
    /*findNextToBeAdded(): find next edge that can be added to graph by a user*/
    void findNextToBeAdded();

    /*makeRandomGraph(bool sparse): generate a random graph with optional argument
    to make it have fewer edges*/
    void makeRandomGraph(bool sparse);
    
    /*mousePressEvent(QMouseEvent *event): Handles event user clicks on
    graphics scene*/
    void mousePressEvent(QMouseEvent *event);

    //methods for checking if click from user was located
    //inScene and if click was on a node or not
    bool inScene(QPointF &P);
    bool onNode(QPointF &P, int& nodeIndex);

    std::vector<int> undoStack;
    std::vector<int> redoStack;
    
//button slots
private slots:

    void on_add_node_clicked();

    void on_remove_node_clicked();

    void on_enter_node_textEdited(const QString& arg1);

    void on_add_edge_clicked();

    void on_reject_edge_clicked();

    void on_clear_clicked();

    void on_auto_fill_clicked();

    void on_random_clicked();

    void on_calc_clicked();

    void on_greedy_clicked();

    void on_random_layout_clicked();
    
    void on_circ_uniform_layout_clicked();

    void on_circ_rand_layout_clicked();

    void on_lattice_layout_clicked();

    void on_euclidean_geometry_clicked();

    void on_hyperbolic_geometry_clicked();

    void on_undo_clicked();

    void on_redo_clicked();

    void on_custom_layout_clicked();

    void on_DFS_clicked();

    void on_connected_clicked();

    void on_sparse_clicked();


private:
    Ui::GraphClass ui;
    QGraphicsScene* scene;
    
    // number of vetices and max number of nodes resp.
    int nodes;
    int max;

    //node size parameters
    int node_w; 
    int node_h;

    //graph layout parameters for circle and lattice layouts
    int radius;
    int lattice_size;

    //constant pi =3.14..
    double const pi = std::atan(1) * 4;;

    //container for nodes coordinates. Accesed by index.
    //coordinates of node i are given by node_coords[i]
    std::vector<std::vector<double> > node_coords;

    //container for all possible edges given N nodes.
    std::vector<std::vector<int>> edgelist;

    //edges added to graph by user
    std::vector<std::vector<int>> selected_edgelist;

    //if user has used mouse to click on a node, store node
    //in container for later use
    std::vector<int> user_selected_node;

    //For keeping track of edge suggested to user
    int selected_edge;

    //w,h: width and height of scene
    int w;
    int h;

    //distributions for generating random node coordinates
    std::uniform_real_distribution<double> WIDTH;
    std::uniform_real_distribution<double> HEIGHT;
    std::uniform_real_distribution<double> theta;
    std::default_random_engine eng;

    //graph G to be constructed/modified
    graph G;

    //default pen and brush to draw nodes
    static QPen defaultPen;
    static QBrush defaultBrush;

    //other pens/brushes to use
    QPen blackPen; 
    QBrush pinkBrush;
    QBrush blackBrush;
    QBrush redBrush;
    QBrush blueBrush;
    QBrush highlight1_brush;
    QBrush highlight2_brush;
    QPen highlight1_pen;
    QPen highlight2_pen;



  //Colour pallete for drawing nodes in different colours
     std::vector<std::vector<double> > colourPalette = 
    { {1.0,0.0,0.0},{1.0,0.039999999999999994,0.0},{1.0,0.07999999999999999,0.0},
        {1.0,0.12,0.0},{1.0,0.15999999999999998,0.0},{1.0,0.2,0.0},{1.0,0.24,0.0},{1.0,0.27999999999999997,0.0},
        {1.0,0.31999999999999995,0.0},{1.0,0.36,0.0},{1.0,0.4,0.0},{1.0,0.44,0.0},{1.0,0.48,0.0},{1.0,0.5199999999999999,0.0},
        {1.0,0.5599999999999999,0.0},{1.0,0.6000000000000001,0.0},{1.0,0.6399999999999999,0.0},{1.0,0.6799999999999999,0.0},
        {1.0,0.72,0.0},{1.0,0.7599999999999999,0.0},{1.0,0.8,0.0},{1.0,0.8400000000000001,0.0},{1.0,0.88,0.0},
        {1.0,0.9199999999999999,0.0},{1.0,0.96,0.0},{1.0,0.9999999999999998,0.0},{0.9600000000000002,1.0,0.0},
        {0.9199999999999999,1.0,0.0},{0.8799999999999997,1.0,0.0},{0.8400000000000001,1.0,0.0},{0.7999999999999998,1.0,0.0},
        {0.7600000000000002,1.0,0.0},{0.72,1.0,0.0},{0.6799999999999997,1.0,0.0},{0.6400000000000001,1.0,0.0},
        {0.5999999999999999,1.0,0.0},{0.5600000000000003,1.0,0.0},{0.52,1.0,0.0},{0.47999999999999976,1.0,0.0},
        {0.44000000000000017,1.0,0.0},{0.3999999999999999,1.0,0.0},{0.35999999999999965,1.0,0.0},
        {0.32000000000000006,1.0,0.0},{0.28000000000000047,1.0,0.0},{0.23999999999999955,1.0,0.0},
        {0.19999999999999996,1.0,0.0},{0.16000000000000036,1.0,0.0},{0.1200000000000001,1.0,0.0},
        {0.07999999999999985,1.0,0.0},{0.04000000000000026,1.0,0.0},{6.661338147750939e-16,1.0,0.0},
        {0.0,1.0,0.04000000000000026},{0.0,1.0,0.07999999999999985},{0.0,1.0,0.11999999999999977},
        {0.0,1.0,0.16000000000000003},{0.0,1.0,0.19999999999999996},{0.0,1.0,0.23999999999999988},
        {0.0,1.0,0.28000000000000014},{0.0,1.0,0.32000000000000006},{0.0,1.0,0.36},{0.0,1.0,0.40000000000000024},
        {0.0,1.0,0.43999999999999984},{0.0,1.0,0.47999999999999976},{0.0,1.0,0.52},{0.0,1.0,0.5599999999999999}
    ,{0.0,1.0,0.5999999999999999},{0.0,1.0,0.6400000000000001},{0.0,1.0,0.68},{0.0,1.0,0.72},{0.0,1.0,0.7600000000000002},
        {0.0,1.0,0.7999999999999998},{0.0,1.0,0.8399999999999997},{0.0,1.0,0.88},{0.0,1.0,0.9199999999999999},
        {0.0,1.0,0.9599999999999999},{0.0,0.9999999999999998,1.0},{0.0,0.9600000000000002,1.0}
    ,{0.0,0.9199999999999999,1.0},{0.0,0.8799999999999997,1.0},{0.0,0.8400000000000001,1.0},
        {0.0,0.7999999999999998,1.0},{0.0,0.7599999999999996,1.0},{0.0,0.72,1.0},{0.0,0.6800000000000004,1.0},
        {0.0,0.6399999999999995,1.0},{0.0,0.5999999999999999,1.0},{0.0,0.5600000000000003,1.0},{0.0,0.52,1.0},
        {0.0,0.47999999999999976,1.0},{0.0,0.44000000000000017,1.0},{0.0,0.3999999999999999,1.0},
        {0.0,0.35999999999999965,1.0},{0.0,0.32000000000000006,1.0},{0.0,0.2799999999999998,1.0},
        {0.0,0.2400000000000002,1.0},{0.0,0.19999999999999996,1.0},{0.0,0.1599999999999997,1.0},
        {0.0,0.1200000000000001,1.0},{0.0,0.07999999999999985,1.0},
        {0.0,0.03999999999999959,1.0},{0.0,6.661338147750939e-16,1.0} };
    
};
