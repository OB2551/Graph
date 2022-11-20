#include "mainwindow.h"
#include <regex>
#include <iostream>
#include "Polynomial.h"
#include <QPainterPath>
#include <QGraphicsItem>
#include <random>
#include <chrono>
#include <thread>



QPen Graph::defaultPen = QPen(QColor::fromRgb(168,168,168));
QBrush Graph::defaultBrush = QBrush(QColor::fromRgb(168, 168, 168));

Graph::Graph(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    
    scene = new QGraphicsScene(this);
    
    ui.graphicsView->setScene(scene);
    ui.graphicsView->setMouseTracking(true);
    //buffer to keep drawing of items from going to close to border
    int buffer = 20;
    

    //initalise attributes
    nodes = 0;
    node_coords = {};
    edgelist = {};
    selected_edgelist = {};
    max = 100;
    node_w = 15;
    node_h = 15;
    radius = 350;
    lattice_size = 700;
    selected_edge = 0;
    user_selected_node = {};
    undoStack = {};
    redoStack = {};
    G = graph();//<- underlying graph G
    w = ui.graphicsView->width();
    h = ui.graphicsView->height();
    ui.graphicsView->setSceneRect(-w/2,-h/2,w, h);


    //distributions for generating (uniformly) random coordinates
    WIDTH = std::uniform_real_distribution<double>(-w/2+buffer, w/2-buffer);
    HEIGHT = std::uniform_real_distribution<double>(-h/2+buffer, h/2-buffer);
    theta = std::uniform_real_distribution<double>(0, 2*pi);
    std::random_device rd;
    std::default_random_engine eng(rd());

    //set defaults:
    ui.circ_uniform_layout->setChecked(true);//default layout of nodes
    ui.euclidean_geometry->setChecked(true);//default geometry of plane
    ui.undo->setEnabled(false);
    ui.redo->setEnabled(false);
    ui.add_edge->setEnabled(false);
    ui.reject_edge->setEnabled(false);
    ui.enter_node->setPlaceholderText("0");
   

    //initialise pens/brush objects
    blackPen = QPen(Qt::black);
    blackBrush = QBrush(Qt::black);
    redBrush = QBrush(Qt::red);
    blueBrush = QBrush(Qt::blue);
    pinkBrush = QBrush(QColor::fromRgb(250, 17, 242));
    highlight1_pen = QPen(QColor::fromRgb(0, 78, 255));
    highlight2_pen = QPen(QColor::fromRgb(0, 150, 255));
    highlight2_brush = QBrush(QColor::fromRgb(0, 150, 255));
    highlight1_brush = QBrush(QColor::fromRgb(0, 78, 255));
    
}

Graph::~Graph()
{}

  
//update methods
void Graph::errorMessage()
{  
    ui.enter_node->setText("");
    ui.enter_node->setPlaceholderText("Expected an integer 0<=N<100");
}
    
void Graph::update_edge_displayed() 
{    
    //update information displayed on ui
    ui.add_edge->setEnabled(true);
    ui.reject_edge->setEnabled(true);
    ui.start_node->setStyleSheet("QLabel { background-color : rgb(0, 78, 255) }");
    ui.end_node->setStyleSheet("QLabel { background-color : rgb(0, 150, 255) }");
    ui.enter_node->setText(QString::number(nodes));
    ui.possible_edge_count->setText(QString::number(edgelist.size()));
    ui.edge_count->setText(QString::number(G.edge_count));
    ui.chromnum->clear();
    ui.chrom_poly->clear();
    updateUndoButtons();
    
    if (edgelist.size() > 0 and selected_edge < edgelist.size())
    {

     
        //find next edge to display, user may have already added edges manually so 
        //we need to skip those.
        findNextToBeAdded();

        //If we can still add edges to graph
        if (selected_edge < edgelist.size())
        {
            //get next edge 
            std::vector<int>   newedge = edgelist[selected_edge];

            //suggest edge to user
            ui.end_node->setText(QString::number(newedge[1] + 1));
            ui.start_node->setText(QString::number(newedge[0] + 1));

            //higligh nodes on edges
            draw_node(newedge[0], highlight1_pen, highlight1_brush);
            draw_node(newedge[1], highlight2_pen, highlight2_brush);

            //exit
            return;
        }
    }
    //if no more edges can be added:
    ui.add_edge->setEnabled(false);
    ui.reject_edge->setEnabled(false);
    ui.end_node->clear();
    ui.start_node->clear();
    ui.start_node->setStyleSheet("QLabel { background-color : rgb(82,82,82) }");
    ui.end_node->setStyleSheet("QLabel { background-color : rgb(82,82,82) }");
}

void Graph::updateUndoButtons()
{
   //if edges can be added to graph
    if (edgelist.size() > 0)
    {
        //if edges have been added, make undo button enabled
        if (undoStack.size() > 0)
        {
            ui.undo->setEnabled(true);
        }
       //if edges have been removed, make redo button enabled
        if (redoStack.size() > 0)
        {
            ui.redo->setEnabled(true);
        }
       
        return;//exit
    }
      //otherwise, disable undo/redo buttons
        ui.undo->setEnabled(false);
        ui.redo->setEnabled(false);

    

}
     
void Graph::update_nodes(bool add = true) 
{   //update node coordinates based on options checked by user
    if (ui.circ_uniform_layout->isChecked())
    {
        setUniformCircCoords();
    }
    if (ui.random_layout->isChecked())
    {
        if (add) //adding vertex
        {
            node_coords.push_back({ WIDTH(eng), HEIGHT(eng) });
        }
        else //removing vertex
        {
            node_coords.pop_back();
        }
        
    }
    if (ui.circ_rand_layout->isChecked())
    {
        if (add)//adding vertex
        {
            double arg = theta(eng);
            double y = radius * std::sin(arg);
            double x = radius * std::cos(arg);
            node_coords.push_back({ x,y });
        }
        else //remove vertex
        {
            node_coords.pop_back();
        }
    }
    if (ui.lattice_layout->isChecked()) 
    {
        setLatticeCoords();
    }
    if (ui.custom_layout->isChecked() and !add) {
        //if user is making a custom graph they can only remove vertex by button
        node_coords.pop_back();
    }
}

//Drawing method
void Graph::draw_nodes() {
    //draw nodes 
    for (int i = 0; i < node_coords.size(); i++) 
    {
        double x = node_coords[i][0], y = node_coords[i][1];
        scene->addEllipse(x, y, node_w, node_h, defaultPen,defaultBrush);   
    }
}

void Graph::draw_node(int V, QPen pen, QBrush brush)
{
    double x = node_coords[V][0], y = node_coords[V][1];
    scene->addEllipse(x, y, node_w, node_h, pen, brush );
}

void Graph::draw_nodes_coloured(std::map<int, std::vector<double> > colours) {
    for (auto& vertex : colours) 
    {
        double x = node_coords[vertex.first][0];
        double y = node_coords[vertex.first][1];
        std::vector<double> colour = vertex.second;
        QPen pen(QColor::fromRgbF(colour[0], colour[1], colour[2]));
        QBrush brush(QColor::fromRgbF(colour[0], colour[1], colour[2]));
        scene->addEllipse(x, y, node_w, node_h, pen, brush);
    }
}

void Graph::draw_edge_line(int i, int j, QPen pen)
{
    
    double x1 = node_coords[i][0], y1 = node_coords[i][1];
    double x2 = node_coords[j][0], y2 = node_coords[j][1];
    scene->addLine(x1 + (node_w / 2), y1 + (node_w / 2), x2 + (node_w / 2), y2 + (node_w / 2), pen);
}

void Graph::draw_edges_hyp_poincaire(int i, int j, QPen pen) 
{  
    QPainterPath* path = new QPainterPath();
    //get node coordinates of edge endpoints
    double x1 = node_coords[i][0], y1 = node_coords[i][1];
    double x2 = node_coords[j][0], y2 = node_coords[j][1];

    //distance between endpoints
    double d = std::sqrt((pow(x2 - x1, 2) + pow(y2 - y1, 2)));

    //if sufficiently close to being opposite points on the circle
    if (abs(d - (2*radius)) < 0.01)
    {
          //hyperbolic geodesic is a straight line
          scene->addLine(x1 + (node_w / 2), y1 + (node_w / 2), x2 + (node_w / 2), y2 + (node_w / 2),pen);
          return;
    }
    //otherwise, find the point of intersection of tangent lines of node i, node j
    //this gives centre of circle whose arc forms edge between node i and node j.
    double mpx{}, mpy{}, beta1{}, beta2{}, m1,m2;
    //handle cases where y coordinates are close to 0
    if (abs(y1) < 0.01) 
    {
        mpx = x1;
        mpy = (radius * radius - x1 * x2) / y2;
        m2 = (mpy - y2) / (mpx - x2);
        beta2 = atan(m2);
        beta1 = pi / 2;
    }
    if (abs(y2) < 0.01) 
    {
        mpx = x2;
        mpy = (radius * radius - x2 * x1) / y1;
         m1 = (mpy - y1) / (mpx - x1);
         beta1 = atan(m1);
         beta2 = pi / 2;
    }

    //otherwise
    if (abs(y1) > 0.01 and abs(y2) > 0.01) 
    {
        mpx = -((radius * radius) * (y2 - y1)) / (y1 * (x2 - x1) - x1 * (y2 - y1));
        mpy = (-x1 / y1) * (mpx - x1) + y1;
        m1 = (mpy - y1) / (mpx - x1);
        m2 = (mpy - y2) / (mpx - x2);
        beta1 = atan(m1), beta2 = atan(m2);
    }
    
    //radius of circle whose arc forms edge
    double rad = std::sqrt(pow(mpx - x1, 2) + pow(mpy - y1, 2)), dist = 2 * rad;

    // beta1, beta2 are angle of tangent lines. 
    //arctan returns values in range(-pi/2, pi/2) but
    //there are 4 possible angles in range (-pi, pi) that give same value, 
    //test which is the correct angle:
    std::vector<double> B1 = { beta1, -beta1, pi + beta1, pi - beta1 };
    std::vector<double> B2 = { beta2, -beta2, pi + beta2, pi - beta2 };

    //test which angles corresponds to coordinates of node i and node j
    for (auto &arg : B1) 
    {
        double x = rad * cos(arg) + mpx;
        double y = rad * sin(arg) + mpy;
        if (abs(x - x1) < 0.01 and abs(y - y1) < 0.01) 
        {
            beta1 = arg;
            break;
        }
    }
    for (auto& arg : B2) 
    {
        double x = rad* cos(arg) +mpx;
        double y = rad * sin(arg) + mpy;
        if (abs(x - x2) < 0.01 and abs(y - y2) < 0.01) 
        {
            beta2 = arg;
            break;
        }
    }
    //convert angles from radians to degrees
    beta1 = beta1 * 180 / pi;
    beta2 = beta2 * 180 / pi;
  
    //sweep angle for which to draw arc
    double sweep = -beta2 + beta1;
    //make sweep angle in range (-180,180)
    if (sweep < -180) 
    {
        sweep = 360 + sweep;
    }
    if (sweep > 180) 
    {
        sweep = sweep - 360;
    }

    //draw arc
    double node_offset = node_w / double(2);
    path->arcMoveTo(mpx-rad+node_offset,mpy-rad+node_offset,dist,dist, -beta1);
    path->arcTo(mpx-rad+node_offset,mpy-rad+node_offset, dist,dist, -beta1,sweep);
    QGraphicsPathItem *pathItem = new QGraphicsPathItem(*path);
    pathItem->setPen(pen);
    scene->addItem(pathItem);
    
}

void Graph::draw_edges_hyp(int i, int j,QPen pen)
{
    QPainterPath* path = new QPainterPath();
    //get node coordinates of edge endpoints node i, node j
    double x1 = node_coords[i][0], y1 = node_coords[i][1];
    double x2 = node_coords[j][0], y2 = node_coords[j][1];

    //if x1, x2 sufficiently close
    if (abs(x1 - x2) < 0.01) 
    {
        //geodesic is a vertical line
        scene->addLine(x1 + (node_w / 2), y1 + (node_w / 2), x2 + (node_w / 2), y2 + (node_w / 2), pen);
    }
    //otherwise
    double mpx{}, mpy{}, beta1{}, beta2{}, m1, m2;
    //find centre of circle position on the bottom of the screen whose arc forms the edge
    //from node i to node j
    mpx = ((x2 * x2 - x1 * x1) + (y2 * y2 - y1 * y1) + (h) * (y1 - y2)) / (2 * (x2 - x1));
    mpy = h/2;

    //radius of circle
    double r = std::sqrt(pow(mpx - x1, 2) + pow(mpy - y1, 2));

    //handle case when centre of circle is directly under node
    if (abs(mpx - x1) < 0.01)
    {
        beta1 = pi / 2;
        m2 = (mpy - y2) / (mpx - x2);
        beta2 = atan(m2);
    }
    else if (abs(mpx - x2) < 0.01)
    {
        m1 = (mpy - y1) / (mpx - x1);
        beta2 = pi / 2;
        beta1 = atan(m1);
    }
    //otherwise get slopes of lines from node i, j to centre of circle
    else
    {
        m2 = (mpy - y2) / (mpx - x2);
        m1 = (mpy - y1) / (mpx - x1);
        beta1 =  atan(m1);
        beta2 = atan(m2);
    }
    //beta1, beta2 are the angles with which lines from node i,j to centre of circle
    //meet horizontal.
    //However arctan returns value in range (-pi/2, pi/2).
    //But 4 possible angles in range (-pi, pi) 
    //gives same value, test which is the correct angle:
    std::vector<double> B1 = { beta1, -beta1, pi + beta1, pi - beta1 };
    std::vector<double> B2 = { beta2, -beta2, pi + beta2, pi - beta2 };

    //check which angle gives back coordinates for nodes i,j
    for (auto& arg : B1)
    {
        double x = r * cos(arg) + mpx;
        double y = r* sin(arg) + mpy;
        if (abs(x - x1) < 0.01 and abs(y - y1) < 0.01)
        {
            beta1 = arg;
            break;
        }
    }
    for (auto& arg : B2)
    {
        double x = r * cos(arg) + mpx;
        double y = r * sin(arg) + mpy;
        if (abs(x - x2) < 0.01 and abs(y - y2) < 0.01)
        {
            beta2 = arg;
            break;
        }
    }
    //convert to from radians to degrees
    beta1 = beta1 * 180 / pi;
    beta2 = beta2 * 180 / pi;

    double arg1 = std::min(abs(beta1), abs(beta2));
    double arg2 = std::max(abs(beta1), abs(beta2));
    double sweep = -beta2 + beta1;
    //make sweep angle in range (-180,180)
    if (sweep < -180)
    {
        sweep = 360 + sweep;
    }
    if (sweep > 180)
    {
        sweep = sweep - 360;
    }
    //draw arc
    double node_offset = node_w / double(2);
    path->arcMoveTo(mpx-r+node_offset, mpy-r + node_offset, 2*r, 2*r, -beta1);
    path->arcTo(mpx-r+node_offset  ,mpy-r + node_offset, 2*r, 2*r, -beta1, sweep);
    QGraphicsPathItem* pathItem = new QGraphicsPathItem(*path);
    pathItem->setPen(pen);
    scene->addItem(pathItem);

}

void Graph::redraw_edges(QPen pen)
{//redraw edge based on geometry settings
    if (ui.euclidean_geometry->isChecked())
    {
for (auto& edge : selected_edgelist)
{
    draw_edge(edge[0], edge[1],pen);
}
    }
    if (ui.hyperbolic_geometry->isChecked())
    {   
        if (ui.circ_rand_layout->isChecked() or ui.circ_uniform_layout->isChecked())
        {
            for (auto& edge : selected_edgelist)
            {
                draw_edges_hyp_poincaire(edge[0], edge[1],pen);
            }
        }
        else
        {
            for (auto& edge : selected_edgelist)
            {
                draw_edges_hyp(edge[0], edge[1],pen);
            }
        }
    }
}

void Graph::draw_edge(int i, int j, QPen pen)
{
    /*Draws edge from vertex i to vertex j, depending on
    geometry*/
    if (ui.euclidean_geometry->isChecked())
    {
        draw_edge_line(i, j,pen);
    }
    if (ui.hyperbolic_geometry->isChecked())
    {
        if (ui.circ_rand_layout->isChecked() or ui.circ_uniform_layout->isChecked())
        {
            draw_edges_hyp_poincaire(i, j,pen);
        }
        else
        {
            draw_edges_hyp(i, j,pen);
        }
    }
}

void Graph::Traverse(int u, std::map<int, bool>& visited, std::map<int, 
    std::vector<int>>& new_adjs, int& edges, std::map<int, bool>& visited2,
    QPen &pen, QBrush &brush)
{
    visited[u] = true; //mark v as visited
    visited2[u] = true; //mark v as visited on updated list carried throughout computation
    //add u to new adjacency list
    new_adjs.insert({ u,{} });
    //for nodes in adjaceny list of G
    for (auto& v : G.adj[u])
    {  
        new_adjs[u].push_back(v);
        //add v to new adjacency list of u
        edges++;
        //increase edge count
        //if v not already visited
        if (!visited[v])
        {
            //draw nodes and edge between them
            draw_node(u, pen, brush);
            draw_edge(u, v,pen);
            draw_node(v, pen, brush);
            //traverse from v
            Traverse(v, visited, new_adjs, edges, visited2, pen, brush);
        }
    }
}

void Graph::DFS()
{  /*highlights traversel of graph using DFS algorithm*/
    
    std::vector<graph> components = {};
    std::map<int, bool> vis, VIS;

    //no nodes visited yet, set VIS to false
    for (auto& i : G.adj) 
    { VIS[i.first] = false; 
    }
    //for each node u
    for (auto& u : G.adj) 
    {
        if (VIS[u.first]) 
        { 
            continue; //if node has already been visited, skip iteration
        }

        for (auto& i : G.adj) 
        { 
            vis[i.first] = false; //starting from u, initialize list with no node is visited
        } 
        //edge count and new adjacency list for component
        int edges = 0;
        std::map<int, std::vector<int> > new_adjs;

        //get colur to traverse component in
        int index = rand() % Graph::colourPalette.size();
        double r = colourPalette[index][0], g = colourPalette[index][1],
            b = colourPalette[index][2];
        QPen pen = QPen(QColor::fromRgbF(r, g, b),2);
        QBrush brush = QBrush(QColor::fromRgbF(r, g, b));

        //traverse graph from u
        Traverse(u.first, vis, new_adjs, edges, VIS, pen, brush);

        //add component to list 
        components.push_back(graph(new_adjs, edges / 2));
        
        //if component is a singular node, colour it
        if (new_adjs.size() == 1)
        {
            int node = new_adjs.begin()->first;
            draw_node(node, pen, brush);
        }

    }
    
}

void Graph::draw_connected_components()
{   /*Highlights connected components of graph*/

    //get connected components
    std::vector<graph> components = G.connectedComponents();
    scene->clear();
     
    //for each component
    for (auto& component : components)
    {   
        //get colour to draw in 
        int index = rand() % Graph::colourPalette.size();
        double r = colourPalette[index][0], g = colourPalette[index][1],
        b = colourPalette[index][2];
        QPen pen = QPen(QColor::fromRgbF(r, g, b));
        QBrush brush = QBrush(QColor::fromRgbF(r, g, b));
        
        //for each node
        for (auto& node : component.adj)
        {
            //draw node
            draw_node(node.first, pen, brush);
            //for each neighbour
            for (auto& neighbour : node.second)
            {
                //draw neighbour and edge between them
                draw_node(neighbour, pen, brush);
                draw_edge(neighbour, node.first,pen);
            }
        }
    }
}


//set/get/helper methods
void Graph::set_node_coords()
{
    /*Given a graph with n nodes, set the
    vertex coordinates depending on layout*/
    if (ui.circ_uniform_layout->isChecked())
    {
        setUniformCircCoords();
    }
    if (ui.random_layout->isChecked())
    {
        setRandomCoords();
    }
    if (ui.circ_rand_layout->isChecked())
    {
        setRandomCircCoords();
    }
    if (ui.lattice_layout->isChecked())
    {
        setLatticeCoords();
    }
}

std::vector<std::vector <int> > Graph::getAllEdges(int n)
{
    //given n nodes, there are nC2= n(n-1)/2 possible edges.
    //iterate over i, and then j>=i+1 to get ordered list of
    //edges (i,j) with i<j.
    std::vector<std::vector<int> > edges = {};
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            std::vector<int> edge = { i,j };
            edges.push_back(edge);
        }
    }
    return edges;
}

std::map<int, std::vector<double> > Graph::getColours(int coloursNeeded, std::map<int, int> colouring)
{
    //determine sampling step size from colour pallete given coloursNeeded.
    double avg_step = colourPalette.size() / double(coloursNeeded);
    int stepsize = (int)colourPalette.size() / coloursNeeded;
    //larger step size
    int larger = stepsize + 1;

    //conatiner for rgb colours
    std::vector < std::vector < double > > colourlist;
    int i = 0;
    int index = 0;
    double moving_avg_step = 0;

    //pick out colours by traversing colour pallette and trying to take the 
    //largest step size possible at that moment by checking moving average
    while (i < coloursNeeded)
    {
        //add colour
        colourlist.push_back(colourPalette[index]);

        //determin step size to be taken
        double next_avg = (i * avg_step + larger) / (double(i + 1));
        if (next_avg < avg_step and index + larger < colourPalette.size())
        {
            moving_avg_step = next_avg;
            index = index + larger;
        }
        else
        {
            index = index + stepsize;
            moving_avg_step = (i * avg_step + stepsize) / double(i + 1);
        }
        i++;
    }

    //map colours to each node given colouring
    std::map<int, std::vector<double> > colours;
    for (auto& colour : colouring)
    {
        colours[colour.first] = colourlist[colour.second];
    }
    return colours;
}

void Graph::findNextToBeAdded()
{   
    //if edge has already been added
    if (G.isNeighbour(edgelist[selected_edge][0], edgelist[selected_edge][1]))
    {       
            //while edges can be added and edge has already been added
            while (selected_edge < edgelist.size()
                and G.isNeighbour(edgelist[selected_edge][0], edgelist[selected_edge][1]))
            {
                //move to next edge
                selected_edge++;
            }
    }

    return;
}

void Graph::makeRandomGraph(bool sparse = false)
{
    on_clear_clicked();
    //set random number of nodes
    nodes = rand() % max + 1;
    //set node_coords
    set_node_coords();

    //construct graph with nodes
    G = graph(nodes);
    edgelist = getAllEdges(nodes);
    if (edgelist.size() > 0)
    {
        //for each possible edge
        for (auto& edge : edgelist)
        {   
            //if not sparse
            if (!sparse)
            {   
                //add 1/3 edges
                if (rand() % 3 == 1)
                {
                    draw_edge(edge[0], edge[1]);
                    selected_edgelist.push_back(edge);
                    G.addEdge(edge[0], edge[1]);
                }
            }
            else
            { //if spares add fewer edges, around every 1/nodes^(3/4)
                if (rand() % ((int)(nodes/std::sqrt(nodes/std::sqrt(nodes)))) == 1)
                {
                    draw_edge(edge[0], edge[1]);
                    selected_edgelist.push_back(edge);
                    G.addEdge(edge[0], edge[1]);
                }
            }
        }
        selected_edge = edgelist.size();
        //draw nodes
        draw_nodes();
        //update information displayed to user
        update_edge_displayed();
    }
    return;
}

void Graph::setUniformCircCoords()
{
    //set unifomrly spaced coordinates about the circle of radius radius
    node_coords = {};
    for (int i = 0; i < nodes; i++)
    {
        double y = radius * std::sin(2 * pi * i / (double(nodes)));
        double x = radius * std::cos(2 * pi * i / double(nodes));
        node_coords.push_back({ x, y });
    }
}

void Graph::setRandomCircCoords()
{
    node_coords = {};
    for (int i = 0; i < nodes; i++)
    {
        double arg = theta(eng);
        double y = radius * std::sin(arg);
        double x = radius * std::cos(arg);
        node_coords.push_back({ x, y });
    }
}

void Graph::setLatticeCoords()
{
    node_coords = {};
    double realgridsize = std::sqrt(nodes);
    int gridsize = (int)realgridsize;
    if (gridsize < realgridsize)
    {
        gridsize++;
    }
    double stepsize = lattice_size / double(gridsize);
    scene->clear();
    double x0 = -lattice_size / double(2);
    double* y0 = &x0;
    for (int i = 0; i < nodes; i++)
    {
        int a = (int)i / gridsize;
        int b = i % gridsize;
        double x = x0 + (b * stepsize);
        double y = *y0 + (a * stepsize);
        node_coords.push_back({ x, y });

    }
}

void Graph::setRandomCoords()
{
    node_coords = {};
    for (int i = 0; i < nodes; i++)
    {
        node_coords.push_back({ WIDTH(eng), HEIGHT(eng) });
    }
}





//Graph construction functions for buttons
void Graph::on_add_node_clicked()
{
    
    if (nodes < max) 
    {
        //add node to graph
        G.addNode(nodes);
        //increase node count
        nodes++;
        //refresh edge list and scene
        edgelist = getAllEdges(nodes);
        scene->clear();
        update_nodes();
        redraw_edges();
        draw_nodes();
        selected_edge = 0;
        update_edge_displayed();
    }
   

}

void Graph::on_remove_node_clicked()
{   
    //clear undo/redo stacks, keeping track of edges to remove after decreasing
    //the number of vertices becomes messy
    undoStack = {};
    redoStack = {};

    if (nodes > 0) 
    {
        nodes--;
        //if no nodes left, have null graph, clear.
        if (nodes == 0) 
        {
            on_clear_clicked();
            return;
        }
        scene->clear();
        //remove node from graph
        G.removeNode(nodes);
        
        std::vector<std::vector<int>> new_selected_edgelist = {};
        //pick out edges which do not visit removed node
        for (int i = 0; i < selected_edgelist.size(); i++)
        {
            if (selected_edgelist[i][0] != nodes and selected_edgelist[i][1] != nodes)
            {
                new_selected_edgelist.push_back(selected_edgelist[i]);
               
            } 
        }
        selected_edgelist = new_selected_edgelist;
        //refresh
        edgelist = getAllEdges(nodes);
        update_nodes(false);
        redraw_edges();
        draw_nodes();
        update_edge_displayed();
    }
  
   // updateInfo();

}

void Graph::on_enter_node_textEdited(const QString& arg1)
{    
    if (ui.enter_node->text().isEmpty()) 
    { 
        ui.enter_node->setPlaceholderText("0");
        on_clear_clicked();
        return;
    }
    //check input
    if (std::regex_match(ui.enter_node->text().toStdString(), std::regex("[0-9]*")))
    {
        int input = arg1.toInt();
        if (input < max and input >= 0)
        {
            nodes = input;
            edgelist = getAllEdges(nodes);
            scene->clear();
            //construct new graph
            G = graph(nodes);
            //refresh
            set_node_coords();
            draw_nodes();
            update_edge_displayed();
            return;
        }
    }
    on_clear_clicked();
    errorMessage();
}

void Graph::on_add_edge_clicked()
{/*adds edge to scene and graph*/
    if (edgelist.size() == 0) 
    {
        return;
    }
    else 
    {

        if (selected_edge < edgelist.size()) 
        {
            std::vector<int> edge = edgelist[selected_edge];
            //draw edges and add edge to graph
            draw_edge(edge[0], edge[1]);
            G.addEdge(edge[0], edge[1]);
            selected_edgelist.push_back(edge);
            //unhighlight nodes
            draw_node(edge[0]);
            draw_node(edge[1]);
            //move to next edge
            selected_edge++;
            //send command to undo stack
            undoStack.push_back(1);
            update_edge_displayed();
            
        }
    }
}

void Graph::on_reject_edge_clicked()
{/*skips adding edge. */
    if (edgelist.size() == 0) 
    {
        return;
    }
    if (selected_edge < edgelist.size()) 
        {
        //add to undo stack
            undoStack.push_back(2); 
            //unhighlight nodes
            draw_node(edgelist[selected_edge][0]);
            draw_node(edgelist[selected_edge][1]);
            //move to next edge
            selected_edge++;
            update_edge_displayed();
            
        }
    
}

void Graph::on_auto_fill_clicked()
{
    if (edgelist.size() > 0)
    {
        //for all remainning possible edges
        for (int i = selected_edge; i < edgelist.size(); i++)
        {
            //draw and add edge
            std::vector<int> edge = edgelist[i];
            draw_edge(edge[0], edge[1]);
            selected_edgelist.push_back(edge);
            G.addEdge(edge[0], edge[1]); 
        }
        selected_edge = edgelist.size();
        //refresh
        draw_nodes();
        update_edge_displayed();
    }
}

void Graph::on_random_clicked()
{
    makeRandomGraph();
}

void Graph::on_sparse_clicked()
{
    makeRandomGraph(true);
}

void Graph::on_clear_clicked()
{
    //clear everything
    G = graph();
    nodes = 0;
    edgelist = {};
    selected_edgelist = {};
    selected_edge = 0;
    scene->clear();
    ui.enter_node->clear();
    ui.start_node->clear();
    ui.end_node->clear();
    ui.chrom_poly->clear();
    ui.chromnum->clear();
    ui.possible_edge_count->clear();
    ui.enter_node->setPlaceholderText("0");
    ui.edge_count->clear();
    node_coords = {};
    undoStack = {};
    redoStack = {};
    user_selected_node = {};
    ui.undo->setEnabled(false);
    ui.redo->setEnabled(false);
    ui.add_edge->setEnabled(false);
    ui.reject_edge->setEnabled(false);
    ui.start_node->setStyleSheet("QLabel { background-color : rgb(82,82,82) }");
    ui.end_node->setStyleSheet("QLabel { background-color : rgb(82,82,82) }");
}



//Graph geometry functions
//set coordinates then refresh graph scene
void Graph::on_random_layout_clicked() {
    /*when random layout is selected by user, set coordinates and refresh
    randomly*/
    fixedDesign();
    scene->clear();
    setRandomCoords();
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}

void Graph::on_circ_uniform_layout_clicked() {
    fixedDesign();
    scene->clear();
    setUniformCircCoords();
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}

void::Graph::on_circ_rand_layout_clicked() {
    fixedDesign();
    scene->clear();
    setRandomCircCoords();
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}

void Graph::on_lattice_layout_clicked()
{
    fixedDesign();
    setLatticeCoords();
    scene->clear();
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}

void Graph::on_custom_layout_clicked()
{
    ui.random->setEnabled(false); 
    ui.add_node->setEnabled(false);
    ui.enter_node->setEnabled(false);
    on_clear_clicked();
    update_edge_displayed();
}

void Graph::fixedDesign()
{
    ui.random->setEnabled(true);
    ui.add_node->setEnabled(true);
    ui.enter_node->setEnabled(true);
}


void Graph::on_euclidean_geometry_clicked()
{
    scene->clear();
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}

void Graph::on_hyperbolic_geometry_clicked()
{
    scene->clear();
    
    redraw_edges();
    draw_nodes();
    update_edge_displayed();
}



//Graph colouring tools
void Graph::on_calc_clicked() 
{/*calculate chromatic polynomial for small graphs and find chromatic number*/
    if (nodes > 0 and nodes<=10)
    {
        Polynomial P = G.getChromaticPolynomial();
        std::string p = P.toString();
        p.pop_back();
        p.pop_back();
        p.pop_back();
        //display chromatic polynomial and chromatic number
        ui.chrom_poly->setText(QString::fromStdString(p)); 
        int k = G.chromaticNumber(P);
        ui.chromnum->setText(QString::fromStdString("X(G) = " + std::to_string(k)));
    }

    
    return;
}

void Graph::on_greedy_clicked() 
{/*Greedy colouring of graph*/
    if (nodes > 0)
    {
        int colours_needed = 0;
        std::map<int, int> colouring = G.greedyColouring(colours_needed);
        std::map<int, std::vector<double> > colours = getColours(colours_needed, colouring);
        draw_nodes_coloured(colours);
        ui.chromnum->setText(QString::fromStdString("X(G)<= " + std::to_string(colours_needed)));
    }
    else
    {
        return;
    }
}

void Graph::on_connected_clicked()
{
    draw_connected_components();
}

void Graph::on_DFS_clicked()
{
    DFS();
}


//Mouse event handler for graphics window
bool Graph::inScene(QPointF &P)
{
    /*Checks if position P is in graphicsView scene*/
    double x = P.x(), y = P.y();
    if (x < w and y <  h)
    {
        return true;
    }
    return false;
}

bool Graph::onNode(QPointF &P, int& nodeIndex)
{   
    /* checks if point P is sufficiently close to node by checking coordinates*/
    double x =  P.x() -w/2-node_w-node_w/4;
    double y =  P.y()-h/2 -2*node_h;
    for (int i = 0; i < nodes; i++)
    {
        if (std::sqrt(pow(x - node_coords[i][0], 2) + pow(y - node_coords[i][1], 2)) < node_w)
        {
            nodeIndex = i;
            return true;
        }
    }
    return false;
}

void Graph::mousePressEvent(QMouseEvent* event)
{
    //get position of mouse click event
    QPointF P = event->position();
    int node_P_index;
   
    //if user making a custom graph
    if (ui.custom_layout->isChecked())
    {   
        //if in scene and click is not on a node, draw new node 
        // at position of click and add to graph
        if (inScene(P) and !onNode(P, node_P_index))
        {
            double x = -w/2 + P.x()-node_w-node_w/4;
            double y = -h/2 + P.y()-2*node_w;
            node_coords.push_back({ x,y });
            scene->addEllipse(x, y, node_w, node_w, defaultPen, defaultBrush);
            G.addNode(nodes);
            nodes++;
            edgelist = getAllEdges(nodes);
            update_edge_displayed();
            if (user_selected_node.size() > 0)
            {
                draw_node(user_selected_node[0]);
                user_selected_node.pop_back();
            }
            return;
        }
    }

    //if click is on a node
    if (onNode(P, node_P_index))
    {   
        //if user has clicked on another node
        if (user_selected_node.size() > 0)
        {
            //draw edge beteween nodes
            int i = std::min(user_selected_node[0], node_P_index);
            int j = std::max(user_selected_node[0], node_P_index);
            draw_node(i);
            draw_node(j);
            //check if edge has not been already added, or not same node
            if (!(G.isNeighbour(i, j)) and !(i==j))
            {
                //then draw edge and add to graph
                
                G.addEdge(i, j);
                draw_edge(i, j);
          
                if (selected_edge < edgelist.size() and edgelist[selected_edge][0] == i and edgelist[selected_edge][1] == j) {
                    selected_edge++;
                }
                selected_edgelist.push_back({ i,j });
                undoStack.push_back(1);
                update_edge_displayed();
                
            }
            //clear user slected node container
            user_selected_node = {};
        }
        else
        //if user hasnt already clicked on a node, add to user_selected_node container
        {
            //check if all options have been covered
            if (selected_edge<edgelist.size())
            {
                user_selected_node.push_back(node_P_index);
                draw_node(node_P_index, highlight1_pen, highlight1_brush);
                std::vector<int> suggested_edge = edgelist[selected_edge];
                //change highlighted node colours
                if (node_P_index == suggested_edge[0])
                {
                    draw_node(suggested_edge[1]);
                }
                if (node_P_index == suggested_edge[1])
                {
                    draw_node(suggested_edge[0]);
                }
                if (node_P_index != suggested_edge[1] and node_P_index != suggested_edge[0])
                {
                    draw_node(suggested_edge[1]);
                    draw_node(suggested_edge[0]);
                }
                return;
            }

        }
    }

    return;
}


//Undo/Redo for edge drawing
void Graph::on_undo_clicked()
{  //if nothing to undo, disbale
    if (undoStack.size() == 0)
    {
        ui.undo->setEnabled(false);
        return;
    }
    else
    {
        //get lost command
        int last_command = undoStack.back();
        undoStack.pop_back();
        //remove from stack and add to redo stack
        redoStack.push_back(last_command);
        
        switch (last_command)
        {
         //case 1, command was add edge, undo by removing edge
        case 1:
        { 
 
            if (selected_edge < edgelist.size()+1 and selected_edge >= 1)
            {  
            //get last edge added
            std::vector<int> edge = selected_edgelist.back();
            G.removeEdge(edge[0], edge[1]);
            selected_edge--;
            selected_edgelist.pop_back();
            //refresh
            scene->clear();
            redraw_edges();
            draw_nodes();
            update_edge_displayed();
            }
            break;
        }
        //case 2 - edge was rejected. to undo just walk back one step and allow
        //user to select again
        case 2:
        {
            selected_edge--;
            draw_nodes();
            update_edge_displayed();
            break;
        }
        }
        //enable redo
        ui.redo->setEnabled(true);
        //if nothing left to undo, disable
        if (undoStack.size() == 0)
        {
            ui.undo->setEnabled(false);
        }
    }
}

void Graph::on_redo_clicked() 
{
    //if nothing to redo, disable
    if (redoStack.size() == 0)
    {
        ui.redo->setEnabled(false);
        return;
    }
    else
    {
        //get last command that was undone
        int last_command = redoStack.back();
        //remove from stack
        redoStack.pop_back();
        
        
        switch (last_command)
        {
         //if adding an edge was undone, add back edge, but without pushing command to undo stack
        case 1:
        {
            on_add_edge_clicked();

            break;
        }
        
        case 2:
        {
            on_reject_edge_clicked();
            break;
        }
        }
        //enable undo
        ui.undo->setEnabled(true);
        //if nothng to redo, disable
        if (redoStack.size() == 0)
        {
            ui.redo->setEnabled(false);
        }
    }
}







   