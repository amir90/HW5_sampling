#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

using namespace std;

const double INF = numeric_limits<double>::max();

struct qPoint {
	Point_2 xy1;
	Point_2 xy2;
	int index;
	double vec[4];
};

double heuristic(Point_2 p1, Point_2 p2) {
	return 0;
}

qPoint newRandomQPoint (double xmin,double xmax,double ymin,double ymax) {
	//TODO: implement
	return new qPoint;
}

bool isLegalConfiguration(qPoint p,Arrangment_2 arr) {
	//TODO: implement
	return true;
}

double dist(qPoint p1, qPoint p2) {
	//TODO: implement
	return 0;
}

struct Distance {
  typedef qPoint Query_item;
  typedef double FT;
  typedef CGAL::Dimension_tag<4> D;
  double transformed_distance(const qPoint& p1, const qPoint& p2) const {
    return dist_1(p1,p2);
  }
  double min_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b) const {
    double distance(0.0), h = p.xy1.x().to_double();
    if (h < b.min_coord(0)) distance += (b.min_coord(0)-h)*(b.min_coord(0)-h);
    if (h > b.max_coord(0)) distance += (h-b.max_coord(0))*(h-b.max_coord(0));
    h=p.xy1.y().to_double();
    if (h < b.min_coord(1)) distance += (b.min_coord(1)-h)*(b.min_coord(1)-h);
    if (h > b.max_coord(1)) distance += (h-b.max_coord(1))*(h-b.min_coord(1));
    h=p.xy2.x();
    if (h < b.min_coord(2)) distance += (b.min_coord(2)-h)*(b.min_coord(2)-h);
    if (h > b.max_coord(2)) distance += (h-b.max_coord(2))*(h-b.max_coord(2));
    return distance;
    h=p.xy2.y();
    if (h < b.min_coord(3)) distance += (b.min_coord(3)-h)*(b.min_coord(3)-h);
    if (h > b.max_coord(3)) distance += (h-b.max_coord(3))*(h-b.max_coord(3));
    return distance;
  }
  double min_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b,std::vector<double>& dists){
    double distance(0.0), h = p.xy1.x().to_double();
    if (h < b.min_coord(0)){
      dists[0] = (b.min_coord(0)-h);
      distance += dists[0]*dists[0];
    }
    if (h > b.max_coord(0)){
      dists[0] = (h-b.max_coord(0));
      distance += dists[0]*dists[0];
    }
    h=p.xy1.y().to_double();
    if (h < b.min_coord(1)){
      dists[1] = (b.min_coord(1)-h);
      distance += dists[1]*dists[1];
    }
    if (h > b.max_coord(1)){
      dists[1] = (h-b.max_coord(1));
      distance += dists[1]*dists[1];
    }
    h=p.xy2.x().to_double();
    if (h < b.min_coord(2)){
      dists[2] = (b.min_coord(2)-h);
      distance += dists[2]*dists[2];
    }
    if (h > b.max_coord(2)){
      dists[2] = (h-b.max_coord(2));
      distance += dists[2]*dists[2];
    }
    h=p.xy2.x().to_double();
    if (h < b.min_coord(3)){
      dists[3] = (b.min_coord(3)-h);
      distance += dists[3]*dists[3];
    }
    if (h > b.max_coord(3)){
      dists[3] = (h-b.max_coord(3));
      distance += dists[3]*dists[3];
    }
    return distance;
  }
  double max_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b) const {
    double h = p.xy1.x().to_double();
    double d0 = (h >= (b.min_coord(0)+b.max_coord(0))/2.0) ?
                (h-b.min_coord(0))*(h-b.min_coord(0)) : (b.max_coord(0)-h)*(b.max_coord(0)-h);
    h=p.xy1.y().to_double();
    double d1 = (h >= (b.min_coord(1)+b.max_coord(1))/2.0) ?
                (h-b.min_coord(1))*(h-b.min_coord(1)) : (b.max_coord(1)-h)*(b.max_coord(1)-h);
    h=p.xy2.x().to_double();
    double d2 = (h >= (b.min_coord(2)+b.max_coord(2))/2.0) ?
                (h-b.min_coord(2))*(h-b.min_coord(2)) : (b.max_coord(2)-h)*(b.max_coord(2)-h);
    h=p.xy2.y().to_double();
    double d3 = (h >= (b.min_coord(3)+b.max_coord(3))/2.0) ?
                (h-b.min_coord(3))*(h-b.min_coord(3)) : (b.max_coord(3)-h)*(b.max_coord(3)-h);
    return d0 + d1 + d2+d3;
  }
  double max_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b,std::vector<double>& dists){
	double h = p.xy1.x().to_double();
    dists[0] = (h >= (b.min_coord(0)+b.max_coord(0))/2.0) ?
                (h-b.min_coord(0)) : (b.max_coord(0)-h);

    h=p.xy1.y().to_double();
    dists[1] = (h >= (b.min_coord(1)+b.max_coord(1))/2.0) ?
                (h-b.min_coord(1)) : (b.max_coord(1)-h);
    h=p.xy2.x().to_double();
    dists[2] = (h >= (b.min_coord(2)+b.max_coord(2))/2.0) ?
                (h-b.min_coord(2)) : (b.max_coord(2)-h);
    h=p.xy2.y().to_double();
    dists[3] = (h >= (b.min_coord(3)+b.max_coord(3))/2.0) ?
                (h-b.min_coord(3)) : (b.max_coord(3)-h);
    return dists[0] * dists[0] + dists[1] * dists[1] + dists[2] * dists[2] + dist[3] * dist[3];
  }
  double new_distance(double& dist, double old_off, double new_off,
              int /* cutting_dimension */)  const {
    return dist + new_off*new_off - old_off*old_off;
  }
  double transformed_distance(double d) const { return d*d; }
  double inverse_of_transformed_distance(double d) { return std::sqrt(d); }
}; // end of struct Distance

Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    CGAL::Orientation orient = ret.orientation();
    if (CGAL::COUNTERCLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}

pair<double, double> to_double(Point_2 p) {
  return pair<double, double>(p.x().to_double(), p.y().to_double());
}

string point2string(Point_2 p) {
	pair<double, double> pDouble = to_double(p);
	stringstream sstm;
	sstm << "(" << pDouble.first << " , " << pDouble.second << ")";
	return sstm.str();
}

void print_point(Point_2  p) {
  cout <<  point2string(p);
}

bool has(set<int> s, int x) {
	return s.find(x) != s.end();
}
vector<pair<Point_2, Point_2>>
findPath(const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2,
         const Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {

	int obstacles_size = obstacles.size();

	// create graph vertices set
	set<Point_2> vertices;
	vertices.insert(start1);
	vertices.insert(end1);
	vertices.insert(start2);
	vertices.insert(end2);

	// add all obstacles to polygon set
	Arrangement_2 free_space_arrangement;
	for(int i = 0; i < obstacles_size; i++) {
		Polygon_2 obstacle = obstacles.at(i);
		CGAL::insert(free_space_arrangement,obstacle.edges_begin(),obstacle.edges_end());
	}

	//identify obstacle faces
	for (auto i=free_space_arrangement.faces_begin(); i!=free_space_arrangement.faces_end(); i++) {
		if (!i->is_unbounded()) {
		i->set_data(true);
		} else {
		i->set_data(false);
		}
	}

	// create an arrangement from the polygon set
	//add outer polygon
	for (auto i=outer_obstacle.edges_begin(); i!=outer_obstacle.edges_end(); i++) {
		Segment_2 addSeg(i->point(0),i->point(1));
		CGAL::insert(free_space_arrangement,addSeg);
	}

	//N random configurations
	int currRandPoints=0;
	int counter =0;
	while (counter < TIMEOUT && currRandPoints<Nrand ) {
		qPoint temp = newRandomQPoint(xmin,xmax,ymin,ymax);
			if(isLegalConfiguration(temp, free_space_arrangement)) {
				temp.index = currInd;
				V.push_back(temp);
				currInd++;
				currRandPoints++;
				counter=0;
			}
		counter++;
	}


	int N=V.size();

	//Range implementation;

	Tree tree;

	tree.insert(V.begin(),V.end());

	double radius = 3/2*pow((log2(N)/N),(1/3));//*(bbox.xmax()-bbox.xmin());//((bbox.xmax()-bbox.xmin())*(bbox.ymax()-bbox.ymin()));

	std::vector<std::list<qPoint>> neighbors(N);

	for (qPoint q: V ) { //sorted by index

//		Fuzzy_Circle fc(q,radius);

//		tree.search(std::back_inserter(neighbors[q.index]), fc);

		K_neighbor_search search(tree, q, K);
/*
		for (auto i=search.begin(); i!=search.end(); i++) {
		neighbors[q.index].push_back((*i).first);



		}*/

		for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
		    	qPoint q1 = it->first;
		    	neighbors[q.index].push_back(q1);
		    	neighbors[q1.index].push_back(q);

	}
	}


	std::cout<<"done finding nodes"<<endl;


	std::vector<double> g(N,numeric_limits<double>::max());
	std::vector<double> f(N,numeric_limits<double>::max());
	std::vector<int> parent(N,-1);
	std::vector<int> Orient(N,2);

		f[0] = heuristic(V[0],V[1]);
		g[0]=0;

	bool foundPath = false;

	std::set<int> Open; std::set<int> Closed;

	Open.insert(0);

	 while (!Open.empty()) {

		 int min_f_ind = *(Open.begin());
		 for (auto i=Open.begin(); i!=Open.end(); i++) {
			 if (f[*i]<f[min_f_ind]) {
				 min_f_ind = *i;
			 }
		 }
		qPoint v = V[min_f_ind];
		if (v.index == 1) {foundPath=true; break;}
		Closed.insert(min_f_ind);
		Open.erase(min_f_ind);
		for (qPoint q: neighbors[v.index]) {


			if (Closed.find(q.index)!=Closed.end()) { //node already expanded
				continue;
			}
			auto temp = cost(v,q,queryHandler);



			if (temp.second==2) {continue;}
			Open.insert(q.index);

			if (g[q.index]<=(g[v.index] + temp.first)) { //this is not a better path
				continue;
			}
			parent[q.index] = v.index; g[q.index]=g[v.index]+temp.first;
			f[q.index] = g[q.index]+heuristic(q,V[1]);
			Orient[q.index] = temp.second; //direction of rotation;


		}
	}

	 std::cout<<"exited loop"<<endl;

	std::vector<int> path;

	//start and end vertices are only connected

	//find trapezoids in optimal path for red robot, using a* with euclidean distance heurisitc (robot is point in configuration space)
	//Open is set of indexes
	//Closed is set of indexes


/*
	// copy vertices to vector
	vector<Point_2> vertices_vectors;
	std::copy(vertices.begin(), vertices.end(), std::back_inserter(vertices_vectors));

	cout << " Vertices are : " << endl;
	// create point to index map
	map<Point_2, int> vertex2index;
	for(int i=0; i<vertices_vectors.size(); i++){
		cout << i << " " << point2string(vertices_vectors.at(i)) << endl;
		vertex2index.insert(pair<Point_2,int>(vertices_vectors.at(i), i));
	}

	// initialize graph
	int numV = vertices.size();
	double** graph = (double**) calloc(numV, sizeof(double *));
	for(int i=0; i<numV; i++) {
		graph[i] = (double*) calloc(numV, sizeof(double));
		for(int j=0; j<numV; j++)
			graph[i][j] = (i == j ? 0 : INF);
	}

	list<Point_2> startEndPositions;
	startEndPositions.push_back(start1);
	startEndPositions.push_back(start2);
	startEndPositions.push_back(end1);
	startEndPositions.push_back(end2);

	// go over throgh all faces, check if free and add appropriate edges
	for(Arrangement_2::Face_handle face = free_space_arrangement.faces_begin(); face != free_space_arrangement.faces_end(); ++face) {
		bool is_obstacle = true;
		if(face->has_outer_ccb()) {
			Arrangement_2::Ccb_halfedge_circulator beginning = face->outer_ccb();
			vector<set<int>> orientations;

			// init orientations
			for(int i=0; i<startEndPositions.size(); i++) {
					orientations.push_back(set<int>());
				}

			auto circular = beginning;
			vector<Point_2> face_vertices;

			do {
				// how to convert circular and get halfedge ??
				Point_2 source = circular->source()->point(),
						target = circular->target()->point();

				// remember orientation to start/end positions
				int i = 0;
				for(auto s = startEndPositions.begin();
						s != startEndPositions.end(); s++, i++) {
					set<int> orientation_set = orientations.at(i);
					CGAL::Orientation o = CGAL::orientation(source, target, *s);
					if(o == CGAL::LEFT_TURN)
						orientation_set.insert(-1);
					else if(o == CGAL::RIGHT_TURN)
						orientation_set.insert(1);
					else orientation_set.insert(0);

				}

//				cout << "half-edge : [";
//				print_point(source);
//				cout << " , ";
//				print_point(target);
//				cout << " ]" << endl;



				face_vertices.push_back(source);

				// check if face includes wall. if it doesn't => obstacle
				for (auto wall=segList.begin(); wall!=segList.end(); wall++) {
					if((wall->source() == source && wall->target() == target) ||
							(wall->target() == source && wall->source() == target)) {
						is_obstacle = false;
						break;
					}

				}

			}
			while(++circular != beginning);

//			vector<Point_2 *> toErase;
			int i=0;
			for(auto s = startEndPositions.begin();
					s != startEndPositions.end(); s++, i++) {
				set<int> orientation_set = orientations.at(i);
				if( !has(orientation_set, -1) || !has(orientation_set, 1)) {
					// start/end position in this face(or on boundry)
					face_vertices.push_back(*s);
					if(!has(orientation_set,0)) {
						// we can be sure that it is not on boundry
						// we don't have to look for this point anymore
//						toErase.push_back(s);
						s = startEndPositions.erase(s);
					}
				}
			}

//			for(Point_2 **x = toErase.begin(); x!=toErase.end(); x++) {
//				startEndPositions.erase(*x);
//			}
			// connect edges
			if(face->has_outer_ccb() && !is_obstacle) {
				for(int i=0; i<face_vertices.size() - 1; i++) {
					Point_2 p1 = face_vertices.at(i);

					//if not in vertices it is probably an outer-bound vertex
					if ( vertex2index.find(p1) != vertex2index.end()) {
						int index1 = vertex2index.at(p1);
						for(int j=i+1; j<face_vertices.size(); j++) {
							Point_2 p2 = face_vertices.at(j);
							if(vertex2index.find(p2) != vertex2index.end()) {
								int index2 = vertex2index.at(p2);
								double dist = sqrt(CGAL::squared_distance(p1, p2).to_double());

								graph[index1][index2] = dist;
								graph[index2][index1] = dist;
							}
						}
					}

				}
			}
		}

	}

	cout << "number of vertices : "<< numV << endl;
	for(int i=0; i< numV; i++) {
		for(int j = 0; j < numV; j++) {
			if(graph[i][j] == INF)
				cout << "inf ";
			else cout << graph[i][j] << " ";
		}
		cout << endl;
	}

	//output mesh structure using ipe
	std::ofstream myFile;
	std::ifstream Template;
	 std::string line2;
	Template.open("ipe2.xml");
	myFile.open("Ipe.xml");
	while (std::getline(Template,line2)) {
		myFile <<line2<<"\n";
	}
	myFile << "<page>\n";
	for (auto i=free_space_arrangement.vertices_begin(); i!=free_space_arrangement.vertices_end(); i++) {
	myFile << "<use name=\"mark/disk(sx)\" " << "pos= \"" << i->point().x().to_double() << " " << i->point().y().to_double() << "\" size=\"normal\" stroke=\"black\"/>\n";
	}
	for (auto i = free_space_arrangement.edges_begin(); i!=free_space_arrangement.edges_end(); i++) {
	Point_2 p1 = i->source()->point();
	Point_2 p2 = i->target()->point();
	myFile << "<path stroke = \"black\"> \n"  << p1.x().to_double() <<" "<< p1.y().to_double() <<" m \n" << p2.x().to_double() <<" "<<p2.y().to_double() << " l \n" << "</path> \n";
	}
	myFile << "</page>\n";
	myFile << "</ipe>\n";
	myFile.close();
*/
		//convert triplets to trapezoid
		//go over all faces created in the decomposition
		//go over the trapezoids
//
//
//
//	 auto i = free_space_arrangement.faces_begin();
//
//	 while (!i->has_outer_ccb()) {
//		 i++;
//	 }
//
//
//	 Arrangement_2::Ccb_halfedge_circulator outerCCb = i->outer_ccb();
//
//	 cout<<"face circulator"<<endl;
//	 auto j = outerCCb;
//
//	 do {
//		 cout<<j->target()->point()<<" "<<j->source()->point()<<endl;
//		 j++;
//
//	 }	while (j!=outerCCb);



	return vector<pair<Point_2, Point_2>>();
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        cerr << "[USAGE]: inputRobots inputObstacles outputFile" << endl;
        return 1;
    }

    ifstream inputRobotsFile(argv[1]), inputObstaclesFile(argv[2]);
    if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        return -1;
    }

    auto startPoint1 = loadPoint_2(inputRobotsFile);
    auto endPoint1 = loadPoint_2(inputRobotsFile);
    auto startPoint2 = loadPoint_2(inputRobotsFile);
    auto endPoint2 = loadPoint_2(inputRobotsFile);
    inputRobotsFile.close();

    auto outer_obstacle = loadPolygon(inputObstaclesFile);
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();

    boost::timer timer;
    auto result = findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
    auto secs = timer.elapsed();
    cout << "Path created:      " << secs << " secs" << endl;

    ofstream outputFile;
    outputFile.open(argv[3]);
    if (!outputFile.is_open()) {
        cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
        return -1;
    }
    outputFile << result.size() << endl;
    for (auto &p : result) {
        outputFile << p.first.x().to_double() << " " << p.first.y().to_double() << " " << p.second.x().to_double()
                   << " " << p.second.y().to_double() << endl;
    }
    outputFile.close();
    return 0;
}
