#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

using namespace std;

const double INF = numeric_limits<double>::max();


double heuristic(Point_2 p1, Point_2 p2) {
	return 0;
}

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


	//Trapezoidal decomposition
	typedef std::list<std::pair<Arrangement_2::Vertex_const_handle,
	                              std::pair<CGAL::Object, CGAL::Object> > >
	Vert_decomp_list;
	Vert_decomp_list vd_list;
	CGAL::decompose(free_space_arrangement, std::back_inserter(vd_list));

	Arr2_Vertex vert;
	Arr2_hEdge edge;
	Vector_2 horizontalVec = Point_2(0,1)-Point_2(0,0);
	Line_2 line;
	Segment_2 seg;
	list<Segment_2> segList;

	for(Vert_decomp_list::iterator it = vd_list.begin(); it != vd_list.end(); ++it) {
		Point_2 p = it->first->point();
		double x = p.x().to_double();
		double y = p.y().to_double();

		//check upper element
		//TODO: wrap all polygons with a bounding box slightly larger.
		if (assign(edge,it->second.first)) { //if upper element is non fictitious half-edge
			if (!edge->is_fictitious()) {
				//todo connect vertex to edge with vertical segment
				line = Line_2(it->first->point(),horizontalVec);
				seg = Segment_2(edge->source()->point(),edge->target()->point());
				auto result = (CGAL::intersection(line,seg));
				Point_2* p = boost::get<Point_2 >(&*(result));
				Segment_2 addSeg(*p,it->first->point());
				segList.push_back(addSeg);
	//			CGAL::insert(free_space_arrangement,addSeg);
			}
		}
		if (assign(vert,it->second.first)) { //if upper element is non fictitious half-edge
				Segment_2 addSeg(vert->point(),it->first->point());
				segList.push_back(addSeg);
	//			 CGAL::insert(free_space_arrangement,addSeg);

			}

		//check lower elements
		if (assign(edge,it->second.second)) { //if upper element is non fictitious half-edge
			if (!edge->is_fictitious()) {
				//todo connect vertex to edge with vertical segment
				line = Line_2(it->first->point(),horizontalVec);
				seg = Segment_2(edge->source()->point(),edge->target()->point());
				auto result = (CGAL::intersection(line,seg));
				Point_2* p = boost::get<Point_2 >(&*(result));
				Segment_2 addSeg(*p,it->first->point());
				segList.push_back(addSeg);
	//			CGAL::insert(free_space_arrangement,addSeg);
			}
		}
		if (assign(vert,it->second.second)) { //if upper element is non fictitious half-edge
				Segment_2 addSeg(vert->point(),it->first->point());
				segList.push_back(addSeg);
	//			 CGAL::insert(free_space_arrangement,addSeg);

			}
		}

	//add horizontal segments to arrangment
	for (auto i=segList.begin(); i!=segList.end(); i++) {
		CGAL::insert(free_space_arrangement,*i);
		vertices.insert(i->source());
		vertices.insert(i->target());
	}


	//the vertices are the middle of legal walls
	int N=0;
	std::vector<Arr2_hEdge> wallVec;
			for (auto i=free_space_arrangement.edges_begin(); i!=free_space_arrangement.edges_end(); i++) {

			if ((i->face()->data()==false) && (i->twin()->face()->data()==false)) {
				i->set_data(N); //illegal wall
				N++;
				wallVec.push_back(i->twin()->twin());
			} else {
				i->set_data(-1);
			}
			}

	//algorithm:

	std::vector<double> g(N,INF);

	std::vector<double> f(N,INF);


	//populate graph

	std::vector<list<Arr2_hEdge>> edges(N);

	for (auto i: wallVec) {
		auto temp = i->twin();
		do {
			if (temp->data()==0)  {//is legal wall
				edges[temp->data()].push_back(temp);
			}
			temp++;
		} while (temp!=(i->twin()));

		auto twin = i->twin()->twin();
		temp = twin;
		do {
			if (temp->data()>=0)  {//is legal wall
				edges[temp->data()].push_back(temp);
			}
			temp++;
		} while (temp!=twin);
	}


	std::set<int> Open; std::set<int> Closed;

	while (!Open.empty()) {

		//choose edge which minimized f(v);


	}




	//start and end vertices are only connected

	//find trapezoids in optimal path for red robot, using a* with euclidean distance heurisitc (robot is point in configuration space)
	//Open is set of indexes
	//Closed is set of indexes


	//TODO: currently, moves through edges of trapezoid, maybe implement something smarter

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
int counter=0;
	for (auto i=free_space_arrangement.edges_begin(); i!=free_space_arrangement.edges_end(); i++) {
		counter++;
	//	i->set_data(true);
	}

	cout<<counter<<endl;
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
