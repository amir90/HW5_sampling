#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

using namespace std;



#define Nrand 3500
#define K 20
#define TIMEOUT 1000

const double INF = numeric_limits<double>::max();

Arrangement_2 free_space_arrangement;



struct Comp{

Comp(std::vector<double> *f) {this->f=f;}
bool operator()( int a ,int b) {
	  if ((f->at(a)>f->at(b)) || (a==b)) {
//		  cout<<a<<" "<<f->at(a)<<" "<<b<<" "<<f->at(b)<<endl;
		  return false;
	  }
	   else {
		  return true;
	  }
}
std::vector<double> *f;
};


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

struct qPoint {
	Point_2 xy1;
	Point_2 xy2;
	int index;
	double vec[4];


	void set(double x1, double y1, double x2, double y2) {
		xy1 = Point_2(x1,y1);
		xy2 = Point_2(x2, y2);
	}

	  bool operator==(const qPoint& p) const
	  {
	    return (index==p.index)  ;
	  }
	  bool  operator!=(const qPoint& p) const { return ! (*this == p); }
};

struct Construct_coord_iterator {
  typedef  const double* result_type;
  const double* operator()(const qPoint& p) const
  { return static_cast<const double*>(p.vec); }
  const double* operator()(const qPoint& p, int)  const
  { return static_cast<const double*>(p.vec+4); }
};

double heuristic(qPoint q1, qPoint q2) {
	//euclidean heuristic
	return std::sqrt((q1.xy1-q2.xy1).squared_length().to_double())+std::sqrt((q1.xy2-q2.xy2).squared_length().to_double());

}

double rand_between(double high, double low) {
	return low + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(high-low)));
}

qPoint newRandomQPoint(double xmin, double xmax, double ymin, double ymax) {
	double x1 = rand_between(xmin,xmax);
	double y1 = rand_between(ymin, ymax);

	double x2 = rand_between(xmin,xmax);
	double y2 = rand_between(ymin, ymax);

	qPoint p;
	p.set(x1,y1,x2,y2);
	p.vec[0] = x1;
	p.vec[1] = y1;
	p.vec[2] = x2;
	p.vec[3] = y2;
	return p;
}

bool withinEnv(double x1, double x2, double env) {
	return x2 < x1 + env && x2 > x1 - env;
}
bool areColliding(Point_2 p1, Point_2 p2) {
	double x1 = p1.x().to_double(), y1 = p1.y().to_double();
	double x2 = p2.x().to_double(), y2 = p2.y().to_double();
	return withinEnv(x1,x2,1) && withinEnv(y1,y2,1);
}

bool isCollisionConfiguration(qPoint q) {
	return areColliding(q.xy1, q.xy2);
}

bool isLegalPoint(Point_2 p,Arrangement_2 &arr,trapezoidalPl &pl) {
	auto location = pl.locate(p);

	Arrangement_2::Face_handle face;

	if(CGAL::assign(face, location)) {

		if (face->data())  {//is obstacle
			return false;
		}

	}

	return true;

}

bool isLegalConfiguration(qPoint p,Arrangement_2 &arr,trapezoidalPl &pl) {
	return isLegalPoint(p.xy1, arr, pl) &&
		isLegalPoint(p.xy2, arr, pl) && !isCollisionConfiguration(p);
}


bool intersect (Rectangle rect, Segment_2 seg) {

	auto seg1 = Segment_2(rect.vertex(0),rect.vertex(1));
	auto seg2 = Segment_2(rect.vertex(1),rect.vertex(2));
	auto seg3 = Segment_2(rect.vertex(2),rect.vertex(3));
	auto seg4 = Segment_2(rect.vertex(3),rect.vertex(0));

	return (CGAL::do_intersect(seg,seg1) || CGAL::do_intersect(seg,seg2) || CGAL::do_intersect(seg,seg3) || CGAL::do_intersect(seg,seg4));

}


//1 - robot 1 goes first
//-1 - robot 2 goes first
//0 - no path

int localPlanner (qPoint q1 ,qPoint q2,Arrangement_2 &arr ) {


	//check if path 1 is ok

	Segment_2 robot1 = Segment_2(q1.xy1,q2.xy1);
	std::vector<CGAL::Object> zone_elems;
	CGAL::zone(arr,robot1,std::back_inserter(zone_elems));
	Arrangement_2::Face_handle face;
	  for ( int i = 0; i < (int)zone_elems.size(); ++i )
	    {
	      if (assign(face, zone_elems[i]) ) {
	    	  if (face->data()==true) { //if obstacle
	    		  return 0;
	    	  }
	      }
	    }

	  //check if path 2 is ok

		Segment_2 robot2 = Segment_2(q1.xy2,q2.xy2);
		CGAL::zone(arr,robot2,std::back_inserter(zone_elems));
		  for ( int i = 0; i < (int)zone_elems.size(); ++i )
		    {
		      if (assign(face, zone_elems[i]) ) {
		    	  if (face->data()==true) { //if obstacle
		    		  return 0;
		    	  }
		      }
		    }

		//create squares
		  Rectangle robot1StartRect(q1.xy1+Vector_2(1,1),q1.xy1+Vector_2(-1,-1) );
		  Rectangle robot2StartRect (q1.xy2+Vector_2(1,1),q1.xy2+Vector_2(-1,-1) );
		  Rectangle robot1EndRect (q2.xy2+Vector_2(1,1),q2.xy2+Vector_2(-1,-1) );
		  Rectangle robot2EndRect (q2.xy2+Vector_2(1,1),q2.xy2+Vector_2(-1,-1) );


		  //check if robot 1 can go first
		  if (!(intersect(robot2StartRect,robot1)) && !(intersect(robot1EndRect,robot2))) {
			  return 1;
		  }

		  //check if robot 2 can go first
		  if (!(intersect(robot1StartRect,robot2)) && !(intersect(robot2EndRect,robot1))) {
			  return -1;
		  }

		return 0;
}

double dist(qPoint q1, qPoint q2) {
	//TODO: local planner is now calcualted as part of the knn tree, if the result is not 0 (no admissible movement) then saving the result to a matrix should speed up performance
	// by not requiring re calculation when computing local planner in the path finding.

	if (localPlanner(q1,q2,free_space_arrangement)!=0) {
	return std::sqrt((q1.xy1-q2.xy1).squared_length().to_double())+std::sqrt((q1.xy2-q2.xy2).squared_length().to_double());
	} else {
		return INF;
	}
}

//return 1 if robot 1 goes first
//return -1 if robot 2 goes first


std::pair<double,int> cost (qPoint q1, qPoint q2, Arrangement_2 arr) {

	//if movement is legal, get cost of movement.
	//Otherwise - return infinite cost
	int j = localPlanner(q1,q2,arr);
	//cout << "calc local planner from : [ (" << point2string(q1.xy1) << ") , (" << point2string(q1.xy2) << ")]" << endl;
	//cout << "calc local planner from : [ (" << point2string(q2.xy1) << ") , (" << point2string(q2.xy2) << ")]" << endl;
	//cout << "result  : " << j << endl;
	std::pair<double,int> temp;
	if (j!=0) {
		temp.first = std::sqrt((q1.xy1-q2.xy1).squared_length().to_double())+std::sqrt((q1.xy2-q2.xy2).squared_length().to_double());
		temp.second = j;
	} else {
		temp.first=INF;
		temp.second = 0;
	}

	return temp;

}

struct Distance {
  typedef qPoint Query_item;
  typedef double FT;
  typedef CGAL::Dimension_tag<4> D;
  double transformed_distance(const qPoint& p1, const qPoint& p2) const {
    return dist(p1,p2);
  }
  double min_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b) const {
    double distance(0.0), h = p.xy1.x().to_double();
    if (h < b.min_coord(0)) distance += (b.min_coord(0)-h)*(b.min_coord(0)-h);
    if (h > b.max_coord(0)) distance += (h-b.max_coord(0))*(h-b.max_coord(0));
    h=p.xy1.y().to_double();
    if (h < b.min_coord(1)) distance += (b.min_coord(1)-h)*(b.min_coord(1)-h);
    if (h > b.max_coord(1)) distance += (h-b.max_coord(1))*(h-b.min_coord(1));
    h=p.xy2.x().to_double();
    if (h < b.min_coord(2)) distance += (b.min_coord(2)-h)*(b.min_coord(2)-h);
    if (h > b.max_coord(2)) distance += (h-b.max_coord(2))*(h-b.max_coord(2));
    return distance;
    h=p.xy2.y().to_double();
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
    return dists[0] * dists[0] + dists[1] * dists[1] + dists[2] * dists[2] + dists[3] * dists[3];
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


typedef CGAL::Dimension_tag<4> D;
typedef CGAL::Search_traits<double, qPoint, const double*, Construct_coord_iterator, D> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits, Distance> K_neighbor_search;
typedef K_neighbor_search::Tree Tree;
typedef typename CGAL::Fuzzy_sphere<Traits> Fuzzy_Circle;

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

	for(int i = 0; i < obstacles_size; i++) {
		Polygon_2 obstacle = obstacles.at(i);
		CGAL::insert(free_space_arrangement,obstacle.edges_begin(),obstacle.edges_end());
	}



	// create an arrangement from the polygon set
	//add outer polygon, if exists

	Arrangement_2::Halfedge_handle OuterEdge;

	bool outerObstacleFlag = true;

	if (outer_obstacle.edges_begin()==outer_obstacle.edges_end()) { // no outer obstacle
		outerObstacleFlag=false;
	} else { //add outer polygon
		Segment_2 addSeg(outer_obstacle.edges_begin()->point(0),outer_obstacle.edges_begin()->point(1));
		OuterEdge = CGAL::insert_non_intersecting_curve(free_space_arrangement,addSeg);
		for (auto i=std::next(outer_obstacle.edges_begin(),1); i!=outer_obstacle.edges_end(); i++) {
			addSeg = Segment_2(i->point(0),i->point(1));
			CGAL::insert(free_space_arrangement,addSeg);
		}

	}


	//identify obstacle faces
	// data = true -> face is an obstacle
	// data = false -> face is not an obstacle

	for (auto i=free_space_arrangement.faces_begin(); i!=free_space_arrangement.faces_end(); i++) {
		if (!outerObstacleFlag && i->is_unbounded()) {
			i->set_data(false);
		} else {
			i->set_data(true);
		}
		}

	if (outerObstacleFlag) {
		auto face1 = OuterEdge->face();
		auto face2 = OuterEdge->twin()->face();
		if (!(face1->is_unbounded())) {
			face1->set_data(false);
		} else {
			face2->set_data(false);
		}
	}

	/*
	//test faces: test passed
int testCounter=0;
int faceCounter=0;
	for (auto i=free_space_arrangement.faces_begin(); i!=free_space_arrangement.faces_end(); i++) {
		faceCounter++;
		if (i->data()==true) {
			testCounter++;
		}
	}
	cout<<"there are: "<<testCounter<<" obstacle faces"<<endl;
	cout<<"out of: "<<faceCounter<<endl;
*/

	trapezoidalPl   pl(free_space_arrangement);


	CGAL::Bbox_2 bbox = outer_obstacle.bbox();

	for (Polygon_2 p: obstacles) {

		bbox = bbox+p.bbox();

	}

	bbox = bbox + Segment_2(start1,start2).bbox();

	bbox = bbox + Segment_2(end1,end2).bbox();

	double xmin = bbox.xmin(), xmax = bbox.xmax(),
	ymin = bbox.ymin(), ymax = bbox.ymax();


	vector<qPoint> V; //Vertices;

	qPoint qstart, qend;
	qstart.xy1 = start1;
	qstart.xy2 = start2;
	qstart.index = 0;
	qend.xy1 = end1;
	qend.xy2 = end2;
	qend.index = 1;
	V.push_back(qstart);
	V.push_back(qend);

	int currInd = 2;

	int counter1=0;
	for (auto i=free_space_arrangement.vertices_begin(); i!=free_space_arrangement.vertices_end(); i++) {
		qPoint temp1;
		temp1.xy1 = i->point();
		temp1.vec[0] = temp1.xy1.x().to_double();
		temp1.vec[1] = temp1.xy1.y().to_double();

		for (auto j=free_space_arrangement.vertices_begin(); j!=free_space_arrangement.vertices_end(); j++) {

			temp1.xy2 = j->point();
			temp1.vec[2] = temp1.xy2.x().to_double();
			temp1.vec[3] = temp1.xy2.y().to_double();
			if(isLegalConfiguration(temp1, free_space_arrangement,pl)) {
			temp1.index = currInd;
			V.push_back(temp1);
			currInd++;
			counter1++;
			}

		}
	}





	//N random configurations
	// TODO : for faster caculation - if one point is legal and the 
	//        other doesn't recalculate one instead of both
	int currRandPoints=0;
	int counter =0;
 	while (counter < TIMEOUT && currRandPoints<(Nrand-counter1) ) {
		qPoint temp = newRandomQPoint(xmin,xmax,ymin,ymax);
			if(isLegalConfiguration(temp, free_space_arrangement,pl)) {
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
	//((bbox.xmax()-bbox.xmin())*(bbox.ymax()-bbox.xmin()))*
	double radius = 20*pow((log2(N)/N),0.25);//*(bbox.xmax()-bbox.xmin());//((bbox.xmax()-bbox.xmin())*(bbox.ymax()-bbox.ymin()));

	std::vector<std::list<qPoint>> neighbors(N);

	for (qPoint q: V ) { //sorted by index

	//	Fuzzy_Circle fc(q,radius);

	//	tree.search(std::back_inserter(neighbors[q.index]), fc);

		K_neighbor_search search(tree, q, K);
/*
		for (auto i=search.begin(); i!=search.end(); i++) {
		neighbors[q.index].push_back((*i).first);
		}
*/
	//	cout<<"K: "<<K<<endl;


		for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
	//	for (auto q1: neighbors[q.index]) {
		    	qPoint q1 = it->first;
		    	neighbors[q.index].push_back(q1);
			if (q1.index != q.index) {
		    	neighbors[q1.index].push_back(q);
			}

		}

	//	cout<<"neigbors: "<<neighbors[q.index].size()<<endl;
	}

// check population of neighbors: test passed
	/*
	std::cout<<"done finding nodes"<<endl;
	for (auto i: neighbors) {
		cout<<i.size()<<endl;
	}
*/

	//test path
/*
	Segment_2 seg(Point_2(3,0), Point_2(-5.36459,4.44451));
	std::vector<CGAL::Object> zone_elems;
	CGAL::zone(free_space_arrangement,seg,std::back_inserter(zone_elems));
	Arrangement_2::Face_handle face;
	  for ( int i = 0; i < (int)zone_elems.size(); ++i )
	    {
		  cout<<"in loop!"<<endl;
	      if (assign(face, zone_elems[i]) ) {
	    	  cout<<face->data()<<endl;
	    	  if (face->data()==false) { //if obstacle
	    		  cout<<"illegal move!"<<endl;
	    	  }
	      }
	    }
*/

	std::vector<double> g(N,numeric_limits<double>::max());
//	std::vector<double> *f = new vector<double>(N,numeric_limits<double>::max());
	std::vector<double> f(N,numeric_limits<double>::max());
	std::vector<int> parent(N,-1);
	std::vector<int> Orient(N,2);

		//f->at(0) = heuristic(V[0],V[1]);
		f[0] = heuristic(V[0],V[1]);
		g[0]=0;

	bool foundPath = false;

	//std::set<int> Open;
	std::set<int> Closed;
	//std::set<int,Comp> Open {Comp{f}};
	std::set<int> Open;

	//test Open
/*
	f->at(0)=1; f->at(1) = 5; f->at(2)=2; f->at(5)=1;
	Open.insert(0); Open.insert(2); Open.insert(1); Open.insert(5);
	auto k = Open.erase(1);
	f->at(1)=0;
	 Open.insert(1);
	cout<<"erased: "<<k<<endl;
	for (auto i=Open.begin(); i!=Open.end(); i++) {
		cout<<*i<<endl;
		cout<<"testing set"<<endl;
	}
*/
	Open.insert(0);

	cout << "V.size()" << V.size() << endl;

	 while (!Open.empty()) {
		 int min_f_ind = *(Open.begin());

		 //testing naive implmementation
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

			auto temp = cost(v,q,free_space_arrangement);

			if (temp.second==0) {continue;}
			Open.insert(q.index);

			if (g[q.index]<=(g[v.index] + temp.first)) { //this is not a better path
				continue;
			}
			parent[q.index] = v.index; g[q.index]=g[v.index]+temp.first;
			//Open.erase(q.index);
			//f->at(q.index) = g[q.index]+heuristic(q,V[1]);
		   //Open.insert(q.index);
			f[q.index] = g[q.index]+heuristic(q,V[1]);
			Orient[q.index] = temp.second; //which robot goes first


		}
	}


	//delete(f);
	std::vector<int> path;


	if (foundPath) {
		//reverse path
		int currInd = 1;
		while (currInd!=0) {
			path.push_back(currInd);
			currInd = parent[currInd];
		}
		path.push_back(0);
	}

	cout << "foundPath? " << foundPath << endl;
	cout << "path length " << path.size() << endl;

	vector<pair<Point_2, Point_2>> points_path;
	Point_2 prev1, prev2;
	for(auto i = path.begin(); i != path.end(); i++) {
		qPoint q = V.at(*i);
		Point_2 p1 = q.xy1;
		Point_2 p2 = q.xy2;
		
		if(i == path.begin()) {
			points_path.push_back(pair<Point_2, Point_2>(p1, p2));
			cout<<p1.x().to_double()<<","<<p1.y().to_double()<<"  "<<p2.x().to_double()<<","<<p2.y().to_double()<<endl;
			prev1 = p1;
			prev2 = p2;
			continue;
		}

			if(Orient[q.index] == 1) {
				points_path.push_back(pair<Point_2, Point_2>(p1, prev2));
				points_path.push_back(pair<Point_2, Point_2>(p1, p2));

				cout<<"robot 1 goes first"<<endl;
				cout<<p1.x().to_double()<<","<<p1.y().to_double()<<"  "<<prev2.x().to_double()<<","<<prev2.y().to_double()<<endl;
				cout<<p1.x().to_double()<<","<<p1.y().to_double()<<"  "<<p2.x().to_double()<<","<<p2.y().to_double()<<endl;

			} else {
				cout<<"robot 2 goes first"<<endl;
				points_path.push_back(pair<Point_2, Point_2>(prev1, p2));
				points_path.push_back(pair<Point_2, Point_2>(p1, p2));

				cout<<prev1.x().to_double()<<","<<prev1.y().to_double()<<"  "<<p2.x().to_double()<<","<<p2.y().to_double()<<endl;
				cout<<p1.x().to_double()<<","<<p1.y().to_double()<<"  "<<p2.x().to_double()<<","<<p2.y().to_double()<<endl;

			}


		prev1 = p1;
		prev2 = p2;
	}

	return points_path;
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
