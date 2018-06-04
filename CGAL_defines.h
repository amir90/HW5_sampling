//
// Created by t-idkess on 18-Mar-18.
//

#ifndef INC_5_CGAL_DEFINES_H
#define INC_5_CGAL_DEFINES_H

#include <CGAL/Gmpq.h>
#include <CGAL/enum.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Arr_vertical_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Fuzzy_sphere.h>
#include<CGAL/Search_traits_3.h>


typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename Kernel::Vector_2 Vector_2;
typedef typename Kernel::Line_2 Line_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Gps_segment_traits_2< Kernel> Gps_traits_2;
typedef typename CGAL::Arr_extended_dcel<Gps_traits_2,bool,int,bool> Dcel;
//edge data - if legal wall has an index, otherwise index=-1
//face data - is obstacle
typedef typename CGAL::Arrangement_2<Gps_traits_2, Dcel> Arrangement_2;
typedef typename CGAL::General_polygon_set_2<Gps_traits_2> Polygon_set_2;
typedef typename Arrangement_2::Vertex_const_handle Arr2_Vertex;
typedef typename Arrangement_2::Halfedge_const_handle Arr2_hEdge;
typedef typename Arrangement_2::Face_const_handle Arr2_Face;
typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Walk_pl;
typedef typename CGAL::Arr_trapezoid_ric_point_location<Arrangement_2> trapezoidalPl;


#endif //INC_5_CGAL_DEFINES_H
