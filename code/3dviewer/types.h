#ifndef TYPES_H
#define TYPES_H

// kernel
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

// Types
typedef Enriched_polyhedron<Kernel, Enriched_items> Polyhedron;
typedef CPoint_cloud<Kernel> Point_cloud;

typedef CGAL::Bbox_3 Bbox;

#endif // TYPES_H

