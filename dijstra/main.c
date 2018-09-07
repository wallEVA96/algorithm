/*
 * main.c
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2018 walleva <walleva@ubuntu>
 * Reference :
 * https://www.cs.usfca.edu/~galles/visualization/Dijkstra.html
 * Note:
 * 1, find cheapest vertex form unknown vertex(that din't update neighbor)
 * 2, update above vertex's neighbori
 *		add center vertex's cost with edge cost firstly,
 *		if calculcating new cost is cheaper than before, instead of that and set the 
 *		path vertex is the center vertex.
 * 3, set center vertex is known.
 * 4 recycle above until there are no cheap vertex or the cheaptest is INF. Abandon!
 * Distributed under terms of the MIT license.
 */


#include "stdio.h"

/* define */
typedef int Vertex;
struct vertex {
	Vertex vertex;
	Vertex cost;
	struct list *next_vertex;
};

typedef struct vertex *adj_list;
struct vertex_state{
	adj_list list;
	int known;
	int dist;
	Vertex path;
};

#define vertex_num 4
#define INFINITY 10000
#define FALSE 0
#define NOPATH 0
#define TRUE 1

struct vertex_state vertex_array[vertex_num] = {NULL,
												FALSE,
												INFINITY,
												NOPATH};

/* dijstra */

/*  main */
int main(){


return 0;
}
