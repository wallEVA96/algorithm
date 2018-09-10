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
 *	  it mean that this point will didnt update again.
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
struct vertex;
typedef struct vertex *Vertex_p;
struct vertex {
	int vertex;
	int cost;
	Vertex_p next_vertex;
};

struct vertex_state{
	Vertex_p adj_list;
	int known;
	int dist;
	int path;
};

#define VERTEX_NUM 4
#define NOPATH 0
#define INFINITY 10000
#define TRUE 1
#define FALSE 0

struct vertex_state vertex_array[VERTEX_NUM] = {NULL,
												FALSE,
												INFINITY,
												NOPATH};
int minmum(struct vertex_state *vertex_array){


}

/* dijstra */
void dijstra(int vertex, struct vertex_state *vertex_array, size_t vertex_num)
{
	int tmp_dist = 0;
	for(;;){
		if(vertex == INFINITY)
				break;
		vertex_array[vertex].known = TRUE;
		Vertex_p adj_vertex = vertex_array[vertex].adj_list;
		while(adj_vertex->next_vertex != NULL){
			if(vertex_array[adj_vertex->next_vertex->vertex].known != TRUE)
					tmp_dist = vertex_array[adj_vertex->vertex].dist + adj_vertex->next_vertex->cost;
					if( tmp_dist < vertex_array[adj_vertex->next_vertex->vertex].dist){
					vertex_array[adj_vertex->next_vertex->vertex].dist = tmp_dist;
			}
			vertex = minmum(vertex_array);
		}
	}

}
/*  main */
int main(){


return 0;
}
