/*
 * main.c
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2018 walleva <walleva@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#include "stdio.h"
#include "stdlib.h"

//define first
struct treeNode;
typedef struct treeNode *ptr_treeNode;
struct treeNode {
	int element;
	ptr_treeNode left;
	ptr_treeNode right;
};

//struct node {
//	int index;
//	struct node *left;
//	struct node *right;
//};

int main(){
	ptr_treeNode tree_node = malloc(sizeof(ptr_treeNode));
	printf("%ld %ld\n",sizeof(unsigned int),sizeof(struct treeNode));

	
	return 0;
}
