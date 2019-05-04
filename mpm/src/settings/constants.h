#pragma once

/* definitions */
#define WORLD_NUM_DIMENSIONS 3

#define PARTS_PER_OBJECT 10000

// uniform length of kernel
#define KERNEL_NUM_NODES_X 3

// lengths of 1 block
#define BLOCK_NUM_NODES_X 4
#define BLOCK_NUM_NODES_Y 4
#define BLOCK_NUM_NODES_Z 4

// lengths of world
#define WORLD_NODE_LENGTH 1 // dx distance between adjacent nodes
#define WORLD_NUM_NODES_X 64
#define WORLD_NUM_NODES_Y 64
#define WORLD_NUM_NODES_Z 64

// for alternating grid partitions
#define WORLD_NUM_SETS 8

/* derived definitions */
// number of neighbor nodes (including self) in 3 dimensions
#define NEIGHBOR_NUM_NODES 27

// total number of nodes in a kernel
#define KERNEL_NUM_NODES (KERNEL_NUM_NODES_X * KERNEL_NUM_NODES_X * KERNEL_NUM_NODES_X)

// length of block including neighbors
// #define BLOCK_RANGE_X (BLOCK_LEN_X + KERNEL_LEN - 1)
// #define BLOCK_RANGE_Y (BLOCK_LEN_Y + KERNEL_LEN - 1)
// #define BLOCK_RANGE_Z (BLOCK_LEN_Z + KERNEL_LEN - 1)

// total number of nodes in a block
#define BLOCK_NUM_NODES (BLOCK_NUM_NODES_X * BLOCK_NUM_NODES_Y * BLOCK_NUM_NODES_Z)

// // total number of neighbor nodes (aka number of ghost nodes)
// #define BLOCK_NEIGHBOR_SIZE ((BLOCK_RANGE_X * BLOCK_RANGE_Y * BLOCK_RANGE_Z) - BLOCK_SIZE)

// total number of nodes in world
#define WORLD_NUM_NODES (WORLD_NUM_NODES_X * WORLD_NUM_NODES_Y * WORLD_NUM_NODES_Z)

// number of blocks in the world
#define WORLD_NUM_BLOCKS_X (WORLD_NUM_NODES_X / BLOCK_NUM_NODES_X)
#define WORLD_NUM_BLOCKS_Y (WORLD_NUM_NODES_Y / BLOCK_NUM_NODES_Y)
#define WORLD_NUM_BLOCKS_Z (WORLD_NUM_NODES_Z / BLOCK_NUM_NODES_Z)
#define WORLD_NUM_BLOCKS (WORLD_NUM_NODES / BLOCK_NUM_NODES)

// total number of blocks in a set (alternating grid)
#define SET_NUM_BLOCKS (WORLD_NUM_BLOCKS / WORLD_NUM_SETS)

/* constraints on settings */
// evenly divide world into blocks
static_assert(WORLD_NUM_NODES_X % BLOCK_NUM_NODES_X == 0);
static_assert(WORLD_NUM_NODES_Y % BLOCK_NUM_NODES_Y == 0);
static_assert(WORLD_NUM_NODES_Z % BLOCK_NUM_NODES_Z == 0);
static_assert(WORLD_NUM_BLOCKS % WORLD_NUM_SETS == 0);
