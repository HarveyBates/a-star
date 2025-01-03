#include <SDL.h>
#include <stdbool.h>

// Window size, equal x and y
#define WINDOW_SIZE 720
#define CELL_COUNT 40
#define CELL_SIZE (WINDOW_SIZE / CELL_COUNT)

// A* Constants
// (Theoretical) Number of neighbours around each cell
#define NEIGHBOURS_COUNT 8
// Max size of queue
#define QUEUE_SIZE (CELL_COUNT * CELL_COUNT)

// Adjust these to adjust the start and end positions
// Starting cell
#define START_X 3  // Must be less than CELL_COUNT
#define START_Y 8  // Must be less than CELL_COUNT
// Target cell
#define TARGET_X 38  // Must be less than CELL_COUNT
#define TARGET_Y 38  // Must be less than CELL_COUNT

// Window utilities
SDL_Renderer* renderer = NULL;
SDL_Window* window = NULL;

// Window function
bool window_init();
void window_kill();
bool window_mainloop();

// Draw functions
void draw_window();
void draw_cells();
void draw_grid();
void create_barriers(int n_barriers);
bool set_cell_colour(int x, int y);

// Cell state
typedef enum {
    CELL_EMPTY,
    CELL_START,
    CELL_BARRIER,
    CELL_VISITED,
    CELL_NEIGHBOUR,
    CELL_PATH,
    CELL_TARGET
} CellState_Typedef;

// Position of a cell
typedef struct {
    int x;
    int y;
} CellPosition_Typedef;

// All information about a cell
typedef struct {
    CellPosition_Typedef position;
    double h_cost;
    double g_cost;
    double cost;
    CellState_Typedef state;
    CellPosition_Typedef parent_position;
} Cell_Typedef;

// Queue for open cells
typedef struct {
    Cell_Typedef cells[QUEUE_SIZE];
    int idx;
} Queue_Typedef;

// Grid of state for each cell
Cell_Typedef grid[CELL_COUNT][CELL_COUNT];

// Nodes to be evaluated (queue)
Queue_Typedef open_nodes_queue = {.idx = -1};

// Already evaluated nodes (just an array)
int closed_nodes_count = 0;
Cell_Typedef closed_nodes[QUEUE_SIZE];

// Flag for when the target is found
bool found_target = false;

// Some queue actions
// Check if queue is full
bool is_full(Queue_Typedef* queue) { return (queue->idx > QUEUE_SIZE - 1); }
// Check if the queue is empty
bool is_empty(Queue_Typedef* queue) { return (queue->idx < 0); }
// Queue a cell (end of queue)
void enqueue(Queue_Typedef* queue, Cell_Typedef cell) {
    if (is_full(queue)) return;
    queue->cells[++queue->idx] = cell;
}
// Get an item off the queue (and "remove")
bool pop(Queue_Typedef* queue, Cell_Typedef* cell) {
    if (is_empty(queue)) return false;
    *cell = queue->cells[queue->idx--];
    return true;
}

// Util functions
// Used in qsort to sort from largest to smallest cost (smallest is popped
// first)
int compare_cell_cost(const void* a, const void* b) {
    Cell_Typedef* qA = (Cell_Typedef*)a;
    Cell_Typedef* qB = (Cell_Typedef*)b;
    return (qA->cost < qB->cost) ? 1 : (qA->cost > qB->cost) ? -1 : 0;
}

// Distance between neighbouring cells
// 14 is roughly sqrt(2) - diagonal
// 10 is adjacent squares
double compute_distance(int x1, int y1, int x2, int y2) {
    double dx = abs(x1 - x2);
    double dy = abs(y1 - y2);
    if (dx > dy) return 14 * dy + 10 * (dx - dy);
    return 14 * dx + 10 * (dy - dx);
}

// Distance from start to some cell
double g(int x1, int y1) { return compute_distance(x1, y1, START_X, START_Y); }

// Distance from target to some cell
double h(int x1, int y1) {
    return compute_distance(x1, y1, TARGET_X, TARGET_Y);
}

// One target found the path is drawn
void draw_path(Cell_Typedef* current_cell) {
    int force_stop = 0;
    Cell_Typedef tmp_cell =
        grid[current_cell->position.x][current_cell->position.y];
    while (force_stop < 1000) {  // Limit is arbitrary (but should not be)
        grid[tmp_cell.position.x][tmp_cell.position.y].state = CELL_PATH;
        tmp_cell = grid[tmp_cell.parent_position.x][tmp_cell.parent_position.y];
        if (tmp_cell.parent_position.x == -1 &&
            tmp_cell.parent_position.y == -1) {
            break;  // Back at start
        }
        force_stop++;
    }
}

// A*
void a_star() {
    Cell_Typedef current_cell;

    // Stop if already found target
    if (found_target) {
        return;
    }

    // Get the current cell (lowest f-score off the queue)
    if (!pop(&open_nodes_queue, &current_cell)) {
        return;
    }

    // Add current node to the list of closed nodes
    closed_nodes[closed_nodes_count++] = current_cell;

    // Get the x and y position of the current cell
    int x = current_cell.position.x, y = current_cell.position.y;

    // Set the current cell colour to a travelled cell colour
    if (grid[x][y].state != CELL_START) {
        grid[x][y].state = CELL_VISITED;
    }

    // Get the positions of all 8 neighbours around the current cell
    CellPosition_Typedef neighbours[NEIGHBOURS_COUNT] = {
        {.x = x - 1, .y = y - 1}, {.x = x + 1, .y = y + 1},
        {.x = x + 1, .y = y - 1}, {.x = x - 1, .y = y + 1},
        {.x = x + 1, .y = y},     {.x = x - 1, .y = y},
        {.x = x, .y = y - 1},     {.x = x, .y = y + 1},
    };

    // Assess each of the neighbours
    for (int n = 0; n < NEIGHBOURS_COUNT; n++) {
        // Get a neighbour
        CellPosition_Typedef neighbour = neighbours[n];
        if (neighbour.x < 0 || neighbour.x > CELL_COUNT - 1 ||
            neighbour.y < 0 || neighbour.y > CELL_COUNT - 1) {
            continue;  // Skip out of bounds neighbours
        }

        // Check if the neighbour is the target
        if (grid[neighbour.x][neighbour.y].state == CELL_TARGET) {
            found_target = true;
            draw_path(&current_cell);
            return;
        }

        // Check if the neighbour is a barrier
        if (grid[neighbour.x][neighbour.y].state == CELL_BARRIER) {
            continue;
        }

        // Check if the neighbour is already visited
        bool is_closed = false;
        for (int c = 0; c < closed_nodes_count; c++) {
            if (closed_nodes[c].position.x == neighbour.x &&
                closed_nodes[c].position.y == neighbour.y) {
                is_closed = true;
                break;
            }
        }
        if (is_closed) {
            continue;
        }

        // Check if neighbour is in the queue
        bool in_queue = false;
        for (int o = 0; o < open_nodes_queue.idx; o++) {
            if (open_nodes_queue.cells[o].position.x == neighbour.x &&
                open_nodes_queue.cells[o].position.y == neighbour.y) {
                in_queue = true;
                break;
            }
        }

        // Calculate the g-cost (distance from the start) plus the cost of
        // the current cell
        double neighbour_g =
            current_cell.g_cost + compute_distance(current_cell.position.x,
                                                   current_cell.position.y,
                                                   neighbour.x, neighbour.y);

        // Check if the neighbour is not in the queue or its g-cost is lower
        // than the current g-cost if so, add it to the queue
        if (!in_queue || neighbour_g <= g(neighbour.x, neighbour.y)) {
            grid[neighbour.x][neighbour.y].position.x = neighbour.x;
            grid[neighbour.x][neighbour.y].position.y = neighbour.y;
            grid[neighbour.x][neighbour.y].g_cost = neighbour_g;
            grid[neighbour.x][neighbour.y].h_cost = h(neighbour.x, neighbour.y);
            grid[neighbour.x][neighbour.y].cost =
                neighbour_g + h(neighbour.x, neighbour.y);
            grid[neighbour.x][neighbour.y].parent_position.x =
                current_cell.position.x;
            grid[neighbour.x][neighbour.y].parent_position.y =
                current_cell.position.y;
            if (grid[neighbour.x][neighbour.y].state == CELL_EMPTY) {
                grid[neighbour.x][neighbour.y].state = CELL_NEIGHBOUR;
            }
            enqueue(&open_nodes_queue, grid[neighbour.x][neighbour.y]);
        }
    }

    // Sort the queue by f-cost
    qsort(open_nodes_queue.cells, open_nodes_queue.idx + 1,
          sizeof(Cell_Typedef), compare_cell_cost);
}

int main() {
    if (!window_init()) {
        return 1;
    }

    // Start Cell
    grid[START_X][START_Y].state = CELL_START;
    grid[START_X][START_Y].position.x = START_X;
    grid[START_X][START_Y].position.y = START_Y;
    grid[START_X][START_Y].g_cost = g(START_X, START_Y);
    grid[START_X][START_Y].h_cost = h(START_X, START_Y);
    grid[START_X][START_Y].cost =
        grid[START_X][START_Y].g_cost + grid[START_X][START_Y].h_cost;
    grid[START_X][START_Y].parent_position.x = -1;
    grid[START_X][START_Y].parent_position.y = -1;
    enqueue(&open_nodes_queue, grid[START_X][START_Y]);

    // End cell
    grid[TARGET_X][TARGET_Y].state = CELL_TARGET;
    grid[TARGET_X][TARGET_Y].position.x = TARGET_X;
    grid[TARGET_X][TARGET_Y].position.y = TARGET_Y;

    create_barriers(1000);

    while (window_mainloop()) {
        a_star();
        SDL_Delay(10);
    }

    window_kill();
    return 0;
}

bool set_cell_colour(int x, int y) {
    if (x < 0 || x > CELL_COUNT - 1 || y < 0 || y > CELL_COUNT - 1) {
        return false;  // Invalid range
    }

    switch (grid[x][y].state) {
        case CELL_EMPTY:
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            break;
        case CELL_START:
            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            break;
        case CELL_BARRIER:
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            break;
        case CELL_PATH:
            SDL_SetRenderDrawColor(renderer, 255, 50, 0, 255);
            break;
        case CELL_VISITED:
            SDL_SetRenderDrawColor(renderer, 0, 180, 0, 255);
            break;
        case CELL_NEIGHBOUR:
            SDL_SetRenderDrawColor(renderer, 255, 234, 0, 255);
            break;
        case CELL_TARGET:
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            break;
        default:
            return false;
    }

    return true;
}

void create_barriers(int n_barriers) {
    int n_created = 0;
    srand(time(NULL));
    while (n_created < n_barriers) {
        int rand_x = rand() % CELL_COUNT;
        int rand_y = rand() % CELL_COUNT;
        if (grid[rand_x][rand_y].state != CELL_START &&
            grid[rand_x][rand_y].state != CELL_TARGET) {
            grid[rand_x][rand_y].state = CELL_BARRIER;
            n_created++;
        }
    }
}

void draw_cells() {
    for (int x = 0; x < CELL_COUNT; x++) {
        for (int y = 0; y < CELL_COUNT; y++) {
            SDL_Rect cell = {.x = x * CELL_SIZE,
                             .y = y * CELL_SIZE,
                             .w = CELL_SIZE,
                             .h = CELL_SIZE};
            set_cell_colour(x, y);
            SDL_RenderFillRect(renderer, &cell);
        }
    }
}

void draw_grid() {
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    for (uint16_t offset = 0; offset < WINDOW_SIZE; offset += CELL_SIZE) {
        SDL_RenderDrawLine(renderer, offset, 0, offset, 720);
        SDL_RenderDrawLine(renderer, 0, offset, 720, offset);
    }
}

void draw_window() {
    draw_cells();
    draw_grid();
}

bool window_mainloop() {
    SDL_Event event;

    // Clear window
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    // Check for exit event
    while (SDL_PollEvent(&event) != 0) {
        switch (event.type) {
            case SDL_QUIT:
                return false;
            default:
                break;
        }
    }

    draw_window();

    // Update window
    SDL_RenderPresent(renderer);

    return true;
}

bool window_init() {
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        printf("Error initialising SDL: %s\n", SDL_GetError());
        return false;
    }

    window = SDL_CreateWindow("A-Star", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, WINDOW_SIZE, WINDOW_SIZE,
                              SDL_WINDOW_SHOWN);

    if (!window) {
        printf("Error creating SDL window: %s\n", SDL_GetError());
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer) {
        printf("Error creating SDL renderer: %s\n", SDL_GetError());
        return false;
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    return true;
}

void window_kill() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
