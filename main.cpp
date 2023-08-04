#include <SDL2/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <vector>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

void drawPixel(SDL_Renderer* renderer, int x, int y) {
    SDL_RenderDrawPoint(renderer, x, y);
}

void line(SDL_Renderer* renderer, glm::vec3 start, glm::vec3 end) {
    int x1 = round(start.x), y1 = round(start.y);
    int x2 = round(end.x), y2 = round(end.y);

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        drawPixel(renderer, x1, y1);

        if (x1 == x2 && y1 == y2) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void drawPolygon(SDL_Renderer* renderer, const std::vector<glm::vec3>& vertices) {
    for (size_t i = 0; i < vertices.size() - 1; ++i) {
        line(renderer, vertices[i], vertices[i + 1]);
    }
    line(renderer, vertices.back(), vertices.front());
}

struct Face {
    std::vector<std::array<int, 3>> vertexIndices;
};


int main() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow("OBJ Polygon", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    bool isRunning = true;
    SDL_Event event;

    // Coordenadas de vértices del polígono
    std::vector<glm::vec3> vertices = {
        {0.125986f * WINDOW_WIDTH, 0.781798f * WINDOW_HEIGHT, 0.0f},
        {0.483198f * WINDOW_WIDTH, 0.372034f * WINDOW_HEIGHT, 0.0f},
        {0.425498f * WINDOW_WIDTH, 0.660990f * WINDOW_HEIGHT, 0.0f},
        {0.768286f * WINDOW_WIDTH, 0.015753f * WINDOW_HEIGHT, 0.0f},
        {0.125986f * WINDOW_WIDTH, 0.781798f * WINDOW_HEIGHT, 0.0f}
    };

    while (isRunning) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                isRunning = false;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        drawPolygon(renderer, vertices);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}