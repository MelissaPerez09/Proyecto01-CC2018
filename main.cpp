#include <SDL2/SDL.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <vector>
#include <array>
#include <fstream>
#include <sstream>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

void drawPixel(SDL_Renderer* renderer, int x, int y) {
    SDL_RenderDrawPoint(renderer, x, y);
}

void bresenhamLine(SDL_Renderer* renderer, int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        drawPixel(renderer, x1, y1);
        if (x1 == x2 && y1 == y2)
            break;

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
    int n = vertices.size();
    for (int i = 0; i < n; ++i) {
        int x1 = int(vertices[i].x);
        int y1 = int(vertices[i].y);
        int x2 = int(vertices[(i + 1) % n].x);
        int y2 = int(vertices[(i + 1) % n].y);
        bresenhamLine(renderer, x1, y1, x2, y2);
    }
}

struct Face {
    std::vector<std::array<int, 3>> vertexIndices;
};

bool loadOBJ(const std::string& path, std::vector<glm::vec3>& out_vertices, std::vector<Face>& out_faces) {
    out_vertices.clear();
    out_faces.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;

        if (keyword == "v") {
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            out_vertices.push_back(vertex);
        }
        else if (keyword == "f") {
            Face face;
            std::string indexStr;
            while (iss >> indexStr) {
                std::array<int, 3> indices;
                std::istringstream indexStream(indexStr);
                char slash;

                indexStream >> indices[0];
                if (indexStream.peek() == '/') {
                    indexStream >> slash;
                    if (indexStream.peek() != '/') {
                        indexStream >> indices[1];
                    }
                    if (indexStream.peek() == '/') {
                        indexStream >> slash >> indices[2];
                    }
                }

                face.vertexIndices.push_back(indices);
            }

            out_faces.push_back(face);
        }
    }

    return true;
}

std::vector<glm::vec3> setupVertexArray(const std::vector<glm::vec3>& vertices, const std::vector<Face>& faces)
{
    std::vector<glm::vec3> vertexArray;
    
    // For each face
    for (const auto& face : faces)
    {
        // For each vertex in the face
        for (const auto& vertexIndices : face.vertexIndices)
        {
            // Get the vertex position and normal from the input arrays using the indices from the face
            glm::vec3 vertexPosition = vertices[vertexIndices[0] - 1]; // Adjust for 1-based indexing in OBJ files

            // Add the vertex position and normal to the vertex array
            vertexArray.push_back(vertexPosition);
        }
    }

    return vertexArray;
}

// ...

int main() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow("OBJ NAVE ESPACIAL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    bool isRunning = true;
    SDL_Event event;

    std::vector<glm::vec3> vertices;
    std::vector<Face> faces;

    bool success = loadOBJ("../nave_espacial.obj", vertices, faces);
    if (!success) {
        std::cerr << "Failed to load OBJ file." << std::endl;
        return -1;
    }

    while (isRunning) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                isRunning = false;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Encontrar el bounding box (caja delimitadora) del modelo
        float minX = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::min();
        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::min();
        for (const auto& vertex : vertices) {
            minX = std::min(minX, vertex.x);
            maxX = std::max(maxX, vertex.x);
            minY = std::min(minY, vertex.y);
            maxY = std::max(maxY, vertex.y);
        }

        // Calcular el tamaño del modelo
        float modelWidth = maxX - minX;
        float modelHeight = maxY - minY;

        // Calcular el factor de escala para ajustar el modelo dentro de la ventana
        float scaleX = WINDOW_WIDTH / modelWidth;
        float scaleY = WINDOW_HEIGHT / modelHeight;
        float scale = 60.0f;


        // Calcular la translación para centrar y espejar horizontalmente el modelo en la ventana
        float translationX = (WINDOW_WIDTH - modelWidth * scale) * 0.5f;
        float translationY = (WINDOW_HEIGHT - modelHeight * scale) * 0.5f;

        for (const auto& face : faces) {
            std::vector<glm::vec3> polygonVertices;
            for (const auto& index : face.vertexIndices) {
                glm::vec3 vertex = vertices[index[0] - 1]; // Adjust for 1-based indexing in OBJ files
                vertex.x = (modelWidth - (vertex.x - minX)) * scale + translationX; // Espejar horizontalmente
                vertex.y = (vertex.y - minY) * scale + translationY;
                polygonVertices.push_back(vertex);
            }

            drawPolygon(renderer, polygonVertices);
        }

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}