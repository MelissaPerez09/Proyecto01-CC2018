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
float rotationAngles[3] = { 180.0f, 0.0f, 0.0f };

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
    std::array<int, 3> vertexIndices;
    std::array<int, 3> normalIndices;
    std::array<int, 3> textureIndices;
};


bool loadOBJ(const std::string& path, std::vector<glm::vec3>& out_vertices, std::vector<glm::vec3>& out_normals, std::vector<Face>& out_faces) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cout << "Error: Could not open file " << path << std::endl;
        return false;
    }

    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec3> temp_normals;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            temp_vertices.push_back(vertex);
        } else if (type == "vn") {
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            temp_normals.push_back(normal);
        }else if (type == "f") {
            Face f;
            for (int i = 0; i < 3; i++) {
                std::string faceGroupStr;
                iss >> faceGroupStr;

                std::istringstream groupStream(faceGroupStr);
                std::string vertexIndexStr, textureIndexStr, normalIndexStr;
                getline(groupStream, vertexIndexStr, '/');
                getline(groupStream, textureIndexStr, '/');
                getline(groupStream, normalIndexStr, '/');

                f.vertexIndices[i] = std::stoi(vertexIndexStr) - 1;
                // Ignoramos el índice de textura, ya que no lo estamos usando
                f.normalIndices[i] = std::stoi(normalIndexStr) - 1;
            }
            out_faces.push_back(f);
        }

    }
    for (const auto& face : out_faces) {
        std::cout << "Face:\n";
        std::cout << "Vertex Indices: ";
        for (int i = 0; i < 3; i++) {
            std::cout << face.vertexIndices[i] << " ";
        }
        std::cout << "\nNormal Indices: ";
        for (int i = 0; i < 3; i++) {
            std::cout << face.normalIndices[i] << " ";
        }
        std::cout << "\n\n";
    }

    out_vertices = temp_vertices;
    out_normals = temp_normals;
    return true;
}

std::vector<glm::vec3> setupVertexArray(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, const std::vector<Face>& faces)
{
    std::vector<glm::vec3> vertexArray;

    // For each face
    for (const auto& face : faces)
    {
        for (int i = 0; i < 3; ++i)
        {
            // Get the vertex position
            glm::vec3 vertexPosition = vertices[face.vertexIndices[i]];

            // Get the normal for the current vertex
            glm::vec3 vertexNormal = normals[face.normalIndices[i]];

            // Add the vertex position and normal to the vertex array
            vertexArray.push_back(vertexPosition);
            vertexArray.push_back(vertexNormal);
        }
    }
    std::cout << "Vertex Array Size: " << vertexArray.size() << std::endl;
    return vertexArray;
}

void SDL_RenderFillConvexPolygon(SDL_Renderer* renderer, const SDL_Point* points, int numPoints) {
    if (numPoints < 3) {
        // A polygon needs at least 3 points to be drawn
        return;
    }

    for (int y = 0; y < WINDOW_HEIGHT; ++y) {
        std::vector<int> intersections;

        for (int i = 0; i < numPoints; ++i) {
            int j = (i + 1) % numPoints;
            int y0 = points[i].y;
            int y1 = points[j].y;
            int x0 = points[i].x;
            int x1 = points[j].x;

            if ((y0 <= y && y1 > y) || (y0 > y && y1 <= y)) {
                int xIntersect = static_cast<int>((x0 * (y1 - y) + x1 * (y - y0)) / (y1 - y0));
                intersections.push_back(xIntersect);
            }
        }

        std::sort(intersections.begin(), intersections.end());

        for (size_t i = 0; i < intersections.size(); i += 2) {
            int startX = intersections[i];
            int endX = intersections[i + 1];

            SDL_RenderDrawLine(renderer, startX, y, endX, y);
        }
    }
}

void drawFilledPolygon(SDL_Renderer* renderer, const std::vector<glm::vec3>& vertices) {
    if (vertices.size() < 3) {
        // A polygon needs at least 3 vertices to be drawn
        return;
    }

    // Prepare an array of SDL points for rendering
    std::vector<SDL_Point> sdlPoints(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        sdlPoints[i] = { static_cast<int>(vertices[i].x), static_cast<int>(vertices[i].y) };
    }

    // Render the filled polygon
    SDL_RenderFillConvexPolygon(renderer, sdlPoints.data(), sdlPoints.size());
}

//camara
const float cameraSpeed = 0.05f;
glm::vec3 cameraPosition(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);


int main() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow("OBJ NAVE ESPACIAL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    bool isRunning = true;
    SDL_Event event;

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<Face> faces;

    bool success = loadOBJ("./cubo.obj", vertices, normals, faces);
    if (!success) {
        std::cerr << "Failed to load OBJ file." << std::endl;
        return -1;
    }

    std::vector<glm::vec3> vertexArray = setupVertexArray(vertices, normals, faces);

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
        float scale = 120.0f;


        // Calcular la translación para centrar y espejar horizontalmente el modelo en la ventana
        float translationX = (WINDOW_WIDTH - modelWidth * scale) * 0.5f;
        float translationY = (WINDOW_HEIGHT - modelHeight * scale) * 0.5f;

        glm::mat4 viewMatrix = glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);
        glm::mat4 modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, glm::vec3(translationX, translationY, 0.0f));
        modelMatrix = glm::rotate(modelMatrix, glm::radians(rotationAngles[0]), glm::vec3(0.0f, 1.0f, 0.0f));
        modelMatrix = glm::scale(modelMatrix, glm::vec3(scale, scale, 1.0f));
        glm::mat4 transformationMatrix = modelMatrix * viewMatrix;
        glm::mat4 projectionMatrix = glm::perspective(glm::radians(45.0f), float(WINDOW_WIDTH) / float(WINDOW_HEIGHT), 0.1f, 100.0f);

        glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotationAngles[0]), glm::vec3(0.0f, 1.0f, 0.0f)); // Rotación en el eje X

        for (const auto& face : faces) {
            std::vector<glm::vec3> polygonVertices;

            for (int i = 0; i < 3; ++i) {
                int vertexIndex = face.vertexIndices[i];
                glm::vec3 vertex = vertices[vertexIndex];
                polygonVertices.push_back(vertex);
            }

            // Calculate face normal by averaging vertex normals
            glm::vec3 normal = glm::normalize(glm::cross(polygonVertices[1] - polygonVertices[0], polygonVertices[2] - polygonVertices[0]));

            // Calculate vector to light source (assuming a directional light)
            glm::vec3 lightDirection(0.0f, 0.0f, -1.0f); // Example light direction

            // Calculate lighting factor using Lambertian shading model and averaged normal
            float lightingFactor = glm::max(glm::dot(normal, -lightDirection), 0.0f);

            // Calculate grayscale color based on lighting factor
            uint8_t shade = static_cast<uint8_t>(255.0f * lightingFactor);

            // Set shading color (using grayscale color)
            SDL_SetRenderDrawColor(renderer, shade, shade, shade, 255);

            std::vector<glm::vec3> transformedVertices;
            for (const auto& vertex : polygonVertices) {
                glm::vec4 transformedVertex = transformationMatrix * glm::vec4(vertex, 1.0f);
                transformedVertices.push_back(glm::vec3(transformedVertex / transformedVertex.w));
            }

            // Proyecta los vértices en la ventana
            std::vector<SDL_Point> projectedPoints;
            for (const auto& vertex : transformedVertices) {
                glm::vec4 projectedVertex = projectionMatrix * glm::vec4(vertex, 1.0f);
                projectedPoints.push_back({ int(projectedVertex.x), int(projectedVertex.y) });
            }

            // Draw filled polygon using the calculated shading
            drawFilledPolygon(renderer, transformedVertices);
        }

        SDL_RenderPresent(renderer);

        for (int i = 0; i < 3; ++i) {
            rotationAngles[i] += 0.5f; 
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}